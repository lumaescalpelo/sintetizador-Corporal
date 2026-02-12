#include <Wire.h>

#define SDA_A 21
#define SCL_A 22
#define SDA_B 16
#define SCL_B 17

TwoWire I2C_B = TwoWire(1);

struct IMUInfo {
  bool present = false;
  bool isMPU6050 = false;
  bool isICM20948 = false;
  uint8_t addr = 0;
};

// ---------- Helpers I2C ----------
bool write8(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  return bus.endTransmission() == 0;
}

bool readN(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom((int)addr, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = bus.read();
  return true;
}

bool read8(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t &val) {
  return readN(bus, addr, reg, &val, 1);
}

bool ping(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

// ---------- AHT2X (AHT20/AHT21) ----------
bool aht_soft_reset(TwoWire &bus) { return write8(bus, 0x38, 0xBA, 0x00); }

bool aht_init(TwoWire &bus) {
  if (!ping(bus, 0x38)) return false;
  // Init cmd: 0xBE 0x08 0x00 (típico)
  bus.beginTransmission(0x38);
  bus.write(0xBE); bus.write(0x08); bus.write(0x00);
  if (bus.endTransmission() != 0) return false;
  delay(20);
  return true;
}

bool aht_read(TwoWire &bus, float &tempC, float &rh) {
  if (!ping(bus, 0x38)) return false;

  // Trigger measurement: 0xAC 0x33 0x00
  bus.beginTransmission(0x38);
  bus.write(0xAC); bus.write(0x33); bus.write(0x00);
  if (bus.endTransmission() != 0) return false;

  delay(80);

  uint8_t data[6];
  if (!readN(bus, 0x38, 0x00, data, 6)) return false;

  // bit7 of status indicates busy
  if (data[0] & 0x80) return false;

  uint32_t rawHum = ((uint32_t)(data[1]) << 12) | ((uint32_t)(data[2]) << 4) | ((data[3] >> 4) & 0x0F);
  uint32_t rawTmp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)(data[4]) << 8) | data[5];

  rh = (rawHum * 100.0f) / 1048576.0f;                 // 2^20
  tempC = ((rawTmp * 200.0f) / 1048576.0f) - 50.0f;    // -50..150

  return true;
}

// ---------- ENS160 ----------
uint8_t ens_find_addr(TwoWire &bus) {
  if (ping(bus, 0x52)) return 0x52;
  if (ping(bus, 0x53)) return 0x53;
  return 0x00;
}

bool ens_write_opmode(TwoWire &bus, uint8_t addr, uint8_t mode) {
  // ENS160: OPMODE reg 0x10
  return write8(bus, addr, 0x10, mode);
}

bool ens_init(TwoWire &bus, uint8_t addr) {
  if (!addr) return false;

  // Soft reset: reg 0x12, value 0x00 (muchos drivers usan esto)
  write8(bus, addr, 0x12, 0x00);
  delay(20);

  // Set standard mode (0x02) en OPMODE
  if (!ens_write_opmode(bus, addr, 0x02)) return false;
  delay(50);
  return true;
}

bool ens_read_basic(TwoWire &bus, uint8_t addr, uint8_t &aqi, uint16_t &tvoc, uint16_t &eco2) {
  if (!addr) return false;

  // AQI: 0x21 (1 byte), TVOC: 0x22 (2 bytes LE), eCO2: 0x24 (2 bytes LE)
  uint8_t b;
  uint8_t buf2[2];

  if (!read8(bus, addr, 0x21, b)) return false;
  aqi = b;

  if (!readN(bus, addr, 0x22, buf2, 2)) return false;
  tvoc = (uint16_t)buf2[1] << 8 | buf2[0];

  if (!readN(bus, addr, 0x24, buf2, 2)) return false;
  eco2 = (uint16_t)buf2[1] << 8 | buf2[0];

  return true;
}

// ---------- MPU-6050 ----------
bool mpu_init(TwoWire &bus, uint8_t addr) {
  // Wake up: PWR_MGMT_1 (0x6B) = 0
  if (!write8(bus, addr, 0x6B, 0x00)) return false;
  delay(10);
  // Gyro ±250 dps (0x1B=0), Accel ±2g (0x1C=0)
  write8(bus, addr, 0x1B, 0x00);
  write8(bus, addr, 0x1C, 0x00);
  return true;
}

bool mpu_read_ag(TwoWire &bus, uint8_t addr, float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps, float &gz_dps) {
  uint8_t buf[14];
  if (!readN(bus, addr, 0x3B, buf, 14)) return false;

  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  // temp buf[6..7] ignored
  int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

  // scaling: ±2g => 16384 LSB/g, ±250 dps => 131 LSB/(dps)
  ax_g = ax / 16384.0f;
  ay_g = ay / 16384.0f;
  az_g = az / 16384.0f;
  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;

  return true;
}

// ---------- ICM-20948 (solo accel/gyro, banco 0) ----------
// ICM usa bancos. Para leer accel/gyro en banco 0, hay que configurar.
// Este init mínimo intenta ponerlo en modo normal.
bool icm_select_bank(TwoWire &bus, uint8_t addr, uint8_t bank) {
  // REG_BANK_SEL = 0x7F, bits [5:4] bank
  return write8(bus, addr, 0x7F, (bank << 4));
}

bool icm_init(TwoWire &bus, uint8_t addr) {
  // Banco 0
  if (!icm_select_bank(bus, addr, 0)) return false;

  // Wake up: PWR_MGMT_1 = 0x06 -> clock auto, disable sleep (varía por driver; usamos 0x01 también suele servir)
  // Intento conservador: 0x01 (auto clock), sleep off
  write8(bus, addr, 0x06, 0x01);
  delay(10);

  // Disable I2C master mode etc (muchos breakouts ya lo hacen). Lo dejamos mínimo.
  return true;
}

bool icm_read_ag(TwoWire &bus, uint8_t addr, float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps, float &gz_dps) {
  // Banco 0: ACCEL_XOUT_H = 0x2D (6 bytes), GYRO_XOUT_H = 0x33 (6 bytes)
  if (!icm_select_bank(bus, addr, 0)) return false;

  uint8_t bufA[6], bufG[6];
  if (!readN(bus, addr, 0x2D, bufA, 6)) return false;
  if (!readN(bus, addr, 0x33, bufG, 6)) return false;

  int16_t ax = (int16_t)((bufA[0] << 8) | bufA[1]);
  int16_t ay = (int16_t)((bufA[2] << 8) | bufA[3]);
  int16_t az = (int16_t)((bufA[4] << 8) | bufA[5]);

  int16_t gx = (int16_t)((bufG[0] << 8) | bufG[1]);
  int16_t gy = (int16_t)((bufG[2] << 8) | bufG[3]);
  int16_t gz = (int16_t)((bufG[4] << 8) | bufG[5]);

  // Asumimos escalas por defecto (depende de config real):
  // Accel ±2g: 16384 LSB/g (en ICM puede ser distinto si cambias FS)
  // Gyro ±250 dps: 131 LSB/dps (ICM difiere; por defecto puede ser 131.2-ish, esto es aproximado)
  ax_g = ax / 16384.0f;
  ay_g = ay / 16384.0f;
  az_g = az / 16384.0f;
  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;

  return true;
}

// ---------- Detección IMUs ----------
IMUInfo detectIMU(TwoWire &bus, uint8_t addr) {
  IMUInfo info;
  info.addr = addr;

  if (!ping(bus, addr)) return info;

  // MPU-6050 WHO_AM_I @ 0x75 -> 0x68/0x69
  uint8_t who_mpu;
  if (read8(bus, addr, 0x75, who_mpu)) {
    if (who_mpu == 0x68 || who_mpu == 0x69) {
      info.present = true;
      info.isMPU6050 = true;
      return info;
    }
  }

  // ICM-20948 WHO_AM_I @ 0x00 -> 0xEA
  uint8_t who_icm;
  if (read8(bus, addr, 0x00, who_icm)) {
    if (who_icm == 0xEA) {
      info.present = true;
      info.isICM20948 = true;
      return info;
    }
  }

  // Algo en 0x68/0x69, pero no identificado
  info.present = true;
  return info;
}

void printIMU(TwoWire &bus, const char* busName, IMUInfo imu) {
  if (!imu.present) return;

  Serial.print(busName);
  Serial.print("  IMU 0x");
  Serial.print(imu.addr, HEX);
  Serial.print(" -> ");

  if (imu.isMPU6050) Serial.println("MPU-6050");
  else if (imu.isICM20948) Serial.println("ICM-20948");
  else Serial.println("IMU desconocida");

  float ax, ay, az, gx, gy, gz;
  bool ok = false;

  if (imu.isMPU6050) {
    mpu_init(bus, imu.addr);
    ok = mpu_read_ag(bus, imu.addr, ax, ay, az, gx, gy, gz);
  } else if (imu.isICM20948) {
    icm_init(bus, imu.addr);
    ok = icm_read_ag(bus, imu.addr, ax, ay, az, gx, gy, gz);
  }

  if (ok) {
    Serial.print("    Accel[g]: ");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.print(az, 3);
    Serial.print(" | Gyro[dps]: ");
    Serial.print(gx, 2); Serial.print(", ");
    Serial.print(gy, 2); Serial.print(", ");
    Serial.println(gz, 2);
  } else {
    Serial.println("    No pude leer accel/gyro (config/registro diferente o sensor no inicializado).");
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Wire.begin(SDA_A, SCL_A);
  I2C_B.begin(SDA_B, SCL_B);

  Serial.println("Lectura base de sensores en 2 buses I2C.");

  // ENS160 init en ambos buses (por si está en uno u otro)
  uint8_t ensA = ens_find_addr(Wire);
  uint8_t ensB = ens_find_addr(I2C_B);
  if (ensA) { ens_init(Wire, ensA); }
  if (ensB) { ens_init(I2C_B, ensB); }

  // AHT init en ambos buses (si solo está en uno, el otro falla sin drama)
  aht_soft_reset(Wire);  aht_init(Wire);
  aht_soft_reset(I2C_B); aht_init(I2C_B);
}

void loop() {
  // ----- AHT2X -----
  float tC, rh;
  bool ahtA = aht_read(Wire, tC, rh);
  if (ahtA) {
    Serial.print("I2C A AHT2X: ");
    Serial.print(tC, 2); Serial.print(" C, ");
    Serial.print(rh, 1); Serial.println(" %");
  }

  bool ahtB = aht_read(I2C_B, tC, rh);
  if (ahtB) {
    Serial.print("I2C B AHT2X: ");
    Serial.print(tC, 2); Serial.print(" C, ");
    Serial.print(rh, 1); Serial.println(" %");
  }

  // ----- ENS160 -----
  uint8_t addrENS;

  addrENS = ens_find_addr(Wire);
  if (addrENS) {
    uint8_t aqi; uint16_t tvoc, eco2;
    if (ens_read_basic(Wire, addrENS, aqi, tvoc, eco2)) {
      Serial.print("I2C A ENS160(0x"); Serial.print(addrENS, HEX); Serial.print("): ");
      Serial.print("AQI="); Serial.print(aqi);
      Serial.print(" TVOC="); Serial.print(tvoc); Serial.print(" ppb");
      Serial.print(" eCO2="); Serial.print(eco2); Serial.println(" ppm");
    } else {
      Serial.print("I2C A ENS160(0x"); Serial.print(addrENS, HEX); Serial.println("): no pude leer datos");
    }
  }

  addrENS = ens_find_addr(I2C_B);
  if (addrENS) {
    uint8_t aqi; uint16_t tvoc, eco2;
    if (ens_read_basic(I2C_B, addrENS, aqi, tvoc, eco2)) {
      Serial.print("I2C B ENS160(0x"); Serial.print(addrENS, HEX); Serial.print("): ");
      Serial.print("AQI="); Serial.print(aqi);
      Serial.print(" TVOC="); Serial.print(tvoc); Serial.print(" ppb");
      Serial.print(" eCO2="); Serial.print(eco2); Serial.println(" ppm");
    } else {
      Serial.print("I2C B ENS160(0x"); Serial.print(addrENS, HEX); Serial.println("): no pude leer datos");
    }
  }

  // ----- IMUs (0x68/0x69) -----
  IMUInfo imuA_68 = detectIMU(Wire, 0x68);
  IMUInfo imuA_69 = detectIMU(Wire, 0x69);
  IMUInfo imuB_68 = detectIMU(I2C_B, 0x68);
  IMUInfo imuB_69 = detectIMU(I2C_B, 0x69);

  printIMU(Wire,  "I2C A", imuA_68);
  printIMU(Wire,  "I2C A", imuA_69);
  printIMU(I2C_B, "I2C B", imuB_68);
  printIMU(I2C_B, "I2C B", imuB_69);

  Serial.println("-------------------------");
  delay(1000);
}
