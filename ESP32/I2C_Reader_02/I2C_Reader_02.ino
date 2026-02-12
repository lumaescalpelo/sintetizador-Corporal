#include <Wire.h>

// ====== BUSES ======
TwoWire I2C_A = TwoWire(0);
TwoWire I2C_B = TwoWire(1);

static const int SDA_A = 21, SCL_A = 22;
static const int SDA_B = 16, SCL_B = 17;

// ====== ADDRESSES (los tuyos) ======
static const uint8_t ENS160_ADDR = 0x53; // tu scan lo ve en 0x53
// AHT2X suele ser 0x38, tu scan lo ve en 0x38

// ====== ENS160 REGISTERS ======
static const uint8_t ENS_PART_ID_L   = 0x00; // 2 bytes, little endian (0x60,0x01)
static const uint8_t ENS_OPMODE      = 0x10; // 0x01 idle, 0x02 standard, 0xF0 reset
static const uint8_t ENS_TEMP_IN     = 0x13; // 2 bytes (Kelvin * 64), little endian
static const uint8_t ENS_RH_IN       = 0x15; // 2 bytes (%RH * 512), little endian
static const uint8_t ENS_DEVICE_STAT = 0x20; // status bits
static const uint8_t ENS_DATA_AQI    = 0x21; // 1 byte
static const uint8_t ENS_DATA_TVOC   = 0x22; // 2 bytes, little endian (ppb)
static const uint8_t ENS_DATA_ECO2   = 0x24; // 2 bytes, little endian (ppm)
static const uint8_t ENS_GPR_READ    = 0x48; // 8 bytes

// DEVICE_STATUS bits (datasheet)
static inline bool ens_STATAS(uint8_t s) { return (s >> 7) & 0x01; }
static inline bool ens_STATER(uint8_t s) { return (s >> 6) & 0x01; }
static inline uint8_t ens_VALIDITY(uint8_t s) { return (s >> 2) & 0x03; } // 0..3
static inline bool ens_NEWDAT(uint8_t s) { return (s >> 1) & 0x01; }
static inline bool ens_NEWGPR(uint8_t s) { return (s >> 0) & 0x01; }

// ====== Simple I2C helpers ======
bool i2cWriteBytes(TwoWire &bus, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) {
  bus.beginTransmission(addr);
  bus.write(reg);
  for (size_t i = 0; i < len; i++) bus.write(data[i]);
  return (bus.endTransmission() == 0);
}

bool i2cReadBytes(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *out, size_t len) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false; // repeated start
  size_t got = bus.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) out[i] = bus.read();
  return true;
}

bool i2cReadU8(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t &v) {
  return i2cReadBytes(bus, addr, reg, &v, 1);
}

bool i2cReadU16LE(TwoWire &bus, uint8_t addr, uint8_t reg, uint16_t &v) {
  uint8_t b[2];
  if (!i2cReadBytes(bus, addr, reg, b, 2)) return false;
  v = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  return true;
}

bool i2cWriteU8(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t v) {
  return i2cWriteBytes(bus, addr, reg, &v, 1);
}

bool i2cWriteU16LE(TwoWire &bus, uint8_t addr, uint8_t reg, uint16_t v) {
  uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
  return i2cWriteBytes(bus, addr, reg, b, 2);
}

// ====== AHT2X minimal read (sin librerías) ======
// Basado en el comando de medición estándar de AHT20/AHT21:
// 1) init (0xBE 0x08 0x00) opcional
// 2) trigger measure (0xAC 0x33 0x00)
// 3) esperar ~80ms, leer 6 bytes
// Nota: esto es “lo suficiente” para pruebas.
static const uint8_t AHT_ADDR = 0x38;

bool ahtTriggerMeasure(TwoWire &bus) {
  uint8_t cmd[3] = { 0xAC, 0x33, 0x00 };
  return i2cWriteBytes(bus, AHT_ADDR, cmd[0], &cmd[1], 2); // escribe 0xAC y luego 2 bytes
}

bool ahtRead(TwoWire &bus, float &tempC, float &rh) {
  if (!ahtTriggerMeasure(bus)) return false;
  delay(90);

  uint8_t d[6];
  // lectura directa sin “reg”, el AHT entrega el buffer
  bus.requestFrom((int)AHT_ADDR, 6);
  if (bus.available() != 6) return false;
  for (int i = 0; i < 6; i++) d[i] = bus.read();

  // d[0] status, d[1..5] payload
  uint32_t rawHum = ((uint32_t)(d[1]) << 12) | ((uint32_t)(d[2]) << 4) | ((uint32_t)(d[3] >> 4) & 0x0F);
  uint32_t rawTmp = ((uint32_t)(d[3] & 0x0F) << 16) | ((uint32_t)(d[4]) << 8) | (uint32_t)d[5];

  rh = (rawHum * 100.0f) / 1048576.0f;         // 2^20
  tempC = ((rawTmp * 200.0f) / 1048576.0f) - 50.0f;

  return true;
}

// ====== ENS160 logic ======
bool ensReadPartID(TwoWire &bus, uint16_t &part) {
  return i2cReadU16LE(bus, ENS160_ADDR, ENS_PART_ID_L, part);
}

bool ensReadStatus(TwoWire &bus, uint8_t &st) {
  return i2cReadU8(bus, ENS160_ADDR, ENS_DEVICE_STAT, st);
}

bool ensClearNEWGPR(TwoWire &bus) {
  uint8_t gpr[8];
  return i2cReadBytes(bus, ENS160_ADDR, ENS_GPR_READ, gpr, 8); // leerlo lo limpia
}

bool ensSetCompensation(TwoWire &bus, float tempC, float rh) {
  // TEMP_IN = Kelvin * 64 ; Kelvin = C + 273.15
  float k = tempC + 273.15f;
  uint16_t temp_in = (uint16_t)(k * 64.0f + 0.5f);

  // RH_IN = %RH * 512
  if (rh < 0) rh = 0;
  if (rh > 100) rh = 100;
  uint16_t rh_in = (uint16_t)(rh * 512.0f + 0.5f);

  bool ok1 = i2cWriteU16LE(bus, ENS160_ADDR, ENS_TEMP_IN, temp_in);
  bool ok2 = i2cWriteU16LE(bus, ENS160_ADDR, ENS_RH_IN, rh_in);
  return ok1 && ok2;
}

bool ensSetOpMode(TwoWire &bus, uint8_t mode) {
  return i2cWriteU8(bus, ENS160_ADDR, ENS_OPMODE, mode);
}

bool ensReadData(TwoWire &bus, uint8_t &aqi, uint16_t &tvoc, uint16_t &eco2) {
  if (!i2cReadU8(bus, ENS160_ADDR, ENS_DATA_AQI, aqi)) return false;
  if (!i2cReadU16LE(bus, ENS160_ADDR, ENS_DATA_TVOC, tvoc)) return false;
  if (!i2cReadU16LE(bus, ENS160_ADDR, ENS_DATA_ECO2, eco2)) return false;
  return true;
}

bool ensInitAndStart(TwoWire &bus, float tempC, float rh) {
  // 1) poner IDLE
  if (!ensSetOpMode(bus, 0x01)) return false;
  delay(5);

  // 2) leer GPR_READ para limpiar NEWGPR si viene prendido de fábrica/arranque
  ensClearNEWGPR(bus);

  // 3) escribir compensación en el formato correcto
  ensSetCompensation(bus, tempC, rh);

  // 4) pasar a STANDARD
  if (!ensSetOpMode(bus, 0x02)) return false;

  // 5) esperar a que realmente “corra”: STATAS=1
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    uint8_t st;
    if (ensReadStatus(bus, st)) {
      if (ens_STATAS(st)) return true;
    }
    delay(50);
  }
  return false;
}

bool ensHardReset(TwoWire &bus) {
  // datasheet: OPMODE=0xF0 => RESET
  bool ok = ensSetOpMode(bus, 0xF0);
  delay(10);
  return ok;
}

void printENSStatus(uint8_t st) {
  Serial.print("STATUS=0x");
  if (st < 16) Serial.print("0");
  Serial.print(st, HEX);

  Serial.print(" STATAS=");
  Serial.print(ens_STATAS(st));

  Serial.print(" STATER=");
  Serial.print(ens_STATER(st));

  Serial.print(" VALID=");
  Serial.print(ens_VALIDITY(st)); // 0 normal,1 warmup,2 initial startup,3 invalid :contentReference[oaicite:7]{index=7}

  Serial.print(" NEWDAT=");
  Serial.print(ens_NEWDAT(st));

  Serial.print(" NEWGPR=");
  Serial.print(ens_NEWGPR(st));
}

void setup() {
  Serial.begin(115200);
  delay(200);

  I2C_A.begin(SDA_A, SCL_A, 400000);
  I2C_B.begin(SDA_B, SCL_B, 400000);

  Serial.println("\n--- Boot ---");

  // Prueba rápida AHT para tener T/RH
  float tC = NAN, rh = NAN;
  if (ahtRead(I2C_B, tC, rh)) {
    Serial.print("AHT2X(B): ");
    Serial.print(tC, 2);
    Serial.print(" C, ");
    Serial.print(rh, 1);
    Serial.println(" %");
  } else {
    Serial.println("AHT2X(B): no pude leer (sin T/RH no hay compensación decente).");
    tC = 25.0f;
    rh = 50.0f;
  }

  // ENS160
  uint16_t part = 0;
  if (!ensReadPartID(I2C_B, part)) {
    Serial.println("ENS160(B): no responde en I2C.");
    return;
  }

  Serial.print("ENS160(B): PART_ID=0x");
  Serial.println(part, HEX);

  // Init + start
  if (!ensInitAndStart(I2C_B, tC, rh)) {
    Serial.println("ENS160(B): no arrancó el OPMODE. Haciendo RESET y reintento...");
    ensHardReset(I2C_B);
    delay(50);

    // releer AHT por si cambió
    ahtRead(I2C_B, tC, rh);

    if (!ensInitAndStart(I2C_B, tC, rh)) {
      Serial.println("ENS160(B): sigue sin arrancar. Algo eléctrico/board-level está raro (CSn/ADDR/power).");
    }
  }

  Serial.println("--- Setup listo ---");
}

void loop() {
  float tC = NAN, rh = NAN;
  bool ahtOK = ahtRead(I2C_B, tC, rh);

  if (ahtOK) {
    Serial.print("AHT2X(B): ");
    Serial.print(tC, 2); Serial.print(" C, ");
    Serial.print(rh, 1); Serial.println(" %");
    ensSetCompensation(I2C_B, tC, rh);
  } else {
    Serial.println("AHT2X(B): fallo lectura");
  }

  uint8_t st = 0x00;
  if (!ensReadStatus(I2C_B, st)) {
    Serial.println("ENS160(B): fallo leyendo DEVICE_STATUS");
    delay(1000);
    return;
  }

  Serial.print("ENS160(B) ");
  printENSStatus(st);
  Serial.println();

  uint8_t valid = (st >> 2) & 0x03;
  bool inWarmup = (valid == 1 || valid == 2);

  if (ens_NEWGPR(st)) {
    ensClearNEWGPR(I2C_B);
    ensReadStatus(I2C_B, st);
  }

  if (ens_NEWDAT(st) && ens_STATAS(st)) {
    uint8_t aqi;
    uint16_t tvoc, eco2;
    if (ensReadData(I2C_B, aqi, tvoc, eco2)) {
      Serial.print("ENS160(B): AQI="); Serial.print(aqi);
      Serial.print(" TVOC="); Serial.print(tvoc);
      Serial.print("ppb eCO2="); Serial.print(eco2);
      Serial.println("ppm");
    } else {
      Serial.println("ENS160(B): fallo leyendo DATA_x");
    }
  }

  // Debug extra cuando está “muerto”
  if (!ens_STATAS(st) && !inWarmup) {
    uint8_t op = 0xFF;
    uint16_t part = 0;
    i2cReadU8(I2C_B, ENS160_ADDR, ENS_OPMODE, op);
    i2cReadU16LE(I2C_B, ENS160_ADDR, ENS_PART_ID_L, part);

    Serial.print("DBG ENS160: PART_ID=0x"); Serial.print(part, HEX);
    Serial.print(" OPMODE=0x"); Serial.println(op, HEX);
  }

  static uint32_t lastKick = 0;
  if (!ens_STATAS(st) && !inWarmup) {
    if (millis() - lastKick > 5000) {
      lastKick = millis();
      Serial.println("ENS160(B): STATAS=0 fuera de warm-up → re-init OPMODE");
      if (ahtOK) ensInitAndStart(I2C_B, tC, rh);
      else       ensInitAndStart(I2C_B, 25.0f, 50.0f);
    }
  }

  Serial.println("-------------------------");
  delay(1000);
}
