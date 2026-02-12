#include <Wire.h>

#define SDA_A 21
#define SCL_A 22
#define SDA_B 16
#define SCL_B 17

TwoWire I2C_B = TwoWire(1);

bool read8(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t &val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false; // repeated start
  if (bus.requestFrom((int)addr, 1) != 1) return false;
  val = bus.read();
  return true;
}

bool read16_LE(TwoWire &bus, uint8_t addr, uint8_t reg, uint16_t &val) {
  // Lee 2 bytes little-endian desde reg
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom((int)addr, 2) != 2) return false;
  uint8_t lo = bus.read();
  uint8_t hi = bus.read();
  val = (uint16_t)hi << 8 | lo;
  return true;
}

const char* identifyDevice(TwoWire &bus, uint8_t addr) {
  // Identificación por direcciones casi fijas
  if (addr == 0x38) return "AHT2X (AHT20/AHT21)";

  // ENS160: 0x52 o 0x53, intentar leer PART_ID
  if (addr == 0x52 || addr == 0x53) {
    // En muchos drivers, PART_ID está en 0x00 (2 bytes)
    uint16_t part;
    if (read16_LE(bus, addr, 0x00, part)) {
      // ENS160 suele reportar 0x0160
      if (part == 0x0160 || (part & 0x00FF) == 0x60) return "ENS160 (air quality)";
      return "Dispositivo en 0x52/0x53 (posible ENS160)";
    }
    return "Dispositivo en 0x52/0x53 (posible ENS160, sin ID)";
  }

  // IMUs: 0x68/0x69
  if (addr == 0x68 || addr == 0x69) {
    // MPU-6050 WHO_AM_I está en 0x75
    uint8_t who_mpu;
    if (read8(bus, addr, 0x75, who_mpu)) {
      if (who_mpu == 0x68 || who_mpu == 0x69) return "MPU-6050 (HW-290 típico)";
    }

    // ICM-20948 WHO_AM_I está en 0x00 y normalmente devuelve 0xEA
    uint8_t who_icm;
    if (read8(bus, addr, 0x00, who_icm)) {
      if (who_icm == 0xEA) return "ICM-20948";
    }

    return "IMU en 0x68/0x69 (no identificada)";
  }

  // 0x77 suele ser barómetro (BMP/BME/MS56xx) o similares
  if (addr == 0x77) return "Dispositivo 0x77 (típico barómetro/BMP/BME/otro)";

  return "Desconocido";
}

void scanBus(TwoWire &bus, const char *name) {
  Serial.print("\nEscaneando ");
  Serial.print(name);
  Serial.println("...");

  int devices = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();

    if (err == 0) {
      devices++;
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" -> ");
      Serial.println(identifyDevice(bus, addr));
    }
  }

  if (devices == 0) {
    Serial.println("  Ninguno detectado.");
  } else {
    Serial.print("  Total: ");
    Serial.println(devices);
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Wire.begin(SDA_A, SCL_A);
  I2C_B.begin(SDA_B, SCL_B);

  // Si hay cables largos, baja velocidad
  // Wire.setClock(100000);
  // I2C_B.setClock(100000);

  Serial.println("Escaner I2C con identificacion (2 buses).");
}

void loop() {
  scanBus(Wire,  "I2C A (SDA=21 SCL=22)");
  scanBus(I2C_B, "I2C B (SDA=16 SCL=17)");

  Serial.println("\n-------------------------");
  delay(3000);
}
