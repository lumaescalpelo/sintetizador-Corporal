#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <SparkFun_ENS160.h>

// ---- I2C dedicado ----
#define SDA_PIN 25
#define SCL_PIN 26

TwoWire I2Cens = TwoWire(0);

Adafruit_AHTX0 aht;
SparkFun_ENS160 ens;

uint8_t ensAddr = 0;

void scanI2C() {
  Serial.println("\nI2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    I2Cens.beginTransmission(addr);
    if (I2Cens.endTransmission() == 0) {
      Serial.print("  Found 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      delay(2);
    }
  }
}

bool initENS() {
  if (ens.begin(I2Cens, 0x53)) {
    ensAddr = 0x53;
    return true;
  }
  if (ens.begin(I2Cens, 0x52)) {
    ensAddr = 0x52;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== ENS160 + AHT20 (GPIO 25/26) ===");

  // Iniciar bus I2C dedicado
  I2Cens.begin(SDA_PIN, SCL_PIN);
  I2Cens.setClock(100000);  // 100kHz estable
  delay(50);

  scanI2C();

  // ---- AHT ----
  if (!aht.begin(&I2Cens)) {
    Serial.println("ERROR: AHT no detectado.");
    while (1) delay(1000);
  }
  Serial.println("AHT OK");

  // ---- ENS ----
  if (!initENS()) {
    Serial.println("ERROR: ENS160 no detectado.");
    while (1) delay(1000);
  }

  Serial.print("ENS160 OK en 0x");
  Serial.println(ensAddr, HEX);

  ens.setOperatingMode(SFE_ENS160_RESET);
  delay(20);
  ens.setOperatingMode(SFE_ENS160_STANDARD);

  Serial.println("Esperando warm-up del ENS...");
  delay(3000);  // importante: dejar estabilizar

  Serial.println("Listo.\n");
}

void loop() {
  sensors_event_t humidity, temp;

  aht.getEvent(&humidity, &temp);

  float T = temp.temperature;
  float RH = humidity.relative_humidity;

  // Compensación ambiental
  ens.setTempCompensation(T);
  ens.setRHCompensation(RH);

  uint8_t aqi = ens.getAQI();
  uint16_t tvoc = ens.getTVOC();
  uint16_t eco2 = ens.getECO2();
  uint8_t flags = ens.getFlags();

  Serial.print("T=");
  Serial.print(T, 2);
  Serial.print(" °C  RH=");
  Serial.print(RH, 2);
  Serial.print(" %  |  AQI=");
  Serial.print(aqi);
  Serial.print("  TVOC=");
  Serial.print(tvoc);
  Serial.print("  eCO2=");
  Serial.print(eco2);
  Serial.print("  flags=");
  Serial.println(flags);

  delay(2000);
}