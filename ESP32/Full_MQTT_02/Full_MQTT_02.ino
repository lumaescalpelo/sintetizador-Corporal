#include <Wire.h>
#include <WiFi.h>

#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>

#include <ArduinoJson.h>
#include <Adafruit_AHTX0.h>
#include <SparkFun_ENS160.h>

// ================= WiFi / MQTT =================
static const char* WIFI_SSID = "INFINITUMD2AC";
static const char* WIFI_PASS = "PCwGdtcV9D";

static const char* MQTT_HOST = "192.168.1.100";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_TOPIC = "sinte/cuerpo1";

static const char* DEVICE_ID = "esp32-cuerpo1";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ================= I2C Pines =================
#define SDA_A 21
#define SCL_A 22
#define SDA_B 16
#define SCL_B 17
TwoWire I2C_B = TwoWire(1);

// ================= ADC Pines =================
static const int PIN_HALL_32 = 32;
static const int PIN_MQ6_34  = 34;
static const int PIN_HALL_35 = 35;

// ================= AHT =================
Adafruit_AHTX0 ahtA;
Adafruit_AHTX0 ahtB;
bool ahtA_present = false;
bool ahtB_present = false;

// ================= ENS (SparkFun) =================
SparkFun_ENS160 ensA;
SparkFun_ENS160 ensB;
bool ensA_present = false;
bool ensB_present = false;
uint8_t ensA_addr = 0;
uint8_t ensB_addr = 0;

// ================= IMUs =================
struct IMUInfo {
  bool present = false;
  bool isMPU6050 = false;
  bool isICM20948 = false;
  uint8_t addr = 0;
};

IMUInfo imuA_68, imuA_69, imuB_68, imuB_69;

// ---------- Helpers I2C ----------
bool ping(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

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

uint8_t ens_find_addr(TwoWire &bus) {
  if (ping(bus, 0x53)) return 0x53;
  if (ping(bus, 0x52)) return 0x52;
  return 0;
}

// ---------- ADC helpers ----------
float adc_to_volts(int raw, float vref = 3.3f) {
  return (raw * vref) / 4095.0f;
}

// ---------- WiFi / MQTT helpers ----------
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("WiFi conectando a ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - t0 > 20000) {
      Serial.println("\nWiFi: timeout, reintentando...");
      WiFi.disconnect(true);
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }

  Serial.println("\nWiFi OK");
  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
}

void mqttConnect() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(5);
  mqtt.setBufferSize(1024);

  while (!mqtt.connected()) {
    uint32_t chip = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
    char clientId[24];
    snprintf(clientId, sizeof(clientId), "cuerpo1-%06lX", (unsigned long)chip);

    Serial.print("MQTT conectando a ");
    Serial.print(MQTT_HOST);
    Serial.print(":");
    Serial.print(MQTT_PORT);
    Serial.print(" como ");
    Serial.print(clientId);
    Serial.print(" ... ");

    if (mqtt.connect(clientId)) {
      Serial.println("OK");
    } else {
      Serial.print("fail rc=");
      Serial.println(mqtt.state());
      delay(1000);
    }
  }
}

// ---------- AHT read ----------
bool readAHT(Adafruit_AHTX0 &aht, float &T, float &RH) {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  T  = temp.temperature;
  RH = humidity.relative_humidity;

  if (isnan(T) || isnan(RH)) return false;
  if (RH < 0 || RH > 100) return false;
  return true;
}

// ---------- ENS read (SparkFun) ----------
bool readENS(SparkFun_ENS160 &ens, float T, float RH, uint8_t &aqi, uint16_t &tvoc, uint16_t &eco2) {
  if (!isnan(T)) ens.setTempCompensation(T);
  if (!isnan(RH)) ens.setRHCompensation(RH);

  aqi  = ens.getAQI();
  tvoc = ens.getTVOC();
  eco2 = ens.getECO2();

  // misma validación suave que tu referencia
  if (aqi > 5) return false;
  return true;
}

// ---------- IMU detect ----------
IMUInfo detectIMU(TwoWire &bus, uint8_t addr) {
  IMUInfo info;
  info.addr = addr;
  if (!ping(bus, addr)) return info;

  uint8_t who_mpu;
  if (read8(bus, addr, 0x75, who_mpu)) {
    if (who_mpu == 0x68) {
      info.present = true;
      info.isMPU6050 = true;
      return info;
    }
  }

  uint8_t who_icm;
  if (read8(bus, addr, 0x00, who_icm)) {
    if (who_icm == 0xEA) {
      info.present = true;
      info.isICM20948 = true;
      return info;
    }
  }

  // algo responde pero no sabemos qué es
  info.present = true;
  return info;
}

// ---------- MPU-6050 ----------
bool mpu_init_once(TwoWire &bus, uint8_t addr) {
  if (!write8(bus, addr, 0x6B, 0x00)) return false; // wake
  delay(10);
  write8(bus, addr, 0x1B, 0x00); // gyro ±250dps
  write8(bus, addr, 0x1C, 0x00); // accel ±2g
  return true;
}

bool mpu_read_ag(TwoWire &bus, uint8_t addr,
                 float &ax_g, float &ay_g, float &az_g,
                 float &gx_dps, float &gy_dps, float &gz_dps) {
  uint8_t buf[14];
  if (!readN(bus, addr, 0x3B, buf, 14)) return false;

  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

  ax_g = ax / 16384.0f;
  ay_g = ay / 16384.0f;
  az_g = az / 16384.0f;
  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;
  return true;
}

// ---------- ICM-20948 ----------
bool icm_select_bank(TwoWire &bus, uint8_t addr, uint8_t bank) {
  return write8(bus, addr, 0x7F, (bank << 4));
}

bool icm_init_once(TwoWire &bus, uint8_t addr) {
  if (!icm_select_bank(bus, addr, 0)) return false;
  write8(bus, addr, 0x06, 0x01);
  delay(10);
  return true;
}

bool icm_read_ag(TwoWire &bus, uint8_t addr,
                 float &ax_g, float &ay_g, float &az_g,
                 float &gx_dps, float &gy_dps, float &gz_dps) {
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

  ax_g = ax / 16384.0f;
  ay_g = ay / 16384.0f;
  az_g = az / 16384.0f;
  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C init (si tu bus B es sensible, bájalo a 100k)
  Wire.begin(SDA_A, SCL_A);
  Wire.setClock(400000);

  I2C_B.begin(SDA_B, SCL_B);
  I2C_B.setClock(400000);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_HALL_32, ADC_11db);
  analogSetPinAttenuation(PIN_MQ6_34,  ADC_11db);
  analogSetPinAttenuation(PIN_HALL_35, ADC_11db);

  Serial.println("Full MQTT: ADC + AHT + ENS(SparkFun) + IMUs -> JSON");

  // AHT
  ahtA_present = ahtA.begin(&Wire);
  Serial.println(ahtA_present ? "AHT (A) OK" : "AHT (A) NO");

  ahtB_present = ahtB.begin(&I2C_B);
  Serial.println(ahtB_present ? "AHT (B) OK" : "AHT (B) NO");

  // ENS (SparkFun) por bus
  ensA_addr = ens_find_addr(Wire);
  if (ensA_addr) ensA_present = ensA.begin(Wire, ensA_addr);
  if (ensA_present) {
    Serial.print("ENS (A) OK en 0x"); Serial.println(ensA_addr, HEX);
    ensA.setOperatingMode(SFE_ENS160_RESET);
    delay(10);
    ensA.setOperatingMode(SFE_ENS160_STANDARD);
  } else {
    Serial.println("ENS (A) NO / FAIL");
    ensA_addr = 0;
  }

  ensB_addr = ens_find_addr(I2C_B);
  if (ensB_addr) ensB_present = ensB.begin(I2C_B, ensB_addr);
  if (ensB_present) {
    Serial.print("ENS (B) OK en 0x"); Serial.println(ensB_addr, HEX);
    ensB.setOperatingMode(SFE_ENS160_RESET);
    delay(10);
    ensB.setOperatingMode(SFE_ENS160_STANDARD);
  } else {
    Serial.println("ENS (B) NO / FAIL");
    ensB_addr = 0;
  }

  // IMUs detect + init once
  imuA_68 = detectIMU(Wire, 0x68);
  imuA_69 = detectIMU(Wire, 0x69);
  imuB_68 = detectIMU(I2C_B, 0x68);
  imuB_69 = detectIMU(I2C_B, 0x69);

  auto initIMUOnce = [](TwoWire &bus, IMUInfo imu) {
    if (!imu.present) return;
    if (imu.isMPU6050)  mpu_init_once(bus, imu.addr);
    if (imu.isICM20948) icm_init_once(bus, imu.addr);
  };

  initIMUOnce(Wire,  imuA_68);
  initIMUOnce(Wire,  imuA_69);
  initIMUOnce(I2C_B, imuB_68);
  initIMUOnce(I2C_B, imuB_69);

  wifiConnect();
  mqttConnect();

  Serial.println("Listo.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) wifiConnect();
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  // ADC
  int raw32 = analogRead(PIN_HALL_32);
  int raw34 = analogRead(PIN_MQ6_34);
  int raw35 = analogRead(PIN_HALL_35);

  float v32 = adc_to_volts(raw32);
  float v34 = adc_to_volts(raw34);
  float v35 = adc_to_volts(raw35);

  // AHT
  float ahtA_T = NAN, ahtA_RH = NAN;
  float ahtB_T = NAN, ahtB_RH = NAN;
  bool ahtA_ok = ahtA_present ? readAHT(ahtA, ahtA_T, ahtA_RH) : false;
  bool ahtB_ok = ahtB_present ? readAHT(ahtB, ahtB_T, ahtB_RH) : false;

  // ENS
  bool ensA_ok = false, ensB_ok = false;
  uint8_t ensA_aqi = 0, ensB_aqi = 0;
  uint16_t ensA_tvoc = 0, ensA_eco2 = 0;
  uint16_t ensB_tvoc = 0, ensB_eco2 = 0;

  if (ensA_present) {
    float T = ahtA_ok ? ahtA_T : NAN;
    float H = ahtA_ok ? ahtA_RH : NAN;
    ensA_ok = readENS(ensA, T, H, ensA_aqi, ensA_tvoc, ensA_eco2);
  }
  if (ensB_present) {
    float T = ahtB_ok ? ahtB_T : NAN;
    float H = ahtB_ok ? ahtB_RH : NAN;
    ensB_ok = readENS(ensB, T, H, ensB_aqi, ensB_tvoc, ensB_eco2);
  }

  // IMUs: leer acc+gyro
  auto readIMU = [](TwoWire &bus, IMUInfo imu,
                    float &ax, float &ay, float &az,
                    float &gx, float &gy, float &gz, bool &ok) {
    ax = ay = az = gx = gy = gz = NAN;
    ok = false;
    if (!imu.present) return;

    if (imu.isMPU6050) ok = mpu_read_ag(bus, imu.addr, ax, ay, az, gx, gy, gz);
    else if (imu.isICM20948) ok = icm_read_ag(bus, imu.addr, ax, ay, az, gx, gy, gz);
  };

  float a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz; bool a68_ok;
  float a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz; bool a69_ok;
  float b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz; bool b68_ok;
  float b69_ax, b69_ay, b69_az, b69_gx, b69_gy, b69_gz; bool b69_ok;

  readIMU(Wire,  imuA_68, a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz, a68_ok);
  readIMU(Wire,  imuA_69, a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz, a69_ok);
  readIMU(I2C_B, imuB_68, b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz, b68_ok);
  readIMU(I2C_B, imuB_69, b69_ax, b69_ay, b69_az, b69_gx, b69_gy, b69_gz, b69_ok);

  // JSON
  StaticJsonDocument<1024> doc;
  doc["ms"] = (uint32_t)millis();
  doc["device"] = DEVICE_ID;

  JsonObject adc = doc.createNestedObject("adc");
  adc["h32"]  = raw32; adc["h32v"] = v32;
  adc["mq6"]  = raw34; adc["mq6v"] = v34;
  adc["h35"]  = raw35; adc["h35v"] = v35;

  if (ahtA_present || ahtB_present) {
    JsonObject aht = doc.createNestedObject("aht");
    if (ahtA_present) {
      JsonObject A = aht.createNestedObject("A");
      if (ahtA_ok) { A["t"] = ahtA_T; A["rh"] = ahtA_RH; }
      else         { A["t"] = nullptr; A["rh"] = nullptr; }
    }
    if (ahtB_present) {
      JsonObject B = aht.createNestedObject("B");
      if (ahtB_ok) { B["t"] = ahtB_T; B["rh"] = ahtB_RH; }
      else         { B["t"] = nullptr; B["rh"] = nullptr; }
    }
  }

  if (ensA_present || ensB_present) {
    JsonObject ens = doc.createNestedObject("ens");
    if (ensA_present) {
      JsonObject A = ens.createNestedObject("A");
      A["addr"] = ensA_addr;
      if (ensA_ok) { A["aqi"] = ensA_aqi; A["tvoc"] = ensA_tvoc; A["eco2"] = ensA_eco2; }
      else         { A["aqi"] = nullptr; A["tvoc"] = nullptr; A["eco2"] = nullptr; }
    }
    if (ensB_present) {
      JsonObject B = ens.createNestedObject("B");
      B["addr"] = ensB_addr;
      if (ensB_ok) { B["aqi"] = ensB_aqi; B["tvoc"] = ensB_tvoc; B["eco2"] = ensB_eco2; }
      else         { B["aqi"] = nullptr; B["tvoc"] = nullptr; B["eco2"] = nullptr; }
    }
  }

  JsonObject imu = doc.createNestedObject("imu");
  auto putImu = [&](const char* key, bool ok,
                    float ax, float ay, float az,
                    float gx, float gy, float gz) {
    JsonObject o = imu.createNestedObject(key);
    if (ok) {
      o["ax"] = ax; o["ay"] = ay; o["az"] = az;
      o["gx"] = gx; o["gy"] = gy; o["gz"] = gz;
    } else {
      o["ax"] = nullptr; o["ay"] = nullptr; o["az"] = nullptr;
      o["gx"] = nullptr; o["gy"] = nullptr; o["gz"] = nullptr;
    }
  };

  if (imuA_68.present) putImu("A68", a68_ok, a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz);
  if (imuA_69.present) putImu("A69", a69_ok, a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz);
  if (imuB_68.present) putImu("B68", b68_ok, b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz);
  if (imuB_69.present) putImu("B69", b69_ok, b69_ax, b69_ay, b69_az, b69_gx, b69_gy, b69_gz);

  char payload[1024];
  size_t n = serializeJson(doc, payload, sizeof(payload));
  bool published = mqtt.publish(MQTT_TOPIC, payload, n);

  Serial.print("MQTT ");
  Serial.print(published ? "OK" : "FAIL");
  Serial.print(" bytes=");
  Serial.print(n);
  Serial.print(" -> ");
  Serial.println(payload);

  delay(1000);
}