#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ================= WiFi / MQTT =================
static const char* WIFI_SSID = "INFINITUMD2AC";
static const char* WIFI_PASS = "PCwGdtcV9D";

static const char* MQTT_HOST = "192.168.1.100";
static const uint16_t MQTT_PORT = 1883;

// Topics
static const char* TOPIC_ENSAHT = "sinte/cuerpo1/ensaht";
static const char* TOPIC_MOVE   = "sinte/cuerpo1/move";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ================= I2C Pines (Hardware) =================
#define SDA_A 21
#define SCL_A 22
#define SDA_B 16
#define SCL_B 17
TwoWire I2C_B = TwoWire(1);

// ================= I2C Pines (Software BUS C) =================
#define SDA_C 25
#define SCL_C 26

// ================= IMUInfo =================
struct IMUInfo {
  bool present = false;
  bool isMPU6050 = false;
  bool isICM20948 = false;
  uint8_t addr = 0;
};

// ============================================================
//                 Helpers I2C (Hardware)
// ============================================================
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

// ============================================================
//                 Software I2C (BUS C) Bitbang
// ============================================================
static inline void i2cDelay() {
  delayMicroseconds(6); // si falla, súbelo (8-12)
}

static inline void sdaHi() { pinMode(SDA_C, INPUT_PULLUP); }
static inline void sdaLo() { pinMode(SDA_C, OUTPUT); digitalWrite(SDA_C, LOW); }

static inline void sclHi() { pinMode(SCL_C, INPUT_PULLUP); }
static inline void sclLo() { pinMode(SCL_C, OUTPUT); digitalWrite(SCL_C, LOW); }

static inline bool sdaRead() { return digitalRead(SDA_C); }

void i2cSoftInit() {
  pinMode(SDA_C, INPUT_PULLUP);
  pinMode(SCL_C, INPUT_PULLUP);
}

void i2cSoftStart() {
  sdaHi(); sclHi(); i2cDelay();
  sdaLo(); i2cDelay();
  sclLo(); i2cDelay();
}

void i2cSoftStop() {
  sdaLo(); i2cDelay();
  sclHi(); i2cDelay();
  sdaHi(); i2cDelay();
}

bool i2cSoftWriteByte(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    if (b & 0x80) sdaHi(); else sdaLo();
    i2cDelay();
    sclHi(); i2cDelay();
    sclLo(); i2cDelay();
    b <<= 1;
  }
  sdaHi();
  i2cDelay();
  sclHi(); i2cDelay();
  bool ack = (sdaRead() == 0);
  sclLo(); i2cDelay();
  return ack;
}

uint8_t i2cSoftReadByte(bool ack) {
  uint8_t b = 0;
  sdaHi();
  for (int i = 0; i < 8; i++) {
    b <<= 1;
    sclHi(); i2cDelay();
    if (sdaRead()) b |= 1;
    sclLo(); i2cDelay();
  }
  if (ack) sdaLo(); else sdaHi();
  i2cDelay();
  sclHi(); i2cDelay();
  sclLo(); i2cDelay();
  sdaHi();
  return b;
}

bool i2cSoftPing(uint8_t addr7) {
  i2cSoftStart();
  bool ok = i2cSoftWriteByte((addr7 << 1) | 0);
  i2cSoftStop();
  return ok;
}

bool i2cSoftWriteReg(uint8_t addr7, uint8_t reg, uint8_t val) {
  i2cSoftStart();
  if (!i2cSoftWriteByte((addr7 << 1) | 0)) { i2cSoftStop(); return false; }
  if (!i2cSoftWriteByte(reg))             { i2cSoftStop(); return false; }
  if (!i2cSoftWriteByte(val))             { i2cSoftStop(); return false; }
  i2cSoftStop();
  return true;
}

bool i2cSoftWriteBytes(uint8_t addr7, const uint8_t *data, size_t n) {
  i2cSoftStart();
  if (!i2cSoftWriteByte((addr7 << 1) | 0)) { i2cSoftStop(); return false; }
  for (size_t i = 0; i < n; i++) {
    if (!i2cSoftWriteByte(data[i])) { i2cSoftStop(); return false; }
  }
  i2cSoftStop();
  return true;
}

bool i2cSoftReadRegN(uint8_t addr7, uint8_t reg, uint8_t *buf, size_t n) {
  i2cSoftStart();
  if (!i2cSoftWriteByte((addr7 << 1) | 0)) { i2cSoftStop(); return false; }
  if (!i2cSoftWriteByte(reg))             { i2cSoftStop(); return false; }

  i2cSoftStart();
  if (!i2cSoftWriteByte((addr7 << 1) | 1)) { i2cSoftStop(); return false; }

  for (size_t i = 0; i < n; i++) {
    buf[i] = i2cSoftReadByte(i + 1 < n);
  }
  i2cSoftStop();
  return true;
}

// ============================================================
//                    AHT2x (BUS C software)
// ============================================================
static const uint8_t AHT_ADDR = 0x38;

bool ahtC_soft_reset() { return i2cSoftWriteReg(AHT_ADDR, 0xBA, 0x00); }

bool ahtC_init() {
  if (!i2cSoftPing(AHT_ADDR)) return false;
  uint8_t cmd[3] = {0xBE, 0x08, 0x00};
  bool ok = i2cSoftWriteBytes(AHT_ADDR, cmd, 3);
  delay(20);
  return ok;
}

bool ahtC_read(float &tempC, float &rh) {
  if (!i2cSoftPing(AHT_ADDR)) return false;

  uint8_t trig[3] = {0xAC, 0x33, 0x00};
  if (!i2cSoftWriteBytes(AHT_ADDR, trig, 3)) return false;

  delay(80);

  uint8_t data[6];
  if (!i2cSoftReadRegN(AHT_ADDR, 0x00, data, 6)) return false;
  if (data[0] & 0x80) return false;

  uint32_t rawHum = ((uint32_t)(data[1]) << 12) | ((uint32_t)(data[2]) << 4) | ((data[3] >> 4) & 0x0F);
  uint32_t rawTmp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)(data[4]) << 8) | data[5];

  rh = (rawHum * 100.0f) / 1048576.0f;
  tempC = ((rawTmp * 200.0f) / 1048576.0f) - 50.0f;
  return true;
}

// ============================================================
//                    ENS160 (BUS C software)
// ============================================================
uint8_t ensC_find_addr() {
  if (i2cSoftPing(0x52)) return 0x52;
  if (i2cSoftPing(0x53)) return 0x53;
  return 0x00;
}

bool ensC_init(uint8_t addr) {
  if (!addr) return false;
  i2cSoftWriteReg(addr, 0x12, 0x00);
  delay(20);
  if (!i2cSoftWriteReg(addr, 0x10, 0x02)) return false; // STANDARD
  delay(50);
  return true;
}

bool ensC_read_basic(uint8_t addr, uint8_t &aqi, uint16_t &tvoc, uint16_t &eco2, uint8_t &flags) {
  if (!addr) return false;

  uint8_t b;
  uint8_t buf2[2];

  if (!i2cSoftReadRegN(addr, 0x21, &b, 1)) return false;
  aqi = b;

  if (!i2cSoftReadRegN(addr, 0x22, buf2, 2)) return false;
  tvoc = (uint16_t)buf2[1] << 8 | buf2[0];

  if (!i2cSoftReadRegN(addr, 0x24, buf2, 2)) return false;
  eco2 = (uint16_t)buf2[1] << 8 | buf2[0];

  if (!i2cSoftReadRegN(addr, 0x28, &b, 1)) return false;
  flags = b;

  return true;
}

// ============================================================
//                 IMU: MPU-6050 / ICM-20948
// ============================================================
bool mpu_init(TwoWire &bus, uint8_t addr) {
  if (!write8(bus, addr, 0x6B, 0x00)) return false;
  delay(2);
  write8(bus, addr, 0x1B, 0x00);
  write8(bus, addr, 0x1C, 0x00);
  return true;
}

bool mpu_read_ag(TwoWire &bus, uint8_t addr, float &ax_g, float &ay_g, float &az_g,
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

bool icm_select_bank(TwoWire &bus, uint8_t addr, uint8_t bank) {
  return write8(bus, addr, 0x7F, (bank << 4));
}

bool icm_init(TwoWire &bus, uint8_t addr) {
  if (!icm_select_bank(bus, addr, 0)) return false;
  write8(bus, addr, 0x06, 0x01);
  delay(2);
  return true;
}

bool icm_read_ag(TwoWire &bus, uint8_t addr, float &ax_g, float &ay_g, float &az_g,
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

IMUInfo detectIMU(TwoWire &bus, uint8_t addr) {
  IMUInfo info;
  info.addr = addr;
  if (!ping(bus, addr)) return info;

  uint8_t who_mpu;
  if (read8(bus, addr, 0x75, who_mpu)) {
    if (who_mpu == 0x68 || who_mpu == 0x69) {
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

  info.present = true;
  return info;
}

// ============================================================
//                      Formateo pedido
// ============================================================

// 1 decimal truncado (no redondeado)
float trunc1(float x) {
  if (isnan(x) || isinf(x)) return x;
  return (float)((int)(x * 10.0f)) / 10.0f;
}

// 2 cifras significativas (redondeado). Mucho más útil para “movimiento”.
float sig2(float x) {
  if (x == 0.0f || isnan(x) || isinf(x)) return x;
  float ax = fabsf(x);
  int e = (int)floorf(log10f(ax));
  float scale = powf(10.0f, 1 - e); // 2 sig figs
  return roundf(x * scale) / scale;
}

// entero truncado hacia cero
int toIntTrunc(float x) {
  if (isnan(x) || isinf(x)) return 0;
  return (int)x;
}

// ============================================================
//                      WiFi / MQTT
// ============================================================
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if (millis() - t0 > 20000) {
      WiFi.disconnect(true);
      delay(500);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }
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
    if (!mqtt.connect(clientId)) delay(1000);
  }
}

// ============================================================
//                      App state / Timers
// ============================================================
static uint8_t ensC_addr = 0;
static bool ensC_ready = false;

static uint32_t lastMoveMs   = 0;
static uint32_t lastEnsAhtMs = 0;

static const uint32_t MOVE_INTERVAL_MS   = 200;   // IMUs cada 200ms
static const uint32_t ENSAHT_INTERVAL_MS = 2000;  // ENS+AHT cada 2s

// ============================================================
//                           Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C hardware
  Wire.begin(SDA_A, SCL_A);
  Wire.setClock(400000);

  I2C_B.begin(SDA_B, SCL_B);
  I2C_B.setClock(400000);

  // I2C software bus C
  i2cSoftInit();
  delay(10);

  // Init AHT+ENS en BUS C (soft)
  if (i2cSoftPing(AHT_ADDR)) {
    ahtC_soft_reset();
    ahtC_init();
  }
  ensC_addr = ensC_find_addr();
  if (ensC_addr) ensC_ready = ensC_init(ensC_addr);

  // WiFi/MQTT
  wifiConnect();
  mqttConnect();

  Serial.println("Split MQTT listo: ensaht@2s / move@200ms");
}

// ============================================================
//                           Loop
// ============================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) wifiConnect();
  if (!mqtt.connected()) mqttConnect();
  mqtt.loop();

  uint32_t now = millis();

  // ==========================================================
  // MOVE (IMUs) cada 200ms
  // ==========================================================
  if ((uint32_t)(now - lastMoveMs) >= MOVE_INTERVAL_MS) {
    lastMoveMs = now;

    // Detect IMUs (NO incluimos B69)
    IMUInfo imuA_68 = detectIMU(Wire, 0x68);
    IMUInfo imuA_69 = detectIMU(Wire, 0x69);
    IMUInfo imuB_68 = detectIMU(I2C_B, 0x68);
    // IMUInfo imuB_69 = detectIMU(I2C_B, 0x69); // <- NO

    auto readIMU = [](TwoWire &bus, IMUInfo imu,
                      float &ax, float &ay, float &az, float &gx, float &gy, float &gz, bool &ok) {
      ax = ay = az = gx = gy = gz = NAN;
      ok = false;
      if (!imu.present) return;

      if (imu.isMPU6050) {
        mpu_init(bus, imu.addr);
        ok = mpu_read_ag(bus, imu.addr, ax, ay, az, gx, gy, gz);
      } else if (imu.isICM20948) {
        icm_init(bus, imu.addr);
        ok = icm_read_ag(bus, imu.addr, ax, ay, az, gx, gy, gz);
      }
    };

    float a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz; bool a68_ok;
    float a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz; bool a69_ok;
    float b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz; bool b68_ok;

    readIMU(Wire,  imuA_68, a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz, a68_ok);
    readIMU(Wire,  imuA_69, a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz, a69_ok);
    readIMU(I2C_B, imuB_68, b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz, b68_ok);

    StaticJsonDocument<512> doc;
    doc["ms"] = now;
    doc["device"] = "esp32-cuerpo1";

    JsonObject imu = doc.createNestedObject("imu");

    auto putIMU = [&](const char* key, bool ok, float ax, float ay, float az, float gx, float gy, float gz) {
      if (!ok) return; // <- NO creamos objeto si no hay lectura válida
      JsonObject o = imu.createNestedObject(key);

      // Acelerómetro: 2 cifras significativas
      o["ax"] = sig2(ax);
      o["ay"] = sig2(ay);
      o["az"] = sig2(az);

      // Giroscopio: enteros
      o["gx"] = toIntTrunc(gx);
      o["gy"] = toIntTrunc(gy);
      o["gz"] = toIntTrunc(gz);
    };

    putIMU("A68", a68_ok, a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz);
    putIMU("A69", a69_ok, a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz);
    putIMU("B68", b68_ok, b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz);

    char payload[512];
    size_t n = serializeJson(doc, payload, sizeof(payload));
    bool okPub = mqtt.publish(TOPIC_MOVE, payload, n);

    Serial.print(okPub ? "MOVE OK " : "MOVE FAIL ");
    Serial.print("bytes="); Serial.print(n);
    Serial.print(" -> ");
    Serial.println(payload);
  }

  // ==========================================================
  // ENSAHT cada 2000ms
  // ==========================================================
  if ((uint32_t)(now - lastEnsAhtMs) >= ENSAHT_INTERVAL_MS) {
    lastEnsAhtMs = now;

    if (!ensC_ready) {
      ensC_addr = ensC_find_addr();
      if (ensC_addr) ensC_ready = ensC_init(ensC_addr);
    }

    float t = NAN, rh = NAN;
    bool aht_ok = ahtC_read(t, rh);

    uint8_t aqi = 0, flags = 0;
    uint16_t tvoc = 0, eco2 = 0;
    bool ens_ok = false;

    if (ensC_ready) {
      ens_ok = ensC_read_basic(ensC_addr, aqi, tvoc, eco2, flags);
      if (!ens_ok) ensC_ready = false;
    }

    StaticJsonDocument<256> doc;
    doc["ms"] = now;
    doc["device"] = "esp32-cuerpo1";

    JsonObject aht = doc.createNestedObject("aht");
    if (aht_ok) {
      // Temp: 1 decimal
      aht["t"] = trunc1(t);
      // RH: entero
      aht["rh"] = (int)rh;
    } else {
      // si falla, no inventamos valores
      aht["t"] = nullptr;
      aht["rh"] = nullptr;
    }

    JsonObject ens = doc.createNestedObject("ens");
    if (ensC_addr) ens["addr"] = ensC_addr; else ens["addr"] = nullptr;

    if (ens_ok) {
      ens["aqi"] = aqi;               // (si luego lo quieres quitar, se quita)
      ens["tvoc"] = (int)tvoc;        // entero
      ens["eco2"] = (int)eco2;        // entero
      ens["flags"] = flags;
    } else {
      ens["aqi"] = nullptr;
      ens["tvoc"] = nullptr;
      ens["eco2"] = nullptr;
      ens["flags"] = nullptr;
    }

    char payload[256];
    size_t n = serializeJson(doc, payload, sizeof(payload));
    bool okPub = mqtt.publish(TOPIC_ENSAHT, payload, n);

    Serial.print(okPub ? "ENSAHT OK " : "ENSAHT FAIL ");
    Serial.print("bytes="); Serial.print(n);
    Serial.print(" -> ");
    Serial.println(payload);
  }
}