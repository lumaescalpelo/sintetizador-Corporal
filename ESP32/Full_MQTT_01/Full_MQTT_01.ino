#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================= WiFi / MQTT =================
static const char* WIFI_SSID = "********";
static const char* WIFI_PASS = "********";

static const char* MQTT_HOST = "192.168.1.100";
static const uint16_t MQTT_PORT = 1883;
static const char* MQTT_TOPIC = "sinte/cuerpo1";

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ================= I2C Pines =================
#define SDA_A 21
#define SCL_A 22
#define SDA_B 16
#define SCL_B 17

TwoWire I2C_B = TwoWire(1);

// ================= ADC Pines =================
// ADC1 pins (OK con WiFi)
static const int PIN_HALL_32 = 32; // 49E
static const int PIN_MQ6_34  = 34; // MQ-6
static const int PIN_HALL_35 = 35; // 49E

// ================= IMUInfo =================
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
  bus.beginTransmission(0x38);
  bus.write(0xBE); bus.write(0x08); bus.write(0x00);
  if (bus.endTransmission() != 0) return false;
  delay(20);
  return true;
}

bool aht_read(TwoWire &bus, float &tempC, float &rh) {
  if (!ping(bus, 0x38)) return false;

  bus.beginTransmission(0x38);
  bus.write(0xAC); bus.write(0x33); bus.write(0x00);
  if (bus.endTransmission() != 0) return false;

  delay(80);

  uint8_t data[6];
  if (!readN(bus, 0x38, 0x00, data, 6)) return false;
  if (data[0] & 0x80) return false;

  uint32_t rawHum = ((uint32_t)(data[1]) << 12) | ((uint32_t)(data[2]) << 4) | ((data[3] >> 4) & 0x0F);
  uint32_t rawTmp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)(data[4]) << 8) | data[5];

  rh = (rawHum * 100.0f) / 1048576.0f;
  tempC = ((rawTmp * 200.0f) / 1048576.0f) - 50.0f;

  return true;
}

// ---------- ENS160 ----------
uint8_t ens_find_addr(TwoWire &bus) {
  if (ping(bus, 0x52)) return 0x52;
  if (ping(bus, 0x53)) return 0x53;
  return 0x00;
}

bool ens_write_opmode(TwoWire &bus, uint8_t addr, uint8_t mode) {
  return write8(bus, addr, 0x10, mode);
}

bool ens_init(TwoWire &bus, uint8_t addr) {
  if (!addr) return false;
  write8(bus, addr, 0x12, 0x00);
  delay(20);
  if (!ens_write_opmode(bus, addr, 0x02)) return false;
  delay(50);
  return true;
}

bool ens_read_basic(TwoWire &bus, uint8_t addr, uint8_t &aqi, uint16_t &tvoc, uint16_t &eco2) {
  if (!addr) return false;

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
  if (!write8(bus, addr, 0x6B, 0x00)) return false;
  delay(10);
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

// ---------- ICM-20948 ----------
bool icm_select_bank(TwoWire &bus, uint8_t addr, uint8_t bank) {
  return write8(bus, addr, 0x7F, (bank << 4));
}

bool icm_init(TwoWire &bus, uint8_t addr) {
  if (!icm_select_bank(bus, addr, 0)) return false;
  write8(bus, addr, 0x06, 0x01);
  delay(10);
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
  gy_dps = gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;

  return true;
}

// ---------- Detección IMUs ----------
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

  while (!mqtt.connected()) {
    // ClientID corto y seguro (<= 23 chars)
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


// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(400);

  // I2C
  Wire.begin(SDA_A, SCL_A);
  I2C_B.begin(SDA_B, SCL_B);

  // ADC config
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_HALL_32, ADC_11db);
  analogSetPinAttenuation(PIN_MQ6_34,  ADC_11db);
  analogSetPinAttenuation(PIN_HALL_35, ADC_11db);

  // Init sensores
  if (ping(Wire, 0x38))  { aht_soft_reset(Wire);  aht_init(Wire); }
  if (ping(I2C_B, 0x38)) { aht_soft_reset(I2C_B); aht_init(I2C_B); }

  uint8_t ensA = ens_find_addr(Wire);
  uint8_t ensB = ens_find_addr(I2C_B);
  if (ensA) ens_init(Wire, ensA);
  if (ensB) ens_init(I2C_B, ensB);

  // Red
  wifiConnect();
  mqttConnect();

  Serial.println("Publicando CSV a MQTT en topic sinte/cuerpo1");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnect();
  }
  if (!mqtt.connected()) {
    mqttConnect();
  }
  mqtt.loop();

  // ===== ADC =====
  int raw32 = analogRead(PIN_HALL_32);
  int raw34 = analogRead(PIN_MQ6_34);
  int raw35 = analogRead(PIN_HALL_35);

  float v32 = adc_to_volts(raw32);
  float v34 = adc_to_volts(raw34);
  float v35 = adc_to_volts(raw35);

  // ===== AHT =====
  float ahtA_t = NAN, ahtA_rh = NAN;
  float ahtB_t = NAN, ahtB_rh = NAN;
  bool ahtA_ok = aht_read(Wire, ahtA_t, ahtA_rh);
  bool ahtB_ok = aht_read(I2C_B, ahtB_t, ahtB_rh);

  // ===== ENS (si está, si no, deja campos vacíos) =====
  uint8_t ensA_addr = ens_find_addr(Wire);
  uint8_t ensB_addr = ens_find_addr(I2C_B);

  bool ensA_ok = false, ensB_ok = false;
  uint8_t ensA_aqi = 0, ensB_aqi = 0;
  uint16_t ensA_tvoc = 0, ensA_eco2 = 0;
  uint16_t ensB_tvoc = 0, ensB_eco2 = 0;

  if (ensA_addr) ensA_ok = ens_read_basic(Wire, ensA_addr, ensA_aqi, ensA_tvoc, ensA_eco2);
  if (ensB_addr) ensB_ok = ens_read_basic(I2C_B, ensB_addr, ensB_aqi, ensB_tvoc, ensB_eco2);

  // ===== IMUs =====
  IMUInfo imuA_68 = detectIMU(Wire, 0x68);
  IMUInfo imuA_69 = detectIMU(Wire, 0x69);
  IMUInfo imuB_68 = detectIMU(I2C_B, 0x68);
  IMUInfo imuB_69 = detectIMU(I2C_B, 0x69);

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
  float b69_ax, b69_ay, b69_az, b69_gx, b69_gy, b69_gz; bool b69_ok;

  readIMU(Wire,  imuA_68, a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz, a68_ok);
  readIMU(Wire,  imuA_69, a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz, a69_ok);
  readIMU(I2C_B, imuB_68, b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz, b68_ok);
  readIMU(I2C_B, imuB_69, b69_ax, b69_ay, b69_az, b69_gx, b69_gy, b69_gz, b69_ok);

  // ===== CSV payload =====
  // Formato fijo (en este orden):
  // ms,raw32,v32,raw34,v34,raw35,v35,
  // ahtA_t,ahtA_rh,ahtB_t,ahtB_rh,
  // ensA_addr,ensA_aqi,ensA_tvoc,ensA_eco2,ensB_addr,ensB_aqi,ensB_tvoc,ensB_eco2,
  // imuA68_ax,imuA68_ay,imuA68_az,imuA68_gx,imuA68_gy,imuA68_gz,
  // imuA69_ax,imuA69_ay,imuA69_az,imuA69_gx,imuA69_gy,imuA69_gz,
  // imuB68_ax,imuB68_ay,imuB68_az,imuB68_gx,imuB68_gy,imuB68_gz,
  // imuB69_ax,imuB69_ay,imuB69_az,imuB69_gx,imuB69_gy,imuB69_gz

  char payload[900];
  int n = snprintf(
    payload, sizeof(payload),
    "%lu,%d,%.3f,%d,%.3f,%d,%.3f,"
    "%.2f,%.1f,%.2f,%.1f,"
    "0x%02X,%u,%u,%u,0x%02X,%u,%u,%u,"
    "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
    "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
    "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
    "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
    (unsigned long)millis(),
    raw32, v32, raw34, v34, raw35, v35,
    ahtA_ok ? ahtA_t : NAN, ahtA_ok ? ahtA_rh : NAN,
    ahtB_ok ? ahtB_t : NAN, ahtB_ok ? ahtB_rh : NAN,
    ensA_addr, ensA_ok ? ensA_aqi : 0, ensA_ok ? ensA_tvoc : 0, ensA_ok ? ensA_eco2 : 0,
    ensB_addr, ensB_ok ? ensB_aqi : 0, ensB_ok ? ensB_tvoc : 0, ensB_ok ? ensB_eco2 : 0,
    a68_ax, a68_ay, a68_az, a68_gx, a68_gy, a68_gz,
    a69_ax, a69_ay, a69_az, a69_gx, a69_gy, a69_gz,
    b68_ax, b68_ay, b68_az, b68_gx, b68_gy, b68_gz,
    b69_ax, b69_ay, b69_az, b69_gx, b69_gy, b69_gz
  );

  if (n > 0 && n < (int)sizeof(payload)) {
    mqtt.publish(MQTT_TOPIC, payload);
    Serial.print("MQTT pub -> ");
    Serial.println(payload);
  } else {
    Serial.println("ERROR: payload demasiado grande");
  }

  delay(1000);
}
