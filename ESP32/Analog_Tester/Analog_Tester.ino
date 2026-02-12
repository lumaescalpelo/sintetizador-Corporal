// Lectura de ADC1 en ESP32 DevKit V1
// Pines: 32, 34, 35

const int PIN_ADC_32 = 32;
const int PIN_ADC_34 = 34;
const int PIN_ADC_35 = 35;

void setup() {
  Serial.begin(115200);

  // Resolución ADC (0–4095 → 12 bits)
  analogReadResolution(12);

  // Atenuación para medir hasta ~3.3 V
  analogSetPinAttenuation(PIN_ADC_32, ADC_11db);
  analogSetPinAttenuation(PIN_ADC_34, ADC_11db);
  analogSetPinAttenuation(PIN_ADC_35, ADC_11db);

  Serial.println("Lectura ADC1 iniciada...");
}

void loop() {
  int val32 = analogRead(PIN_ADC_32);
  int val34 = analogRead(PIN_ADC_34);
  int val35 = analogRead(PIN_ADC_35);

  Serial.print("GPIO32: ");
  Serial.print(val32);
  Serial.print(" | GPIO34: ");
  Serial.print(val34);
  Serial.print(" | GPIO35: ");
  Serial.println(val35);

  delay(500);
}
