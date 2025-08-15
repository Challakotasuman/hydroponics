#define BLYNK_TEMPLATE_ID "TMPL3xnVG6Wc-"
#define BLYNK_TEMPLATE_NAME "HYDROPHONICS"
#define BLYNK_AUTH_TOKEN "dSqui73xNGyLQochxxFDCDdUrudC6vK3"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "HYDROPHONICS";
char pass[] = "HYDROPHONICS";

#define PH_SENSOR_PIN A0
#define TDS_SENSOR_PIN 39
#define DS18B20_PIN 23
#define phdummy_pin 35
#define tdsdummy_pin 34
#define pump 19
#define fan 17
#include <OneWire.h>
#include <DallasTemperature.h>

#define VREF 3.3   // Analog reference voltage (Volt) of the ADC
#define SCOUNT 30  // Number of sample points for TDS

float calibration_value = 21.34 - 0.7;
int ph_buffer[10], ph_temp;
unsigned long avgval;
float ph_act;

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  pinMode(PH_SENSOR_PIN, INPUT);
  pinMode(TDS_SENSOR_PIN, INPUT);
  pinMode(pump, OUTPUT);
  pinMode(fan, OUTPUT);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Blynk.run();
  sensors.begin();
}

void loop() {
  // DS18B20 Temperature Reading
  Blynk.run();
  float dummyPh = analogRead(35) * (11.85 / 4095.0);
  float dummyTDS = analogRead(34) * (999.25 / 4095.0);
  Serial.print("dummyPh =");
  Serial.println(dummyPh);
  Serial.print("dummyTDS =");
  Serial.println(dummyTDS);
  Blynk.virtualWrite(V0, dummyPh);
  Blynk.virtualWrite(V1, dummyTDS);
  if ((dummyPh <= 5.0) || (dummyTDS >= 500.0)) {
    digitalWrite(pump, 1);
    Serial.println("Pump On..!");
  } else {
    digitalWrite(pump, 0);
    Serial.println("Pump Off..!");
  }

  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Blynk.virtualWrite(V2, temperature);
  Serial.println(" °C");
  if (temperature >= 36.0) {
    digitalWrite(fan, 1);
    Serial.println("Fan On..!");
  } else {
    digitalWrite(fan, 0);
    Serial.println("Fan Off..!");
  }
  // pH Sensor Reading
  for (int i = 0; i < 10; i++) {
    ph_buffer[i] = analogRead(PH_SENSOR_PIN);
    delay(30);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (ph_buffer[i] > ph_buffer[j]) {
        ph_temp = ph_buffer[i];
        ph_buffer[i] = ph_buffer[j];
        ph_buffer[j] = ph_temp;
      }
    }
  }
  avgval = 0;
  for (int i = 2; i < 8; i++)
    avgval += ph_buffer[i];
  float volt = (float)avgval * 3.3 / 4095 / 6;
  ph_act = -5.70 * volt + calibration_value;
  Serial.print("pH Value: ");
  Serial.println(ph_act);
  Blynk.virtualWrite(V3, ph_act);
  // TDS Sensor Reading
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_SENSOR_PIN);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4095.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Blynk.virtualWrite(V4, tdsValue);
    Serial.println(" ppm");
  }

  delay(600);  // General delay between readings
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
