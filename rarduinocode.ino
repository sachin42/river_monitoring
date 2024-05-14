#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include <SimpleTimer.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char *ssid = "project";
const char *password = "12345678";
const char *serverUrl = "http://4.213.103.171:3000/submit"; // Change this to your Node.js server URL

#define MQ135_SENSOR_PIN 36
int sensitivity = 200; // Adjust this value based on your calibration

#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempC = 0;

#define PhSensorPin 39      // the pH meter Analog output is connected with the Arduino’s Analog
unsigned long int avgValue; // Store the average value of the sensor feedback
float b;
int buf[10], temp;
float phValue = 0;

#define SalSensorPin 34                  // Analog pin for the sensor
const float referenceResistor = 10000.0; // Value of the reference resistor in ohms
float salinity = 0;

#define TdsSensorPin 35
#define VREF 3.3  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

#define DHTPIN 18     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
float h = 0, t = 0, f = 0, hic = 0, hif = 0;

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25; // current temperature for compensation

SimpleTimer timer;
String jsonString;

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sendData()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    http.begin(serverUrl); // Specify the URL of your Node.js server
    http.addHeader("Content-Type", "application/json");

    // Send the POST request with the JSON data
    int httpResponseCode = http.POST(jsonString);

    // Check for a successful POST request
    if (httpResponseCode > 0)
    {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String response = http.getString();
      Serial.println(response);
    }
    else
    {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }

    // End the HTTP connection
    http.end();
  }
}

void checkWiFiStatus()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn off WiFi LED
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW); // Turn on WiFi LED
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  pinMode(TdsSensorPin, INPUT);
  pinMode(PhSensorPin, INPUT);
  pinMode(SalSensorPin, INPUT);
  pinMode(MQ135_SENSOR_PIN, INPUT);
  pinMode(0, INPUT_PULLUP);
  dht.begin();
  sensors.begin();
  timer.setInterval(300000UL, sendData);
  timer.setInterval(3000UL, checkWiFiStatus);
  timer.setTimeout(10000UL, sendData);
}

void loop()
{
  timer.run();
  for (int i = 0; i < 10; i++) // Get 10 sample value from the sensor for smooth the value
  {
    buf[i] = analogRead(PhSensorPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++) // sort the analog from small to large
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buf[i] > buf[j])
      {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++) // take the average value of 6 center sample
    avgValue += buf[i];
  phValue = (float)avgValue * 3.3 / 4096 / 6; // convert the analog into millivolt
  phValue = 3.5 * phValue;                    // convert the millivolt into pH value

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)
  { // every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
    {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      // temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      // convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
      // Serial.print("voltage:");
      // Serial.print(averageVoltage,2);
      // Serial.print("V   ");
    }

    float sensorValue = analogRead(SalSensorPin);          // Read the sensor value
    salinity = mapfloat(sensorValue, 0, 4095.0, 100.0, 0); // Assuming conductivity ranges from 0 to 1.0, and salinity is mapped to a 0-100 scale

    int sensor_value = analogRead(MQ135_SENSOR_PIN);
    // Serial.print("Air Quality: ");
    // Serial.println(sensor_value, DEC);
    int air_quality = sensor_value * sensitivity / 4095;
    // Print the salinity value
    // Serial.print("Salinity: ");
    // Serial.print(salinity);

    // Serial.print("\tTDS Value:");
    // Serial.print(tdsValue, 0);
    // Serial.print("ppm  |");

    // Serial.print("    pH:");
    // Serial.print(phValue, 2);
    // Serial.println(" ");

    sensors.requestTemperatures(); // Send the command to get temperatures
    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    tempC = sensors.getTempCByIndex(0);

    h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f))
    {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    // Compute heat index in Fahrenheit (the default)
    hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    hic = dht.computeHeatIndex(t, h, false);

    JsonDocument doc;

    // Add sensor data to JSON object
    doc["A_humidity"] = h;
    doc["A_temperature_C"] = t;
    doc["A_heat_index_C"] = hic;
    doc["A_temperature_F"] = f;
    doc["A_heat_index_F"] = hif;
    doc["A_air_quality"] = air_quality;

    doc["W_salinity"] = salinity;
    doc["W_tds_value"] = tdsValue;
    doc["W_pH_value"] = phValue;
    doc["W_temperature"] = tempC;

    // Serialize JSON object to a string
    serializeJson(doc, jsonString);
  }
  if (digitalRead(0) == LOW)
  {
    delay(100);
    sendData();
  }
}
