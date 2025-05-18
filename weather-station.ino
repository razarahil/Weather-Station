#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>

/* Configuration */
#define WIFI_SSID "WIFI Name"
#define WIFI_PASSWORD "WIFI PASSWORD"  // Redacted for security
#define MQTT_SERVER "SERVER IP HERE"
#define MQTT_PORT 1883
#define MQTT_USER "SERVER USERNAME"
#define MQTT_PASSWORD "SERVER PASSWORD"  // Redacted for security
#define MQTT_CLIENT_ID "outdoor_temperature_sensor"

// MQTT Topics
#define MQTT_TOPIC_TEMP "homeassistant/sensor/outdoor/temperature"
#define MQTT_TOPIC_HUMIDITY "homeassistant/sensor/outdoor/humidity"
#define MQTT_TOPIC_PRESSURE "homeassistant/sensor/outdoor/pressure"
#define MQTT_TOPIC_AHT_TEMP "homeassistant/sensor/outdoor/aht_temperature"
#define MQTT_TOPIC_AHT_HUMIDITY "homeassistant/sensor/outdoor/aht_humidity"
#define MQTT_TOPIC_COMBINED_TEMP "homeassistant/sensor/outdoor/combined_temperature"  // New topic for combined temperature

// Environmental parameters
#define SEALEVELPRESSURE_HPA (1015)
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP (30 * 60)    // 30 minutes sleep time
#define CONNECTION_TIMEOUT 90000
#define TRANSMISSION_TIMEOUT 5000
#define MAX_WIFI_RETRIES 3

// Temperature calculation parameters
#define AHT_WEIGHT 0.8           // Weight for AHT10 sensor (external)
#define BME_WEIGHT 0.2           // Weight for BME280 sensor (internal)
#define MAX_TEMP_DIFF 8.0        // Maximum temperature difference for weighted calculation
#define NIGHT_START_HOUR 19      // Start of night period (7 PM)
#define NIGHT_END_HOUR 7         // End of night period (7 AM)

uint8_t button = 9;  // BOOT button on FireBeetle 2 ESP32-C6
Adafruit_BME280 bme;
Adafruit_AHTX0 aht;
WiFiClient espClient;
PubSubClient mqtt(espClient);

void goToSleep() {
  Serial.println("Going to sleep for 30 minutes");
  delay(100);
  esp_deep_sleep_start();
}

bool setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_WIFI_RETRIES) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed after multiple attempts");
    return false;
  }
  
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

bool connectMQTT() {
  if (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      return false;
    }
  }
  return true;
}

// Get current hour (0-23) using internal RTC
int getCurrentHour() {
  // If you have RTC setup, use that instead
  // This is a placeholder for demo purposes
  return (millis() / 3600000) % 24;
}

bool isNightTime() {
  int currentHour = getCurrentHour();
  return (currentHour >= NIGHT_START_HOUR || currentHour < NIGHT_END_HOUR);
}

float calculateCombinedTemperature(float bmeTemp, float ahtTemp) {
  // Calculate temperature difference
  float tempDiff = abs(bmeTemp - ahtTemp);
  
  // If at night or temperature difference is small, use equal weighting
  if (isNightTime() || tempDiff < 2.0) {
    return (bmeTemp + ahtTemp) / 2.0;
  }
  
  // If the temperature difference is too large, trust the AHT10 completely
  if (tempDiff > MAX_TEMP_DIFF) {
    return ahtTemp;
  }
  
  // Otherwise use weighted average with more weight to AHT10
  return (ahtTemp * AHT_WEIGHT) + (bmeTemp * BME_WEIGHT);
}

void measureAndSleep() {
  bool bme_ok = true;
  bool aht_ok = true;
  
  // Initialize BME280
  if (!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    bme_ok = false;
  }

  // Initialize AHT10
  if (!aht.begin()) {
    Serial.println("Could not find AHT10 sensor!");
    aht_ok = false;
  }

  // If both sensors failed, go to sleep
  if (!bme_ok && !aht_ok) {
    Serial.println("No sensors found - going to sleep");
    delay(1000);
    goToSleep();
    return;
  }

  // Ensure MQTT is connected before measuring
  if (!connectMQTT()) {
    Serial.println("Failed to connect to MQTT - skipping measurements");
    goToSleep();
    return;
  }

  float bmeTemp = 0;
  float bmeHumidity = 0;
  float bmePressure = 0;
  float ahtTemp = 0;
  float ahtHumidity = 0;
  float combinedTemp = 0;

  // Measure BME280 sensor values if available
  if (bme_ok) {
    bmeTemp = bme.readTemperature();
    bmeHumidity = bme.readHumidity();
    bmePressure = bme.readPressure() / 100.0F;

    // Convert BME280 values to strings
    char tempStr[8];
    char humStr[8];
    char presStr[8];
    dtostrf(bmeTemp, 1, 2, tempStr);
    dtostrf(bmeHumidity, 1, 2, humStr);
    dtostrf(bmePressure, 1, 2, presStr);

    // Publish BME280 values
    mqtt.publish(MQTT_TOPIC_TEMP, tempStr, true);
    mqtt.publish(MQTT_TOPIC_HUMIDITY, humStr, true);
    mqtt.publish(MQTT_TOPIC_PRESSURE, presStr, true);

    Serial.printf("Published BME280 - Temperature: %.2f°C, Humidity: %.2f%%, Pressure: %.2f hPa\r\n", 
                  bmeTemp, bmeHumidity, bmePressure);
  }

  // Measure AHT10 sensor values if available
  if (aht_ok) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);  // Get AHT10 readings
    
    ahtTemp = temp.temperature;
    ahtHumidity = humidity.relative_humidity;
    
    // Convert AHT10 values to strings
    char ahtTempStr[8];
    char ahtHumStr[8];
    dtostrf(ahtTemp, 1, 2, ahtTempStr);
    dtostrf(ahtHumidity, 1, 2, ahtHumStr);
    
    // Publish AHT10 values
    mqtt.publish(MQTT_TOPIC_AHT_TEMP, ahtTempStr, true);
    mqtt.publish(MQTT_TOPIC_AHT_HUMIDITY, ahtHumStr, true);
    
    Serial.printf("Published AHT10 - Temperature: %.2f°C, Humidity: %.2f%%\r\n", 
                  ahtTemp, ahtHumidity);
  }

  // Calculate and publish combined temperature if both sensors are working
  if (bme_ok && aht_ok) {
    combinedTemp = calculateCombinedTemperature(bmeTemp, ahtTemp);
    char combinedTempStr[8];
    dtostrf(combinedTemp, 1, 2, combinedTempStr);
    mqtt.publish(MQTT_TOPIC_COMBINED_TEMP, combinedTempStr, true);
    
    Serial.printf("Published Combined Temperature: %.2f°C (BME: %.2f, AHT: %.2f)\r\n", 
                  combinedTemp, bmeTemp, ahtTemp);
  }

  // Wait briefly to ensure data transmission
  unsigned long startTime = millis();
  while ((millis() - startTime) < TRANSMISSION_TIMEOUT) {
    mqtt.loop();
    delay(100);
  }

  Serial.println("Transmission completed");
  delay(60000);  // Added delay so that esp32 finished data transfer before going to sleep. Adjust based on your usuage.
  goToSleep();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C bus (shared by both sensors)

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Configure wake up timer for 30 minutes
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Setup WiFi
  if (!setupWiFi()) {
    Serial.println("Failed to connect to WiFi. Going to sleep...");
    goToSleep();
    return;
  }

  // Setup MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);

  // Try to connect to MQTT
  if (!connectMQTT()) {
    Serial.println("Failed to connect to MQTT. Going to sleep...");
    goToSleep();
    return;
  }

  Serial.println("Successfully connected to MQTT broker");
  delay(1000); // Allow time for connection establishment
}

void loop() {
  // Check for WiFi reset button
  if (digitalRead(button) == LOW) {
    delay(100); // Debounce
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 3000) {
        Serial.println("WiFi reset initiated. Rebooting in 1s.");
        WiFi.disconnect(true);  // Clear stored credentials
        delay(1000);
        ESP.restart();
      }
    }
  }

  // Measure and go to sleep
  measureAndSleep();
}