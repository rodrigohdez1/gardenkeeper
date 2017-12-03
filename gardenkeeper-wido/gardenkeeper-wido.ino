/*
 * This program is used to automate watering, lighting and temperature control for any garden setup.
 * Data can be published into any MQTT-like service. In this case Adafrui IO is used.
 * WiFi and MQTT functions from Adafruit CC3300 and MQTT example libraries.
 * Mostly inspired on Instructables Garduino by liseman http://www.instructables.com/id/Garduino-Gardening-Arduino/
 * 
 * Hardware: Wido-WIFI IoT Node https://www.dfrobot.com/product-1159.html
*/

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/*
    credentials.h contains the username and passwords for you WiFi and MQTT broker endpoints
    The fille should look like:
      #define WLAN_SSID "My-WiFi-Network"
      #define WLAN_PASS "My-WiFi-Password"
      #define AIO_SERVER      "io.adafruit.com"
      #define AIO_SERVERPORT  1883 // Use 8883 for SSL
      #define AIO_USERNAME    "Adafruit username"
      #define AIO_KEY         "Adafruit OI Key"
*/
#include "credentials.h"

// WiFi CC3300 connection settings
#define WiDo_IRQ   7
#define WiDo_VBAT  5
#define WiDo_CS    10

Adafruit_CC3000 WiDo = Adafruit_CC3000(WiDo_CS, WiDo_IRQ, WiDo_VBAT, SPI_CLOCK_DIVIDER);
#define WLAN_SECURITY   WLAN_SEC_WPA2
Adafruit_CC3000_Client WidoClient;

// Inputs
#define temperature_sensor   A0
#define light_sensor         A1
#define moisture_sensor      A2
#define water_pump           6
#define light_switch         4
#define fan_switch           8

// Thresholds for low and high temperature, humidity and light levels
int LOW_LIGHT = 500;
int HIGH_LIGHT = 900;

// Temperature in Celsius
int LOW_TEMPERATURE = 20;
int HIGH_TEMPERATURE = 28;

int HIGH_HUMIDITY = 636;
int LOW_HUMIDITY = 570;

void setup() {
  Serial.begin(115200);
  connectToWiFi();

  pinMode(moisture_sensor, INPUT);
  pinMode(light_sensor, INPUT);
  pinMode(temperature_sensor, INPUT);
  pinMode(water_pump, OUTPUT);
  pinMode(light_switch, OUTPUT);
  pinMode(fan_switch, OUTPUT);

  digitalWrite(light_switch, LOW);
  digitalWrite(fan_switch, LOW);
  digitalWrite(water_pump, LOW);
}

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&WidoClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

void connectToWiFi() {
  if (!WiDo.begin()) {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    while (1);
  }
  if (!WiDo.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed to connecto to AP"));
    while (1);
  }
  Serial.println(F("Connected!"));
  Serial.println(F("Request DHCP"));
  while (!WiDo.checkDHCP()) {
    delay(100); // ToDo: Insert a DHCP timeout!
  }
  if (!WiDo.checkConnected()) {
    Serial.println("Failed to connect");
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      Serial.println("Couldn't connect to MQTT server");
    }
  }
  Serial.println("MQTT Connected!");
}

float check_temperature() {
  float temperature = 0;
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    temperature = analogRead(temperature_sensor);
    float mv = (temperature / 1024.0) * 5000;
    float celsius = mv / 10;
    sum = sum + celsius;
  }
  float temp_avg = sum / 99;

  Serial.print("Temperature: ");
  Serial.println(temp_avg);
  return temp_avg;

  //  if (temp_avg > HIGH_TEMPERATURE) {
  //    Serial.println("Temperature too high, turn the fan!");
  //    analogWrite(fan_switch, HIGH);
  //  }
  //  else {
  //    Serial.println("Temperature OK");
  //    analogWrite(light_switch, LOW);
  //  }
}

int check_lighting() {
  float light = 0;
  light = analogRead(light_sensor);
  Serial.print("Light level: ");
  Serial.println(light);
  return light;

  //  else if (light < LOW_LIGHT) {
  //    Serial.println("Light too low, please turn on the lights!");
  //    digitalWrite(light_switch, LOW);
  //  }
  //  else {
  //    Serial.println("Light OK");
  //  }
}

int check_humidity() {
  float humidity = 0;
  humidity = analogRead(moisture_sensor);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  if (humidity < LOW_HUMIDITY) {
    Serial.println("Humidity too low, please turn on the water pump");
    digitalWrite(water_pump, HIGH);
  }
  else {
    Serial.println("Humidity OK");
    digitalWrite(water_pump, LOW);
  }
  return humidity;
}

void loop() {
  Serial.println("MQTT connect");
  MQTT_connect();

  // Setup a feeds for publishing.
  // Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
  Adafruit_MQTT_Publish lightingFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gardenlight");
  Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gardenhumidity");
  Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gardentemperature");

  // Initialize local variables and retrieve values
  float temperature = 0;
  float lighting = 0;
  float humidity = 0;

  temperature = check_temperature();
  lighting = check_lighting();
  humidity = check_humidity();

  // Send data into MQTT server.
  Serial.print(F("\nSending temperature val "));
  Serial.print(temperature);
  Serial.print("...");
  if (! temperatureFeed.publish(temperature)) {
    Serial.println(F("Failed to send temperature value"));
  } else {
    Serial.println(F("Temperature value sent!"));
  }

  Serial.print(F("\nSending humidity val "));
  Serial.print(humidity);
  Serial.print("...");
  if (! humidityFeed.publish(humidity)) {
    Serial.println(F("Failed to send humidity value"));
  } else {
    Serial.println(F("Humidity value!"));
  }
  // Now we can publish stuff!
  Serial.print(F("\nSending lighting val "));
  Serial.print(lighting);
  Serial.print("...");
  if (! lightingFeed.publish(lighting)) {
    Serial.println(F("Failed lighting value"));
  } else {
    Serial.println(F("Lighting value sent!"));
  }

  // Wait for 10 seconds to recheck metrics
  delay(10000);
}

