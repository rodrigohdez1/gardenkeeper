/*
 * Define inputs
*/
#define moisture_sensor      A0
#define light_sensor         A1
#define temperature_sensor   A2
#define watering_pump        4
#define light_switch         6
#define fan_switch           8

/*
 * Define thresholds
*/
int LOW_MOISTURE = 200;
int HIGH_MOISTURE = 0;

int LOW_LIGHT = 200;
int HIGH_LIGHT = 0;

int LOW_TEMPERATURE = 200;
int HIGH_TEMPERATURE = 0;

int HIGH_HUMIDITY = 200;
int LOW_HUMIDITY = 0;

void setup() {
  Serial.begin(115200);

  pinMode(moisture_sensor, INPUT);
  pinMode(light_sensor, INPUT);
  pinMode(temperature_sensor, INPUT);
  pinMode(watering_pump, OUTPUT);
  pinMode(light_switch, OUTPUT);
  pinMode(fan_switch, OUTPUT);

  digitalWrite(watering_pump, LOW);
  digitalWrite(light_switch, LOW);
  digitalWrite(fan_switch, LOW);
}

void check_temperature() {
  int temperature = 0;
  temperature = analogRead(temperature_sensor);
  Serial.print("Temperature: ");
  Serial.println(temperature);

  if (temperature > HIGH_TEMPERATURE) {
    Serial.println("Temperature too high, turn the fan!");
    analogWrite(fan_switch, HIGH);
  }
  else if (temperature < LOW_TEMPERATURE) {
    Serial.println("Temperature too low, please heat the room");
    analogWrite(light_switch, LOW);
  }
  else {
    Serial.println("Temperature OK");
  }
}

void check_lighting() {
  int light = 0;
  light = analogRead(light_sensor);
  Serial.print("Light level: ");
  Serial.println(light);

  if (light > HIGH_LIGHT) {
    Serial.println("Light too high, turn off the lights!");
    digitalWrite(light_switch, HIGH);
  }
  else if (light < LOW_LIGHT) {
    Serial.println("Light too low, please turn on the lights!");
    digitalWrite(light_switch, LOW);
  }
  else {
    Serial.println("Light OK");
  }
}

void check_humidity() {
  int humidity = 0;
  humidity = analogRead(light_sensor);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  if (humidity > HIGH_HUMIDITY) {
    Serial.println("Humidity too high, turn on the fan!");
    digitalWrite(fan_switch, HIGH);
  }
  else if (humidity < LOW_HUMIDITY) {
    Serial.println("Humidity too low, please turn on the water pump");
    digitalWrite(fan_switch, HIGH);
  }
  else {
    Serial.println("Humidity OK");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  check_temperature();
  check_lighting();
  check_humidity();
  delay(5000);
}
