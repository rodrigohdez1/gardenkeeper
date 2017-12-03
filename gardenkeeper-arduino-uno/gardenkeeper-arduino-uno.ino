/*
 * Define inputs
*/
#define temperature_sensor   A0
#define light_sensor         A1
#define moisture_sensor      A2
#define water_pump           6
#define light_switch         4
#define fan_switch           8

/*
 * Define thresholds
*/
int LOW_LIGHT = 500;
int HIGH_LIGHT = 900;

/*
 * Temperature in Celsius
*/
int LOW_TEMPERATURE = 20;
int HIGH_TEMPERATURE = 28;

int HIGH_HUMIDITY = 636;
int LOW_HUMIDITY = 570;

void setup() {
  Serial.begin(115200);

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

void check_temperature() {
  float temperature = 0;
  float sum = 0;
  for (int i=0; i < 100; i++) {
    temperature = analogRead(temperature_sensor);
    float mv = (temperature / 1024.0) * 5000;
    float celsius = mv / 10;
    sum = sum + celsius;
  }
  float temp_avg = sum / 99;

  Serial.print("Temperature: ");
  Serial.println(temp_avg);

//  if (temp_avg > HIGH_TEMPERATURE) {
//    Serial.println("Temperature too high, turn the fan!");
//    analogWrite(fan_switch, HIGH);
//  }
//  else {
//    Serial.println("Temperature OK");
//    analogWrite(light_switch, LOW);
//  }
}

void check_lighting() {
  int light = 0;
  light = analogRead(light_sensor);
  Serial.print("Light level: ");
  Serial.println(light);

//  if (light > HIGH_LIGHT) {
//    Serial.println("Light too high, turn off the lights!");
//    digitalWrite(light_switch, HIGH);
//  }
//  else if (light < LOW_LIGHT) {
//    Serial.println("Light too low, please turn on the lights!");
//    digitalWrite(light_switch, LOW);
//  }
//  else {
//    Serial.println("Light OK");
//  }
}

void check_humidity() {
  int humidity = 0;
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
}

void loop() {
  // put your main code here, to run repeatedly:
  check_temperature();
  check_lighting();
  check_humidity();
  delay(5000);
}
