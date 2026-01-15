#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Ultrasonic pins
#define TRIG_PIN 9
#define ECHO_PIN 10
#define TEMP_PIN 11 
#define LED_PIN 12

// DHT11 settings
#define DHTPIN A0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Potentiometer pin for temperature set point
#define POT_PIN A1

// PI controller variables
float Kp = 12.0;
float Ki = 9.0;
float integral = 0.0;
unsigned long prevTime = 0;

// ETA estimation
float lastTemperature = 0.0;
unsigned long lastTime = 0;
float estimatedTimeToSetpoint = -1;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(TEMP_PIN, OUTPUT);

  dht.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 initialization failed!"));
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Temperature Controlled Water Tap");
  display.display();
  delay(1000);

  analogWrite(TEMP_PIN, 0);
  prevTime = millis();
  lastTime = millis();
}

void loop() {
  long duration;
  float distance, temperature, setPoint;
  float output = 0; // Declare here



  // Read distance
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.0343 / 2;

  // Read temperature
  temperature = dht.readTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read setpoint
  int potValue = analogRead(POT_PIN);
  setPoint = map(potValue, 0, 1023, 20, 40); // 20 to 40 °C

  // LED logic
  digitalWrite(LED_PIN, (distance < 8.0) ? HIGH : LOW);

  // PI control
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  float error = setPoint - temperature;
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100); // Anti-windup

  output = Kp * error + Ki * integral;
  output = constrain(output, 0, 255); // Output limit for PWM

  // Heater control with deadband
  if (output > 0) {
    digitalWrite(TEMP_PIN, HIGH);
  } else {
    digitalWrite(TEMP_PIN, LOW);
  }

  // Estimate time to reach setpoint
  float deltaT = (currentTime - lastTime) / 1000.0;
  float tempChangeRate = (temperature - lastTemperature) / deltaT;

  if (abs(tempChangeRate) > 0.01) {
    estimatedTimeToSetpoint = abs(setPoint - temperature) / abs(tempChangeRate);
  } else {
    estimatedTimeToSetpoint = -1;
  }

  lastTemperature = temperature;
  lastTime = currentTime;

  // Serial output
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | Temp: ");
  Serial.print(temperature);
  Serial.print(" C | Set: ");
  Serial.print(setPoint);
  Serial.print(" C | PWM: ");
  Serial.print((int)output);
  Serial.print(" | ETA: ");
  if (estimatedTimeToSetpoint >= 0 && estimatedTimeToSetpoint < 600) {
    Serial.print(estimatedTimeToSetpoint, 1);
    Serial.println("s");
  } else {
    Serial.println("Calculating....");
  }

  // OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Dist: ");
  display.print(distance, 1);
  display.println(" cm");

  display.setCursor(0, 14);
  display.print("Temp: ");
  display.print(temperature, 1);
  display.println(" C");

  display.setCursor(0, 28);
  display.print("Set Pt: ");
  display.print(setPoint, 1);
  display.println(" C");

  display.setCursor(0, 42);
  if (estimatedTimeToSetpoint >= 0 && estimatedTimeToSetpoint < 600) {
    display.print("ETA: ");
    display.print(estimatedTimeToSetpoint, 0);
    display.println("s");
  } else {
    display.println("ETA: Calculating....");
  }

  display.display();
  delay(1000);
}
