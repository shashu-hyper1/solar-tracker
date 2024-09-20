


#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

float voltage;
#define currentSensorPin A5
#define voltageSensorPin A6
#define rly 11
#define rled 3
#define gled 4

Servo servohori;  // horizontal servo
Servo servoverti; // vertical servo

// Servo Calibration Values (Adjust these)
int servoh = 90; // Start at the middle
int servohLimitHigh = 160;
int servohLimitLow = 50;

int servov = 90; // Start at the middle
int servovLimitHigh = 160;
int servovLimitLow = 20;

// LDR Pin Definitions
#define ldrtopl A0
#define ldrtopr A1
#define ldrbotl A2
#define ldrbotr A3
#define ldrmt   A7

// Button and Motor Pin Definitions
#define button1 10
#define button2 7
#define motorA  8
#define motorB  9

// Voltage and Current Measurement Constants
const float voltageFactor = 5.128; // Reduction factor of the Voltage Sensor
const float vCC = 5.00; // Arduino input voltage
const float currentSensorSensitivity = 0.0264; // Sensitivity of the current sensor (A/V)
const float currentSensorOffset = 13.51; // Offset of the current sensor (A)

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long delayTime = 170UL * 60UL * 1000UL; // 2h 50 min minutes in milliseconds
void setup() {
  // Initialize LCD and display welcome message
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  lcd.print("Solar Tracker");
  lcd.setCursor(4, 1);
  lcd.print("By: SAI TECH");
  delay(1000);
  displayMessage("Initializing", "Setup: Completed");
  delay(2000);
  lcd.clear();

  // Attach Servos and set initial positions
  servohori.attach(5);
  servoverti.attach(6);
  servohori.write(servoh);
  servoverti.write(servov);

  // Pin Mode Setup
  pinMode(voltageSensorPin, INPUT);
  pinMode(currentSensorPin, INPUT);
  pinMode(rly, OUTPUT);
  pinMode(rled, OUTPUT);
  pinMode(gled, OUTPUT);
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  digitalWrite(rly, LOW);
}

void loop() {
  int ldrStatus = analogRead(ldrmt); // Read the middle LDR
  int buttonStateA = digitalRead(button1);
  int buttonStateB = digitalRead(button2);
  Serial.println(ldrStatus);
  // Automatic Tracking Mode (ldrStatus < 50 indicates low light)
  if (ldrStatus < 50) {
    if (buttonStateA == HIGH || buttonStateB == LOW) {
      digitalWrite(motorA, HIGH);
      digitalWrite(motorB, LOW);
      //delay(1100);


     
      trackSun();
      readVoltageAndCurrent();
    }
    else
    {
      digitalWrite(motorA, LOW);
      digitalWrite(motorB, LOW);
    }
  }

  else if (buttonStateA == LOW || buttonStateB == HIGH) {

    digitalWrite(motorA, LOW);
    digitalWrite(motorB, HIGH);
    //delay(950);
  } else {
    digitalWrite(motorA, LOW);
    digitalWrite(motorB, LOW);
  }

  delay(200); // Short delay for stability
}

// Function to track the sun based on LDR readings
void trackSun() {
  Serial.println("Tracking sun...");

  int topl = analogRead(ldrtopl);
  int topr = analogRead(ldrtopr);
  int botl = analogRead(ldrbotl);
  int botr = analogRead(ldrbotr);

  int avgtop = (topl + topr) / 2;
  int avgbot = (botl + botr) / 2;
  int avgleft = (topl + botl) / 2;
  int avgright = (topr + botr) / 2;

  // Adjust Vertical Servo (Up/Down)
  if (abs(avgtop - avgbot) > 50) { // Adjust the threshold (50) as needed
    if (avgtop < avgbot) {
      servov = moveServo(servoverti, servov, servov + 4, servovLimitLow, servovLimitHigh); // Smaller steps for smoother movement
    } else {
      servov = moveServo(servoverti, servov, servov - 4, servovLimitLow, servovLimitHigh);
    }
  }

  // Adjust Horizontal Servo (Left/Right)
  if (abs(avgleft - avgright) > 70) { // Adjust the threshold (50) as needed
    if (avgleft > avgright) {
      servoh = moveServo(servohori, servoh, servoh - 5, servohLimitLow, servohLimitHigh);
    } else {
      servoh = moveServo(servohori, servoh, servoh + 5, servohLimitLow, servohLimitHigh);
    }
  }
}

// Function to move servo with limits
int moveServo(Servo &servo, int currentPos, int targetPos, int lowerLimit, int upperLimit) {
  targetPos = constrain(targetPos, lowerLimit, upperLimit); // Ensure within limits
  servo.write(targetPos);
  return targetPos;
}

// Function to read and display voltage and current
void readVoltageAndCurrent() {
  // Read and Calculate Voltage
  float voltageSensorVal = analogRead(voltageSensorPin);
  float vOut = (voltageSensorVal / 1024.0) * vCC;
  voltage = vOut * voltageFactor;

  // Read and Calculate Current (Averaging for stability)
  float current = 0;
  for (int i = 0; i < 10; i++) {
    current += (currentSensorSensitivity * analogRead(currentSensorPin) - currentSensorOffset);
    delay(1);
  }
  current /= 10.0;

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(voltage, 2);
  lcd.print("V");

  // Check if voltage is less than 1.25V (You might need to adjust this threshold)
  if (voltage < 1.25) {
    handleLowVoltage();
  }

  lcd.setCursor(0, 1);
  lcd.print("I:");
  lcd.print(current, 2);
  lcd.print("A");
}

// Function to handle low voltage conditions (This is placeholder logic, needs to be implemented based on your system)
void handleLowVoltage() {
  lcd.setCursor(0, 1);
  lcd.print("Charging");
  digitalWrite(rled, HIGH);
  digitalWrite(rly, HIGH);
  delay(100);
  //delay(delayTime); // Replace with actual charging logic
  lcd.clear();
  digitalWrite(rly, LOW);
  digitalWrite(rled, LOW);
}

// Function to display messages on the LCD
void displayMessage(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
