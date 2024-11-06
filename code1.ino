#include <Adafruit_NeoPixel.h> // Include the NeoPixel library
#include <Servo.h> // Include the Servo library

// Pin assignments for NeoPixel strip
const int neoPixelPin = 12; // Pin connected to NeoPixel strip
const int numPixels = 4;     // Number of pixels in the strip
Adafruit_NeoPixel strip(numPixels, neoPixelPin, NEO_GRB + NEO_KHZ800);

// Pin assignments for IR sensors
const int irSensor1 = A0; // IR Sensor 1 (Right)
const int irSensor2 = A1; // IR Sensor 2 (Middle)
const int irSensor3 = A2; // IR Sensor 3 (Left)

// Pin assignments for motors
const int motorA_IN1 = 4;   // Motor A (Right Wheel) direction control pin
const int motorA_IN2 = 5;   // Motor A direction control pin
const int motorA_EN = 3;    // Motor A speed control (PWM)

const int motorB_IN3 = 6;   // Motor B (Left Wheel) direction control pin
const int motorB_IN4 = 7;   // Motor B direction control pin
const int motorB_EN = 9;    // Motor B speed control (PWM)

// Pin assignments for ultrasonic sensor
const int trigPin = 10; // Trigger pin
const int echoPin = 11; // Echo pin

// Pin assignment for servo
const int servoPin = 2; // Pin connected to the servo
Servo myServo; // Create a servo object

// Pin assignment for the water pump simulation (buzzer)
const int pumpPin = 8; // Choose a digital pin for controlling the buzzer

void setup() {
  Serial.begin(9600); // Start serial communication for debugging
  
  // Initialize NeoPixel strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  // Initialize the servo
  myServo.attach(servoPin);
  myServo.write(90); // Set servo to middle position

  // Set motor control pins as outputs
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_EN, OUTPUT);
  
  pinMode(motorB_IN3, OUTPUT);
  pinMode(motorB_IN4, OUTPUT);
  pinMode(motorB_EN, OUTPUT);

  // Set IR sensor pins as inputs
  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);
  pinMode(irSensor3, INPUT);

  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set pump pin as output
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW); // Ensure the buzzer is off initially

  // Start with both motors off
  stopMotors();
}

void loop() {
  // Get distance from the ultrasonic sensor
  long distance = getDistance();

  // Print distance to the Serial Monitor
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  // Read values from the IR sensors
  int value1 = analogRead(irSensor1); // A0 - Right
  int value2 = analogRead(irSensor2); // A1 - Middle
  int value3 = analogRead(irSensor3); // A2 - Left

  // Print IR sensor values to the Serial Monitor
  Serial.print("Sensor 1 (Right) Value: "); Serial.print(value1);
  Serial.print(" | Sensor 2 (Middle) Value: "); Serial.print(value2);
  Serial.print(" | Sensor 3 (Left) Value: "); Serial.println(value3);

  // Define a threshold for fire detection
  const int fireThreshold = 500; // Adjust this based on your testing

  // If the vehicle is moving and the distance is greater than 20 cm
  if (distance > 20) {
    chaseNeoPixels(255, 0, 0); // Create a chasing effect with red
    moveForward(255); // Move forward at full speed
    Serial.println("Moving forward.");
  } else {
    // Stop the motors if the distance is less than or equal to 20 cm
    Serial.println("Stopping due to proximity.");
    stopMotors();

    // Determine the direction of the fire and move the servo
    if (value1 < fireThreshold) { // Fire detected on the right
      Serial.println("Fire detected on the right! Turning servo right.");
      myServo.write(180); // Turn servo to the right
    } else if (value2 < fireThreshold) { // Fire detected in the middle
      Serial.println("Fire detected in the middle! Keeping servo straight.");
      myServo.write(90); // Keep servo straight
    } else if (value3 < fireThreshold) { // Fire detected on the left
      Serial.println("Fire detected on the left! Turning servo left.");
      myServo.write(0); // Turn servo to the left
    } else {
      myServo.write(90); // No fire detected, reset servo to middle
    }

    // Simulate the water pump with the buzzer when fire is detected nearby
    if (value1 < fireThreshold || value2 < fireThreshold || value3 < fireThreshold) {
      Serial.println("Activating pump to extinguish fire.");
      digitalWrite(pumpPin, HIGH); // Turn on the buzzer to simulate the pump
      delay(5000); // Keep the buzzer on for 5 seconds
      digitalWrite(pumpPin, LOW); // Turn off the buzzer
    }

    setNeoPixelColor(0, 0, 0); // Turn off NeoPixel strip
  }

  delay(500); // Small delay for stability
}

// Function to create a chasing effect on the NeoPixel strip
void chaseNeoPixels(int r, int g, int b) {
  for (int i = 0; i < numPixels; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b)); // Set the current pixel to the specified color
    strip.show(); // Update the strip
    delay(200); // Wait for a short period
    strip.setPixelColor(i, strip.Color(0, 0, 0)); // Turn off the current pixel
  }
}

// Function to set the NeoPixel color
void setNeoPixelColor(int r, int g, int b) {
  for(int i = 0; i < numPixels; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

// Function to get the distance from the ultrasonic sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration * 0.034) / 2; // Distance in cm
  return distance;
}

// Function to move both motors forward
void moveForward(int speed) {
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_EN, speed);

  digitalWrite(motorB_IN3, HIGH);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_EN, speed);
}

// Function to stop both motors
void stopMotors() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  analogWrite(motorA_EN, 0);

  digitalWrite(motorB_IN3, LOW);
  digitalWrite(motorB_IN4, LOW);
  analogWrite(motorB_EN, 0);
}
