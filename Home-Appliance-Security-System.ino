#include <SoftwareSerial.h>

// Define pins for Ultrasonic Sensors
// First pair (controls their own relay, buzzer, and GSM for SMS)
const int trigPinA1 = 7;
const int echoPinA1 = 6;
const int trigPinB1 = 9;
const int echoPinB1 = 8;
const int relayPin1 = 10;

// Second pair (controls their own relay, buzzer, and GSM for Call)
const int trigPinA2 = 12;
const int echoPinA2 = 13;
const int trigPinB2 = 14;
const int echoPinB2 = 15;
const int relayPin2 = 16;

// Third pair (controls their own relay, buzzer, and GSM for SMS)
const int trigPinA3 = 17;
const int echoPinA3 = 18;
const int trigPinB3 = 19;
const int echoPinB3 = 20;
const int relayPin3 = 21;

// Define pin for buzzer (shared)
const int buzzerPin = 11;

// Define a threshold distance (in cm)
const int detectionThreshold = 5;

// Pins for GSM Module (TX, RX - shared)
SoftwareSerial GSM(2, 3);  // RX, TX

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  GSM.begin(9600);  // Initialize GSM at 9600 baud rate

  // Set sensor pins as input/output for all sensors
  pinMode(trigPinA1, OUTPUT);
  pinMode(echoPinA1, INPUT);
  pinMode(trigPinB1, OUTPUT);
  pinMode(echoPinB1, INPUT);
  
  pinMode(trigPinA2, OUTPUT);
  pinMode(echoPinA2, INPUT);
  pinMode(trigPinB2, OUTPUT);
  pinMode(echoPinB2, INPUT);
  
  pinMode(trigPinA3, OUTPUT);
  pinMode(echoPinA3, INPUT);
  pinMode(trigPinB3, OUTPUT);
  pinMode(echoPinB3, INPUT);

  // Set relay pins as output
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);

  // Set buzzer pin as output (shared)
  pinMode(buzzerPin, OUTPUT);

  // Start with relays off and buzzer off
  digitalWrite(relayPin1, HIGH);  // Relay off
  digitalWrite(relayPin2, HIGH);  // Relay off
  digitalWrite(relayPin3, HIGH);  // Relay off
  digitalWrite(buzzerPin, LOW);   // Buzzer off

  // Test GSM module initialization
  GSM.println("AT");
  delay(1000);
  if (GSM.available()) {
    Serial.println("GSM module ready.");
  } else {
    Serial.println("No response from GSM module.");
  }
}

void loop() {
  // Get distance readings from each pair of sensors
  int distanceA1 = getDistance(trigPinA1, echoPinA1);
  int distanceB1 = getDistance(trigPinB1, echoPinB1);
  int distanceA2 = getDistance(trigPinA2, echoPinA2);
  int distanceB2 = getDistance(trigPinB2, echoPinB2);
  int distanceA3 = getDistance(trigPinA3, echoPinA3);
  int distanceB3 = getDistance(trigPinB3, echoPinB3);

  // Control first pair (SMS)
  if (distanceA1 < detectionThreshold && distanceB1 > detectionThreshold) {
    Serial.println("First pair: Toddler detected. Sending SMS, cutting power.");
    digitalWrite(relayPin1, LOW);  // Turn relay ON (cut power)
    digitalWrite(buzzerPin, HIGH); // Activate buzzer
    sendSMS("+250789652808", "Toddler detected - First Ultrasonic Sensors");
    delay(5000);                   // Keep state for 5 seconds
    digitalWrite(buzzerPin, LOW);  // Turn off buzzer
  } else {
    digitalWrite(relayPin1, HIGH); // Turn relay OFF (restore power)
  }

  // Control second pair (Call)
  if (distanceA2 < detectionThreshold && distanceB2 > detectionThreshold) {
    Serial.println("Second pair: Toddler detected. Making call, cutting power.");
    digitalWrite(relayPin2, LOW);  // Turn relay ON (cut power)
    digitalWrite(buzzerPin, HIGH); // Activate buzzer
    makeCall("+250789652808");
    delay(5000);                   // Keep state for 5 seconds
    digitalWrite(buzzerPin, LOW);  // Turn off buzzer
  } else {
    digitalWrite(relayPin2, HIGH); // Turn relay OFF (restore power)
  }

  // Control third pair (SMS)
  if (distanceA3 < detectionThreshold && distanceB3 > detectionThreshold) {
    Serial.println("Third pair: Toddler detected. Sending SMS, cutting power.");
    digitalWrite(relayPin3, LOW);  // Turn relay ON (cut power)
    digitalWrite(buzzerPin, HIGH); // Activate buzzer
    sendSMS("+250789652808", "Toddler detected - Third Ultrasonic Sensors");
    delay(5000);                   // Keep state for 5 seconds
    digitalWrite(buzzerPin, LOW);  // Turn off buzzer
  } else {
    digitalWrite(relayPin3, HIGH); // Turn relay OFF (restore power)
  }

  // Wait before next loop iteration
  delay(1000);
}

// Function to get the distance from an ultrasonic sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

// Function to send an SMS
void sendSMS(String phoneNumber, String message) {
  Serial.println("Sending SMS...");
  GSM.println("AT+CMGF=1");
  delay(1000);
  GSM.println("AT+CMGS=\"" + phoneNumber + "\"");
  delay(1000);
  GSM.println(message);
  delay(1000);
  GSM.write(26);  // ASCII code for CTRL+Z (end message)
  delay(5000);
  Serial.println("SMS sent.");
}

// Function to make a call
void makeCall(String phoneNumber) {
  Serial.println("Making call...");
  GSM.println("ATD" + phoneNumber + ";");
  delay(10000); // 10 seconds call duration
  GSM.println("ATH"); // Hang up call
  Serial.println("Call ended.");
}