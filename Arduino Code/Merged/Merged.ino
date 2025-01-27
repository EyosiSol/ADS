#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Pin definitions for SoftwareSerial and buttons
#define txPin 2            // TX pin for second virtual terminal (VT2)
#define rxPin 3            // RX pin for second virtual terminal (VT2)
#define BUTTON_PIN 4       // Pushbutton pin (for emergency)
#define TILT_BUTTON_PIN 5  // Second Pushbutton pin (used to replace tilt switch)

#define triggerLED 11      // Pin for Trigger LED
#define sensorRightPin A0  // Potentiometer simulating Sensor Right
#define sensorLeftPin A1   // Potentiometer simulating Sensor Left
#define ledRight 9         // PWM pin for Haptic Driver Right
#define ledLeft 10         // PWM pin for Haptic Driver Left

// GPS & GPRS Setup
SoftwareSerial vt2(rxPin, txPin);
SoftwareSerial GPRS(6, 7); // RX=pin 6, TX=pin 7 for GSM
SoftwareSerial serial_connection(8, 9); // RX=pin 8, TX=pin 9 for GPS
TinyGPSPlus gps;

// Global Variables
float latitude = 0.0;  // Default latitude
float longitude = 0.0; // Default longitude
bool emergencyTriggered = false;
bool emergencyHandled = false;  // New flag to track if emergency has been handled
// Variables for potentiometers and distances
int lastDistanceRight = -1;
int lastDistanceLeft = -1;

// Variables for tilt button logic
unsigned long tiltButtonPressStartTime = 0;  // To store when the tilt button was first pressed
bool tiltButtonPressed = false;  // Flag to indicate if the tilt button is being pressed

// Function to calculate brightness based on distance
int calculateBrightness(int distance) {
  return map(distance, 0, 400, 255, 0); // Map 0-400 cm to 255-0 brightness
}

// Handle emergency scenario
void handleEmergency() {
  if (emergencyHandled) return;  // Skip if already handled this emergency

  for (int i = 0; i < 2; i++) {
    // Calling emergency number
    vt2.println("Calling emergency...");  // Change Serial to vt2
    GPRS.print("ATD+xxxxxxxxxxxx;\r"); // Replace with actual number
    delay(2000); // Give it time to connect
    GPRS.print("ATH\r"); // Disconnect call after dialing
    vt2.println("Disconnect");  // Change Serial to vt2

    // Check if GPS location is available
    if (gps.location.isUpdated()) {
      // Send emergency message with real location details
      vt2.print("Accident Detected; ");
      vt2.print("Location; ");
      vt2.print("Latitude - ");
      vt2.println(gps.location.lat());
      vt2.println(", Longitude - ");
      vt2.println(gps.location.lng());

      vt2.print(" http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
      vt2.print(gps.location.lat());
      vt2.print("+");
      vt2.println(gps.location.lng());
    } else {
      // If GPS location is not available, use default location
      vt2.print("Accident Detected; ");
      vt2.print("Location; ");
      vt2.print("Latitude - ");
      vt2.println(latitude);  // Default latitude
      vt2.println(", Longitude - ");
      vt2.println(longitude); // Default longitude

      vt2.print(" http://maps.google.com/maps?&z=15&mrt=yp&t=k&q=");
      vt2.print(latitude);  // Default latitude
      vt2.print("+");
      vt2.println(longitude); // Default longitude
    }

    GPRS.write(26); // End of message signal
    delay(2000); // Wait before next call attempt to avoid rapid firing
  }

  emergencyHandled = true;  // Mark this emergency as handled
  Serial.println("Emergency handled, waiting for system reset.");
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  vt2.begin(9600);  // Change vt2 to your virtual terminal baud rate

  // Initialize GPS and GPRS communication
  serial_connection.begin(9600);
  GPRS.begin(9600);

  // Set pin modes
  pinMode(sensorRightPin, INPUT);
  pinMode(sensorLeftPin, INPUT);
  pinMode(ledRight, OUTPUT);
  pinMode(ledLeft, OUTPUT);
  pinMode(triggerLED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(TILT_BUTTON_PIN, INPUT);
}

void loop() {
  // Read potentiometer values
  int potValueRight = analogRead(sensorRightPin);
  int potValueLeft = analogRead(sensorLeftPin);

  // Convert potentiometer values to distances (0-400 cm)
  int distanceRight = map(potValueRight, 0, 1023, 0, 400);
  int distanceLeft = map(potValueLeft, 0, 1023, 0, 400);

  // Calculate brightness based on distance
  int brightnessRight = calculateBrightness(distanceRight);
  int brightnessLeft = calculateBrightness(distanceLeft);

  // Apply brightness to LEDs
  analogWrite(ledRight, brightnessRight);
  analogWrite(ledLeft, brightnessLeft);

  // Trigger LED logic: If either distance is below 300 cm, turn it on
  if (distanceRight <= 300 || distanceLeft <= 300) {
    digitalWrite(triggerLED, HIGH); // Turn on trigger LED
  } else {
    digitalWrite(triggerLED, LOW);  // Turn off trigger LED
  }

  // Show distances on virtual terminals
  static int prevDistanceRight = -1;
  static int prevDistanceLeft = -1;

  if (distanceRight != prevDistanceRight) {
    Serial.print("Right Distance: ");
    Serial.print(distanceRight);
    Serial.println(" cm");
    prevDistanceRight = distanceRight;
  }

  if (distanceLeft != prevDistanceLeft) {
    vt2.print("Left Distance: ");
    vt2.print(distanceLeft);
    vt2.println(" cm");
    prevDistanceLeft = distanceLeft;
  }

  delay(100); // Small delay for stability

  // GPS Update logic
  while (serial_connection.available()) { // While there are characters from the GPS
    gps.encode(serial_connection.read()); // Feed NMEA data into the library one char at a time
  }

  if (gps.location.isUpdated()) { // Update location info when available
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
    Serial.println("");
  } else {
    Serial.println("No GPS signal, using default location.");
    Serial.print("Default Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Default Longitude: ");
    Serial.println(longitude, 6);
    Serial.println("");
  }

  // Check for pushbutton press for immediate emergency alert (first button)
  if (digitalRead(BUTTON_PIN) == HIGH && !emergencyHandled) {
    Serial.println("First push button pressed");
    emergencyTriggered = true;
    handleEmergency();
    delay(1000); // Debounce delay to prevent multiple triggers
  }

  // Tilt button logic: Detect if the button is held for 3 seconds
  if (digitalRead(TILT_BUTTON_PIN) == HIGH) {
    if (!tiltButtonPressed) {
      // Start counting time when the tilt button is first pressed
      tiltButtonPressStartTime = millis();
      tiltButtonPressed = true;
      Serial.println("Tilt button pressed, starting timer.");
    }

    // Check if the button has been pressed for 3 seconds
    if (millis() - tiltButtonPressStartTime >= 3000 && !emergencyHandled) {
      Serial.println("Second push button pressed for 3 seconds.");
      emergencyTriggered = true;
      handleEmergency();
      delay(1000); // Debounce delay to prevent multiple triggers
    }
  } else {
    if (tiltButtonPressed) {
      // If the tilt button is not pressed anymore, reset the flag and the timer
      Serial.println("Tilt button released, timer reset.");
      tiltButtonPressed = false;  // Reset button pressed state
      tiltButtonPressStartTime = 0;  // Reset the timer
    }
  }

  // Reset the emergency state if both buttons are released
  if (digitalRead(BUTTON_PIN) == LOW && digitalRead(TILT_BUTTON_PIN) == LOW) {
    // Only reset if we were in an emergency state
    if (emergencyTriggered) {
      emergencyTriggered = false;
      emergencyHandled = false;  // Reset the handled flag
      Serial.println("Both buttons released, system reset.");
    }
  }

  delay(100); // Small delay for stability
}