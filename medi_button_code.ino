#include <Servo.h>
#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>

// Replace with your network credentials
#define WIFI_SSID "Shamindi"
#define WIFI_PASSWORD "Sha1008#"

// Replace with your Firebase project credentials
#define FIREBASE_HOST "medi-dispenser-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "xc9cgCMBdLUk3xUV9bMIH3iJ7gYBbm1utKZBU8EI"

// Buzzer pin
#define BUZZER_PIN D4

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Variables to store current positions of the servos and rotation counts
int servo1Position = 0;
int servo2Position = 0;
int servo3Position = 0;
int servo4Position = 0;

int servo1Count = 0;
int servo2Count = 0;
int servo3Count = 0;
int servo4Count = 0;

// Variables to track the state of each pill
bool previousPill1State = false;
bool previousPill2State = false;
bool previousPill3State = false;
bool previousPill4State = false;

// Timing variables
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval for updating Firebase values

void setup() {
  Serial.begin(9600);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi");

  // Initialize Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  // Attach servos to respective pins
  servo1.attach(D0); // Connect servo1 to D0
  servo2.attach(D5); // Connect servo2 to D1
  servo3.attach(D2); // Connect servo3 to D2
  servo4.attach(D3); // Connect servo4 to D3

  // Set initial servo positions
  servo1.write(0);  // Initialize servo1 to 0 degrees
  servo2.write(0);  // Initialize servo2 to 0 degrees
  servo3.write(0);  // Initialize servo3 to 0 degrees
  servo4.write(0);  // Initialize servo4 to 0 degrees

  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Save the last time you updated the Firebase values
    previousMillis = currentMillis;

    // Fetch and process the state of each pill from Firebase
    processPill("Pill1/status", servo1, servo1Position, servo1Count, previousPill1State);
    processPill("Pill2/status", servo2, servo2Position, servo2Count, previousPill2State);
    processPill("Pill3/status", servo3, servo3Position, servo3Count, previousPill3State);
    processPill("Pill4/status", servo4, servo4Position, servo4Count, previousPill4State);

    // Fetch and process the buzzer state from Firebase
    processBuzzer("Remind/status");
  }
}

void processPill(String path, Servo &servo, int &servoPosition, int &rotationCount, bool &previousPillState) {
  // Fetch pill state
  bool pillState = Firebase.getBool(path);
  if (Firebase.failed()) {
    Serial.print("Failed to get ");
    Serial.print(path);
    Serial.print(": ");
    Serial.println(Firebase.error());
    return;
  }
  Serial.print(path);
  Serial.print(" State: ");
  Serial.println(pillState);

  // Rotate servo if state is true
  if (pillState) {
    // Rotate servo 90 degrees for each activation
    rotationCount++;
    if (rotationCount > 3) {
      rotationCount = 1; // Reset count after 4 rotations
      servoPosition = 0; // Reset servo position to 0 degrees
      servo.write(servoPosition);
    } else {
      int targetPosition = rotationCount * 90; // Target position for current count
      rotateServo(servo, servoPosition, targetPosition); // Rotate to target position
      servoPosition = targetPosition; // Update position
    }
  }

  // Update the previous state
  previousPillState = pillState;
}

void processBuzzer(String path) {
  // Fetch buzzer state
  bool buzzerState = Firebase.getBool(path);
  if (Firebase.failed()) {
    Serial.print("Failed to get ");
    Serial.print(path);
    Serial.print(": ");
    Serial.println(Firebase.error());
    return;
  }
  Serial.print(path);
  Serial.print(" State: ");
  Serial.println(buzzerState);

  // Control the buzzer based on the state
  if (buzzerState) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
  } else {
    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer
  }
}

void rotateServo(Servo &servo, int currentPos, int targetPos) {
  // Rotate the servo to the target position
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      servo.write(pos);
      delay(30); // Delay to allow smoother servo movement
    }
  } else if (currentPos > targetPos) {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(30); // Delay to allow smoother servo movement
    }
  }
}
