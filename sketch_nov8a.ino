#include <Servo.h>

// ---------- Pin assignments ----------
const int rainPin        = 4;   // Digital input from rain module (LOW = rain)
const int servoPin       = 5;   // Servo signal
const int soilPin        = A0;  // Analog soil moisture
const int pump1RelayPin  = 12;   // Relay for Pump 1 (soil)
const int trigPin        = 3;  // Ultrasonic trigger
const int echoPin        = 2;  // Ultrasonic echo
const int pump2RelayPin  = 11;   // Relay for Pump 2 (ultrasonic)

// ---------- Relay logic level ----------
const int RELAY_ON_LEVEL  = LOW;
const int RELAY_OFF_LEVEL = HIGH;

// ---------- Behavior settings ----------
const int  soilThreshold  = 900;       // (testing threshold)
const long pingTimeoutUs  = 30000L;    // ~5m max; safety timeout for pulseIn

// Hysteresis thresholds for ultrasonic
const int onThresholdCm   = 4;         // Turn Pump2 ON below this distance
const int offThresholdCm  = 7;         // Turn Pump2 OFF above this distance

// Servo angles
const int servoOpenDeg    = 90;        // Angle when raining
const int servoCloseDeg   = 0;         // Angle when not raining

// Optional smoothing
const uint8_t ULTRA_SAMPLES = 5;       // average N readings

Servo gateServo;

// Track state for Pump 2 (ultrasonic)
bool pump2On = false;

void setRelay(int pin, bool on) {
  digitalWrite(pin, on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
}

long measureDistanceCm() {
  // Send a clean trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo with timeout
  unsigned long duration = pulseIn(echoPin, HIGH, pingTimeoutUs);
  if (duration == 0) {
    return -1;  // timeout / invalid
  }

  // Speed of sound ~343 m/s -> 29.1 us per cm (round-trip), /2 for one-way
  long cm = duration / 29 / 2;
  return cm;
}

long averagedDistanceCm(uint8_t samples) {
  long   sum  = 0;
  uint8_t good = 0;

  for (uint8_t i = 0; i < samples; i++) {
    long d = measureDistanceCm();
    if (d > 0) {
      sum  += d;
      good++;
    }
    delay(20);
  }

  if (good == 0) return -1;
  return sum / good;
}

void setup() {
  Serial.begin(115200);

  pinMode(rainPin, INPUT);
  pinMode(pump1RelayPin, OUTPUT);
  pinMode(pump2RelayPin, OUTPUT);
  setRelay(pump1RelayPin, false);
  setRelay(pump2RelayPin, false);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  gateServo.attach(servoPin);
  gateServo.write(servoCloseDeg);  // start closed
}

void loop() {
  // -------- Rain -> Servo --------
  int rainState = digitalRead(rainPin);  // LOW = rain detected
  if (rainState == LOW) {
    gateServo.write(servoOpenDeg);
  } else {
    gateServo.write(servoCloseDeg);
  }

  // -------- Soil moisture -> Pump 1 --------
  int  soilReading   = analogRead(soilPin);
  bool pump1ShouldOn = (soilReading > soilThreshold);
  setRelay(pump1RelayPin, pump1ShouldOn);

  // -------- Ultrasonic -> Pump 2 with hysteresis --------
  long dist = averagedDistanceCm(ULTRA_SAMPLES);
  if (dist > 0) {
    if (!pump2On && dist < onThresholdCm) {
      pump2On = true;
    } else if (pump2On && dist > offThresholdCm) {
      pump2On = false;
    }
    setRelay(pump2RelayPin, pump2On);
  }
  // If distance invalid, leave Pump 2 state unchanged

  // -------- Debug prints (optional) --------
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Rain(D): ");
    Serial.print(rainState == LOW ? "RAIN" : "DRY");
    Serial.print(" | Soil(A): ");
    Serial.print(soilReading);
    Serial.print(" | Dist(cm): ");
    Serial.print(dist);
    Serial.print(" | P1: ");
    Serial.print(pump1ShouldOn ? "ON" : "OFF");
    Serial.print(" | P2: ");
    Serial.println(pump2On ? "ON" : "OFF");
  }

  delay(100);  // small loop delay
}
