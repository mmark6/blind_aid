#include <NewPing.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <SoftwareSerial.h>



// HC-SR04 Pins
#define TRIG_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 400 // in cm
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Adafruit_MPU6050 mpu;

// Vibration Motor Pin
#define VIB_PIN 9 

// SIM800L V2 EVB Pins
#define SIM_RX 11 
#define SIM_TX 10 
SoftwareSerial sim800l(SIM_RX, SIM_TX);

// Constants
const float DISTANCE_THRESHOLD = 200.0; 
const float MIN_DISTANCE = 2.0; // Minimum reliable distance (cm)
const float MAX_VIB_STRENGTH = 255.0;
const float MIN_VIB_STRENGTH = 50.0; 
const unsigned long SAMPLE_INTERVAL = 100; // Time between measurements (ms)

// Fall Detection Constants
const float FREE_FALL_THRESHOLD = 2.0; // m/s²
const float IMPACT_THRESHOLD = 29.4; // m/s²
const float TILT_THRESHOLD = 45.0; // Degrees (post-fall orientation)
const unsigned long FALL_WINDOW = 2000; 
const unsigned long SMS_COOLDOWN = 60000; 

// Variables
float ax_offset = 0.0; // Replace with your MPU6050 calibration value
float gx_offset = 0.0; // Replace with your MPU6050 calibration value
float lastDistance = 0.0;
unsigned long lastTime = 0;
float wearerSpeed = 0.0;
bool fallDetected = false;
unsigned long lastSMSTime = 0;

// Caregiver's number
const char* CAREGIVER_NUMBER = "+256751167441";

void setup() {
  Serial.begin(9600); 
  pinMode(VIB_PIN, OUTPUT);
  Serial.println("Vibration motor set up");

  // Initialize VL53L0X
  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X!");
  }
  // Serial.println("ToF ignored");

  // Serial.println("Initialize MPU6050");
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // ±2g for walking speed
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // ±250°/s for orientation
  ax_offset = ax_calibrate();
  gx_offset = gx_calibrate();

  // Initialize SIM800L
  sim800l.begin(9600);
  delay(1000);
  if (!initializeSIM800L()) {
    Serial.println("Failed to initialize SIM800L!");
  }
}

float ax_calibrate() {
  Serial.println("Calibrating accelerometre... Keep MPU6050 stationary for 5 seconds.");
  delay(2000); 
  
  float ax_sum = 0.0;
  int samples = 1000;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax_sum += a.acceleration.x;
    delay(5);
  }
  ax_offset = ax_sum / samples;
  Serial.print("X-Axis Offset: "); Serial.println(ax_offset);
  return ax_offset;
}

float gx_calibrate() {
  Serial.println("Calibrating gyro... Keep MPU6050 stationary for 5 seconds.");
  delay(2000); 

  float gx_sum = 0.0;
  int samples = 1000;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx_sum += g.gyro.x;
    delay(5);
  }
  gx_offset = gx_sum / samples;
  Serial.print("Gyro offset: "); Serial.println(gx_offset);
  return gx_offset;
}

bool initializeSIM800L() {
  // Test communication with SIM800L
  sim800l.println("AT");
  delay(100);
  if (sim800l.find("OK")) {
    // check signal quality
    sim800l.println("AT+CSQ");
    // Set SMS to text mode
    sim800l.println("AT+CMGF=1");
    delay(100);
    return sim800l.find("OK");
  }
  return false;
}

void sendSMS(float distance) {
  if (millis() - lastSMSTime < SMS_COOLDOWN) return; // Prevent multiple SMS
  char message[100];
  snprintf(message, sizeof(message), "ALERT: Fall detected! Last distance to obstacle: %.1f cm", distance);
  sim800l.println("AT+CMGS=\"" + String(CAREGIVER_NUMBER) + "\"");
  delay(100);
  sim800l.print(message);
  delay(100);
  sim800l.write(26); // End sending SMS
  delay(1000);
  if (sim800l.find("OK")) {
    Serial.println("SMS sent successfully!");
  } else {
    Serial.println("Failed to send SMS!");
  }
  lastSMSTime = millis();
}

float getTemperature() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return temp.temperature; // °C
}

float getWearerSpeed() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x - ax_offset;
  static float filtered_ax = 0.0;
  const float alpha = 0.1; // Low-pass filter coefficient
  filtered_ax = alpha * ax + (1 - alpha) * filtered_ax;
  
  static float velocity = 0.0;
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  
  if (abs(filtered_ax) < 0.1) { // Stationary threshold
    velocity = 0.0;
  } else if (lastUpdate != 0) {
    float dt = (now - lastUpdate) / 1000.0; // To seconds
    velocity += filtered_ax * dt;
    velocity = constrain(velocity, -2.0, 2.0);
  }
  lastUpdate = now;
  return velocity; // m/s
}

bool detectFall() {
  static unsigned long fallStartTime = 0;
  static bool freeFallDetected = false;
  static bool impactDetected = false;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate total acceleration magnitude
  float acc_magnitude = sqrt(sq(a.acceleration.x - ax_offset) + 
                            sq(a.acceleration.y) + 
                            sq(a.acceleration.z));
  
  // Calculate tilt angle from gyroscope
  float tilt = abs(g.gyro.x - gx_offset) * (SAMPLE_INTERVAL / 1000.0) * 180.0 / PI;
  static float total_tilt = 0.0;
  total_tilt += tilt;
  
  // Fall detection logic
  if (!freeFallDetected && acc_magnitude < FREE_FALL_THRESHOLD) {
    freeFallDetected = true;
    fallStartTime = millis();
  } else if (freeFallDetected && !impactDetected && acc_magnitude > IMPACT_THRESHOLD) {
    impactDetected = true;
  } else if (freeFallDetected && impactDetected && total_tilt > TILT_THRESHOLD) {
    if (millis() - fallStartTime <= FALL_WINDOW) {
      freeFallDetected = false;
      impactDetected = false;
      total_tilt = 0.0;
      return true; 
    }
  }
  
  // Reset if fall sequence times out
  if (freeFallDetected && (millis() - fallStartTime > FALL_WINDOW)) {
    freeFallDetected = false;
    impactDetected = false;
    total_tilt = 0.0;
  }
  
  return false;
}

float getDistance() {
  // Use the Time of Flight sensor if the object is closer than 2m
  if (lastDistance <= DISTANCE_THRESHOLD) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      float distance_mm = measure.RangeMilliMeter;
      float distance_cm = distance_mm / 10.0;
      if (distance_cm >= MIN_DISTANCE && distance_cm <= DISTANCE_THRESHOLD) {
        return distance_cm;
      }
    }
  }
  
  // Otherwise, use the ultrasonic sensor
  unsigned int duration = sonar.ping();
  float temperature = getTemperature();
  float speedOfSound = 331.4 + (0.606 * temperature); // m/s
  float distance_cm = (duration * (speedOfSound / 10000)) / 2; // cm
  if (distance_cm < MIN_DISTANCE || distance_cm > MAX_DISTANCE) {
    return -1.0;
  }
  return distance_cm;
}

float calculateObjectSpeed(float currentDistance, float deltaTime) {
  if (lastDistance == 0.0 || deltaTime == 0) return 0.0;
  float distanceChange = lastDistance - currentDistance; // cm
  float objectSpeed = distanceChange / (deltaTime / 1000.0); // cm/s to m/s
  return objectSpeed / 100.0;
}

void vibrateFeedback(float distance, float relativeSpeed) {
  if (distance < 0) {
    analogWrite(VIB_PIN, 0);
    return;
  }
  
  float strength = map(distance, MIN_DISTANCE, DISTANCE_THRESHOLD, MAX_VIB_STRENGTH, MIN_VIB_STRENGTH);
  strength = constrain(strength, MIN_VIB_STRENGTH, MAX_VIB_STRENGTH);
  
  if (abs(relativeSpeed) < 0.1) {
    analogWrite(VIB_PIN, (int)strength); // Continuous for stationary
  } else {
    float absSpeed = abs(relativeSpeed);
    long pulseDelay = map(absSpeed * 100, 0, 200, 500, 50);
    pulseDelay = constrain(pulseDelay, 50, 500);
    analogWrite(VIB_PIN, (int)strength);
    delay(pulseDelay / 2);
    analogWrite(VIB_PIN, 0);
    delay(pulseDelay / 2);
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= SAMPLE_INTERVAL) {
    float distance = getDistance();
    wearerSpeed = getWearerSpeed();
    float deltaTime = currentTime - lastTime;
    float objectSpeed = calculateObjectSpeed(distance, deltaTime);
    float relativeSpeed = objectSpeed - wearerSpeed;
    
    // Fall detection
    if (detectFall()) {
      fallDetected = true;
      sendSMS(distance);
    }
    
    // Vibration feedback (skip if fall detected to prioritize alert)
    if (!fallDetected) {
      vibrateFeedback(distance, relativeSpeed);
    }
    
    // Debug output
    Serial.print("Distance: "); Serial.print(distance); Serial.print(" cm, ");
    Serial.print("Object Speed: "); Serial.print(objectSpeed); Serial.print(" m/s, ");
    Serial.print("Wearer Speed: "); Serial.print(wearerSpeed); Serial.print(" m/s, ");
    Serial.print("Relative Speed: "); Serial.print(relativeSpeed); Serial.println(" m/s");
    if (fallDetected) Serial.println("FALL DETECTED!");
    
    lastDistance = distance;
    lastTime = currentTime;
  }
}
