#include <Arduino_FreeRTOS.h>
#include <DHT.h>
#include <Servo.h>

// -----------------------
// Pin definitions
// -----------------------
const int TRIG_FLOOR = 2;
const int ECHO_FLOOR = 3;

const int TRIG_BASIN = 4;
const int ECHO_BASIN = 5;

const int DHTPIN     = 7;
const int DHTTYPE    = DHT11;

const int SERVO_PIN  = 9;

// -----------------------
// Hardware objects
// -----------------------
DHT dht(DHTPIN, DHTTYPE);
Servo dispenserServo;

// -----------------------
// System mode state machine
// -----------------------
enum SystemMode {
  MODE_DETECTING = 0,
  MODE_DISP_READY,
  MODE_DISPENSING
};

volatile SystemMode currentMode = MODE_DETECTING;

// Shared sensor data (updated by sensor task)
volatile float floorDistanceCm = NAN;
volatile float basinDistanceCm = NAN;
volatile float humidityPercent = NAN;

// Thresholds (tune these!)
const float FLOOR_DETECT_MIN_CM = 2.0f;
const float FLOOR_DETECT_MAX_CM = 80.0f;    // "something moving on floor"
const float BASIN_DETECT_MIN_CM = 2.0f;
const float BASIN_DETECT_MAX_CM = 40.0f;    // "something in basin"
const float HUMIDITY_MAX_FOR_FEED = 80.0f;

// -----------------------
// Helper: sonar reading
// -----------------------
float readSonarCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // 25 ms timeout

  if (duration == 0) {
    return NAN; // no echo
  }
  return (duration * 0.0343f) / 2.0f;
}

// -----------------------
// FreeRTOS: Sensor Task
//  - Periodically updates sonar & DHT values
// -----------------------
void TaskSensors(void *pvParameters) {
  (void) pvParameters;

  unsigned long lastDhtMs = 0;

  for (;;) {
    // Update sonars every cycle
    float f = readSonarCm(TRIG_FLOOR, ECHO_FLOOR);
    float b = readSonarCm(TRIG_BASIN, ECHO_BASIN);

    // Simple atomic copy (RA4 is 32-bit so float writes are atomic enough here)
    floorDistanceCm = f;
    basinDistanceCm = b;

    // Update DHT every 2 seconds
    unsigned long now = millis();
    if (now - lastDhtMs >= 2000) {
      lastDhtMs = now;
      float h = dht.readHumidity();
      if (!isnan(h)) {
        humidityPercent = h;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
  }
}

// -----------------------
// Helper: detection logic
// -----------------------
bool floorMovementDetected() {
  float d = floorDistanceCm;
  return !isnan(d) && d >= FLOOR_DETECT_MIN_CM && d <= FLOOR_DETECT_MAX_CM;
}

bool basinMotionDetected() {
  float d = basinDistanceCm;
  return !isnan(d) && d >= BASIN_DETECT_MIN_CM && d <= BASIN_DETECT_MAX_CM;
}

bool humidityOKForFeeding() {
  float h = humidityPercent;
  return !isnan(h) && h <= HUMIDITY_MAX_FOR_FEED;
}

// -----------------------
// Helper: run dispensing cycle (servo)
//   - This runs inside the logic task when we enter MODE_DISPENSING
// -----------------------
void runDispenseCycle() {
  Serial.println("[Dispensing] Starting dispense cycle...");

  // Example sequence: move from 0° -> 90° -> 0°
  dispenserServo.write(0);
  vTaskDelay(pdMS_TO_TICKS(300));

  dispenserServo.write(90);   // open gate / tilt feeder
  vTaskDelay(pdMS_TO_TICKS(1000));

  dispenserServo.write(0);    // close
  vTaskDelay(pdMS_TO_TICKS(500));

  Serial.println("[Dispensing] Done. Returning to Detecting mode.");
}

// -----------------------
// FreeRTOS: Logic Task
//   - Implements the 3-mode state machine
// -----------------------
void TaskLogic(void *pvParameters) {
  (void) pvParameters;

  SystemMode lastMode = MODE_DETECTING;

  for (;;) {
    SystemMode mode = currentMode;

    // On-mode-entry actions
    if (mode != lastMode) {
      switch (mode) {
        case MODE_DETECTING:
          Serial.println("== Mode 1: Detecting ==");
          break;
        case MODE_DISP_READY:
          Serial.println("== Mode 2: Dispensing Ready ==");
          break;
        case MODE_DISPENSING:
          Serial.println("== Mode 3: Dispensing ==");
          break;
      }
      lastMode = mode;
    }

    switch (mode) {
      case MODE_DETECTING: {
        // Requirement:
        //   Monitor floor sonar only.
        //   If movement detected → Dispensing Ready.
        if (floorMovementDetected()) {
          Serial.println("[Detecting] Movement on floor detected → Dispensing Ready");
          currentMode = MODE_DISP_READY;
        }
        break;
      }

      case MODE_DISP_READY: {
        // Requirement:
        //   If humidity <= 80% AND basin sonar detects motion → Dispensing.
        //   Otherwise → Detecting.
        bool humOK   = humidityOKForFeeding();
        bool basinOK = basinMotionDetected();

        Serial.print("[Disp Ready] Humidity=");
        Serial.print(humidityPercent);
        Serial.print("%  BasinDist=");
        Serial.print(basinDistanceCm);
        Serial.print("cm  -> humOK=");
        Serial.print(humOK);
        Serial.print(" basinOK=");
        Serial.println(basinOK);

        if (humOK && basinOK) {
          Serial.println("[Disp Ready] Criteria met → Dispensing");
          currentMode = MODE_DISPENSING;
        } else {
          Serial.println("[Disp Ready] Criteria NOT met → Detecting");
          currentMode = MODE_DETECTING;
        }
        break;
      }

      case MODE_DISPENSING: {
        // Requirement:
        //   Ignore sensors and run motor to dispense.
        runDispenseCycle();
        // After dispense, go back to Detecting
        currentMode = MODE_DETECTING;
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // State machine tick: 5 Hz
  }
}

// -----------------------
// Arduino setup & FreeRTOS start
// -----------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("Terrarium Feeder - 3 Mode State Machine (FreeRTOS)");

  pinMode(TRIG_FLOOR, OUTPUT);
  pinMode(ECHO_FLOOR, INPUT);
  pinMode(TRIG_BASIN, OUTPUT);
  pinMode(ECHO_BASIN, INPUT);

  dht.begin();

  dispenserServo.attach(SERVO_PIN);
  dispenserServo.write(0); // safe default position

  // Start in Detecting Mode
  currentMode = MODE_DETECTING;

  // Create tasks
  xTaskCreate(TaskSensors, "Sensors", 512, NULL, 2, NULL);
  xTaskCreate(TaskLogic,   "Logic",   512, NULL, 1, NULL);

  // Start scheduler
  vTaskStartScheduler();
}

void loop() {
  // Not used - FreeRTOS is running
}
