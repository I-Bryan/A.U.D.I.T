// A.U.D.I.T. - Automatic Ultrasonic Density and Integrity Tester
// Controller firmware skeleton (UNTESTED). Target: ESP32-S3-WROOM-1 on the A.U.D.I.T. controller board.
//
// What it does:
//   - drives 3 TMC2209 drivers in STEP/DIR mode (X, Y, Z)
//   - homes X min, Y min, Z max using the three endstop headers
//   - switches PUMP1 (fill pump) and PUMP2 (drain valve) and reads the LEVEL input
//   - raster scan "stop and shoot": move, settle, fire the ultrasonic front end via FE_TRIG
//   - simple text commands over USB serial (see handleCommand)
//
// Pin numbers are IO labels from the controller board pin map.

#include <Arduino.h>
#include <AccelStepper.h>

// ---------------- Pin map (IO labels) ----------------
constexpr int PIN_X_STEP = 4,  PIN_X_DIR = 5;
constexpr int PIN_Y_STEP = 6,  PIN_Y_DIR = 7;
constexpr int PIN_Z_STEP = 15, PIN_Z_DIR = 16;
constexpr int PIN_EN_ALL = 17;            // shared driver enable, active LOW on TMC2209
constexpr int PIN_ENDSTOP_X = 9, PIN_ENDSTOP_Y = 10, PIN_ENDSTOP_Z = 11;  // 10k pull-up, switch to GND
constexpr int PIN_PUMP1 = 12;             // fill pump gate drive (24 V, <= 2 A)
constexpr int PIN_PUMP2 = 13;             // drain valve gate drive (24 V, <= 2 A)
constexpr int PIN_LEVEL = 14;             // float switch, 10k pull-up on the board
constexpr int PIN_TEMP  = 21;             // DS18B20 (1-Wire) - not used yet
constexpr int PIN_FE_SCK = 39, PIN_FE_MOSI = 40, PIN_FE_MISO = 41, PIN_FE_CS = 42; // front-end SPI (TODO)
constexpr int PIN_FE_TRIG = 47;           // trigger out to pulser/receiver
constexpr int PIN_FE_RDY  = 48;           // data-ready in from front end
constexpr int PIN_LED = 38;

// ---------------- Mechanics (CHECK THESE against your hardware) ----------------
// TMC2209 modules in STEP/DIR mode with MS1/MS2 unconnected: default microstepping depends on the
// module (many default to 1/8). Measure and set this to match your modules.
constexpr float MICROSTEPS = 8.0f;
constexpr float FULL_STEPS_PER_REV = 200.0f;            // 1.8 deg motors
constexpr float STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;
constexpr float BELT_MM_PER_REV = 40.0f;                // GT2, 20-tooth pulley
constexpr float Z_LEAD_MM = 2.0f;                       // T8 lead screw lead per turn (CHECK your actuator)
constexpr float X_STEPS_PER_MM = STEPS_PER_REV / BELT_MM_PER_REV;
constexpr float Y_STEPS_PER_MM = STEPS_PER_REV / BELT_MM_PER_REV;
constexpr float Z_STEPS_PER_MM = STEPS_PER_REV / Z_LEAD_MM;

constexpr float X_TRAVEL_MM = 180.0f, Y_TRAVEL_MM = 180.0f, Z_TRAVEL_MM = 160.0f;  // soft limits
// Flip these if an axis moves the wrong way
constexpr bool X_INVERT = false, Y_INVERT = false, Z_INVERT = false;
// Homing: X and Y home toward min (negative), Z homes at the top (Z=0 is top, positive = down)
constexpr float HOME_SPEED_MM_S = 10.0f, HOME_BACKOFF_MM = 3.0f;
constexpr float MOVE_SPEED_MM_S = 40.0f, MOVE_ACCEL_MM_S2 = 300.0f;
constexpr float Z_SPEED_MM_S = 5.0f, Z_ACCEL_MM_S2 = 20.0f;

// ---------------- Water system ----------------
constexpr bool LEVEL_FULL_WHEN_LOW = true;              // float switch closes to GND when full (CHECK)
constexpr uint32_t FILL_TIMEOUT_MS = 300000;            // 5 min safety stop
constexpr uint32_t DRAIN_TIME_MS   = 240000;            // 4 min

AccelStepper stepX(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, PIN_Y_STEP, PIN_Y_DIR);
AccelStepper stepZ(AccelStepper::DRIVER, PIN_Z_STEP, PIN_Z_DIR);

bool homed = false;

// ---------------- Helpers ----------------
bool endstopHit(int pin) { return digitalRead(pin) == LOW; }
bool tankFull() { return (digitalRead(PIN_LEVEL) == LOW) == LEVEL_FULL_WHEN_LOW; }
void drivers(bool on) { digitalWrite(PIN_EN_ALL, on ? LOW : HIGH); }
void pump(bool on)  { digitalWrite(PIN_PUMP1, on ? HIGH : LOW); }
void valve(bool open) { digitalWrite(PIN_PUMP2, open ? HIGH : LOW); }

void setupAxis(AccelStepper& s, bool invert, float stepsPerMm, float speed, float accel) {
  s.setPinsInverted(invert, false, false);
  s.setMaxSpeed(speed * stepsPerMm);
  s.setAcceleration(accel * stepsPerMm);
}

// Move until the endstop triggers. dir = -1 or +1 (in machine coordinates). Returns false on timeout.
bool homeAxis(AccelStepper& s, int pin, float stepsPerMm, int dir, float maxTravelMm) {
  const long maxSteps = (long)((maxTravelMm + 20.0f) * stepsPerMm);
  s.setSpeed(dir * HOME_SPEED_MM_S * stepsPerMm);
  long count = 0;
  while (!endstopHit(pin)) {
    if (s.runSpeed()) { if (++count > maxSteps) return false; }
  }
  s.setCurrentPosition(0);
  s.moveTo(-dir * (long)(HOME_BACKOFF_MM * stepsPerMm));
  s.runToPosition();
  s.setCurrentPosition(0);
  return true;
}

bool homeAll() {
  drivers(true);
  // Z first (top), so the probe is clear of the tank before X/Y move
  bool ok = homeAxis(stepZ, PIN_ENDSTOP_Z, Z_STEPS_PER_MM, -1, Z_TRAVEL_MM + 100.0f);
  ok = ok && homeAxis(stepX, PIN_ENDSTOP_X, X_STEPS_PER_MM, -1, X_TRAVEL_MM + 50.0f);
  ok = ok && homeAxis(stepY, PIN_ENDSTOP_Y, Y_STEPS_PER_MM, -1, Y_TRAVEL_MM + 50.0f);
  homed = ok;
  return ok;
}

void runUntilStopped() {
  while (stepX.isRunning() || stepY.isRunning() || stepZ.isRunning()) {
    stepX.run(); stepY.run(); stepZ.run();
  }
}

bool moveTo(float x, float y, float z) {
  if (!homed) { Serial.println("ERR not homed"); return false; }
  if (x < 0 || x > X_TRAVEL_MM || y < 0 || y > Y_TRAVEL_MM || z < 0 || z > Z_TRAVEL_MM) {
    Serial.println("ERR out of range"); return false;
  }
  stepX.moveTo((long)(x * X_STEPS_PER_MM));
  stepY.moveTo((long)(y * Y_STEPS_PER_MM));
  stepZ.moveTo((long)(z * Z_STEPS_PER_MM));
  runUntilStopped();
  return true;
}

// ---------------- Water system ----------------
bool fillTank() {
  valve(false);
  uint32_t t0 = millis();
  pump(true);
  while (!tankFull()) {
    if (millis() - t0 > FILL_TIMEOUT_MS) { pump(false); Serial.println("ERR fill timeout"); return false; }
    delay(50);
  }
  pump(false);
  return true;
}

void drainTank() {
  pump(false);
  valve(true);
  delay(DRAIN_TIME_MS);
  valve(false);
}

// ---------------- Ultrasonic front end (stub) ----------------
// Fires one pulse and waits for the front end to report data ready.
// TODO: read the waveform over SPI (FE_SCK/MOSI/MISO/CS) once the front-end board exists.
bool fireAndRead(uint32_t timeoutMs, int& rdy) {
  digitalWrite(PIN_FE_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_FE_TRIG, LOW);
  uint32_t t0 = millis();
  while (digitalRead(PIN_FE_RDY) == LOW) {
    if (millis() - t0 > timeoutMs) { rdy = 0; return false; }
  }
  rdy = 1;
  return true;
}

// Raster scan, stop-and-shoot. Prints one "P x y rdy" line per point.
void scan(float x0, float y0, float x1, float y1, float step, float z, uint32_t settleMs) {
  if (!moveTo(x0, y0, z)) return;
  bool forward = true;
  for (float y = y0; y <= y1 + 0.001f; y += step) {
    for (int i = 0; ; i++) {
      float x = forward ? x0 + i * step : x1 - i * step;
      if ((forward && x > x1 + 0.001f) || (!forward && x < x0 - 0.001f)) break;
      if (!moveTo(x, y, z)) return;
      delay(settleMs);
      int rdy = 0;
      fireAndRead(50, rdy);
      Serial.printf("P %.2f %.2f %d\n", x, y, rdy);
    }
    forward = !forward;
  }
  Serial.println("SCAN DONE");
}

// ---------------- Serial commands ----------------
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  float a, b, c, d, e, f;
  if (line.equalsIgnoreCase("HOME")) {
    Serial.println(homeAll() ? "OK homed" : "ERR homing failed");
  } else if (sscanf(line.c_str(), "G %f %f %f", &a, &b, &c) == 3) {
    Serial.println(moveTo(a, b, c) ? "OK" : "ERR");
  } else if (line.equalsIgnoreCase("FILL")) {
    Serial.println(fillTank() ? "OK full" : "ERR");
  } else if (line.equalsIgnoreCase("DRAIN")) {
    drainTank(); Serial.println("OK drained");
  } else if (sscanf(line.c_str(), "PUMP %f", &a) == 1) {
    pump(a > 0.5f); Serial.println("OK");
  } else if (sscanf(line.c_str(), "VALVE %f", &a) == 1) {
    valve(a > 0.5f); Serial.println("OK");
  } else if (sscanf(line.c_str(), "SCAN %f %f %f %f %f %f", &a, &b, &c, &d, &e, &f) == 6) {
    // SCAN x0 y0 x1 y1 step z   (settle time is fixed at 150 ms)
    scan(a, b, c, d, e, f, 150);
  } else if (line.equalsIgnoreCase("OFF")) {
    drivers(false); pump(false); valve(false); homed = false; Serial.println("OK off");
  } else if (line.equalsIgnoreCase("STATUS")) {
    Serial.printf("homed=%d full=%d X=%d Y=%d Z=%d\n", homed, tankFull(),
                  endstopHit(PIN_ENDSTOP_X), endstopHit(PIN_ENDSTOP_Y), endstopHit(PIN_ENDSTOP_Z));
  } else {
    Serial.println("ERR unknown. Commands: HOME | G x y z | FILL | DRAIN | PUMP 0/1 | VALVE 0/1 | SCAN x0 y0 x1 y1 step z | STATUS | OFF");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN_ALL, OUTPUT); drivers(false);
  pinMode(PIN_PUMP1, OUTPUT); pinMode(PIN_PUMP2, OUTPUT); pump(false); valve(false);
  pinMode(PIN_ENDSTOP_X, INPUT); pinMode(PIN_ENDSTOP_Y, INPUT); pinMode(PIN_ENDSTOP_Z, INPUT);
  pinMode(PIN_LEVEL, INPUT);
  pinMode(PIN_FE_TRIG, OUTPUT); digitalWrite(PIN_FE_TRIG, LOW);
  pinMode(PIN_FE_RDY, INPUT);
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, HIGH);
  setupAxis(stepX, X_INVERT, X_STEPS_PER_MM, MOVE_SPEED_MM_S, MOVE_ACCEL_MM_S2);
  setupAxis(stepY, Y_INVERT, Y_STEPS_PER_MM, MOVE_SPEED_MM_S, MOVE_ACCEL_MM_S2);
  setupAxis(stepZ, Z_INVERT, Z_STEPS_PER_MM, Z_SPEED_MM_S, Z_ACCEL_MM_S2);
  Serial.println("A.U.D.I.T. controller ready. Type a command (e.g. STATUS).");
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { handleCommand(buf); buf = ""; }
    else if (ch != '\r') buf += ch;
  }
}
