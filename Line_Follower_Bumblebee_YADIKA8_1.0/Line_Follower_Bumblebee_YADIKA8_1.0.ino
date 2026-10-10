#include <Arduino.h>
#include <esp_arduino_version.h>
#include <math.h>

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "Pin mapping hanya untuk ESP32 klasik WROOM/DevKit."
#endif
#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Gunakan Arduino-ESP32 core 3.x."
#endif

namespace Config {
constexpr uint8_t SENSOR_COUNT = 8;
constexpr uint8_t SENSOR_PINS[] = {36, 39, 34, 35, 32, 33, 13, 4};
constexpr uint8_t LEFT_IN1 = 25, LEFT_IN2 = 26, RIGHT_IN1 = 27, RIGHT_IN2 = 14;
constexpr bool INVERT_LEFT = false, INVERT_RIGHT = false;
constexpr int BASE_PWM = 100, CORNER_PWM = 65, MAX_PWM = 180, SEARCH_PWM = 70;
constexpr float KP = 30.0f, KI = 0.0f, KD = 0.15f;
constexpr float INTEGRAL_LIMIT = 1.0f, DERIVATIVE_TIME_CONSTANT_S = 0.015f;
constexpr uint32_t CONTROL_INTERVAL_US = 4000, CONTROL_TIMEOUT_US = 100000;
constexpr uint32_t CALIBRATION_DURATION_MS = 8000, SEARCH_TIMEOUT_MS = 700;
constexpr uint16_t MIN_CALIBRATION_SPAN = 300, ACTIVE_THRESHOLD = 450;
}

enum class RobotState : uint8_t { IDLE, CALIBRATING, RUNNING, SEARCHING, STOPPED, FAULT };
struct SensorReading {
  uint16_t raw[Config::SENSOR_COUNT] = {};
  uint8_t activeCount = 0, groups = 0;
  bool found = false;
  float position = 0.0f;
};

RobotState state = RobotState::IDLE;
SensorReading sensors;
uint16_t calibrationMin[Config::SENSOR_COUNT], calibrationMax[Config::SENSOR_COUNT];
bool calibrated = false, motorsReady = false, pidInitialized = false, telemetryEnabled = true;
float previousError = 0.0f, integral = 0.0f, filteredDerivative = 0.0f;
int lastDirection = 0, commandedLeft = 0, commandedRight = 0;
uint32_t previousControlUs = 0, calibrationStartedMs = 0, searchStartedMs = 0, previousTelemetryMs = 0;

void resetPid() {
  previousError = integral = filteredDerivative = 0.0f;
  pidInitialized = false;
}

void writeMotor(uint8_t in1, uint8_t in2, int pwm, bool invert) {
  pwm = constrain(pwm, -Config::MAX_PWM, Config::MAX_PWM);
  if (invert) pwm = -pwm;
  // Matikan input arah lawan sebelum memberi PWM untuk arah baru.
  if (pwm > 0) {
    ledcWrite(in2, 0);
    ledcWrite(in1, pwm);
  } else {
    ledcWrite(in1, 0);
    ledcWrite(in2, -pwm);
  }
}

void driveMotors(int left, int right) {
  commandedLeft = constrain(left, -Config::MAX_PWM, Config::MAX_PWM);
  commandedRight = constrain(right, -Config::MAX_PWM, Config::MAX_PWM);
  if (!motorsReady) return;
  writeMotor(Config::LEFT_IN1, Config::LEFT_IN2, commandedLeft, Config::INVERT_LEFT);
  writeMotor(Config::RIGHT_IN1, Config::RIGHT_IN2, commandedRight, Config::INVERT_RIGHT);
}

void stopRobot() {
  driveMotors(0, 0);
  resetPid();
  if (state != RobotState::FAULT) state = RobotState::STOPPED;
}

void readSensors() {
  sensors = SensorReading{};
  uint32_t totalStrength = 0;
  float weightedStrength = 0.0f;
  bool previousActive = false;
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    sensors.raw[i] = analogRead(Config::SENSOR_PINS[i]);
    if (!calibrated) continue;
    const int span = calibrationMax[i] - calibrationMin[i];
    const int normalized = (static_cast<int>(sensors.raw[i]) - calibrationMin[i]) * 1000 / span;
    const uint16_t strength = constrain(normalized, 0, 1000);
    const bool active = strength >= Config::ACTIVE_THRESHOLD;
    if (active) {
      ++sensors.activeCount;
      if (!previousActive) ++sensors.groups;
    }
    previousActive = active;
    // Abaikan pantulan latar lemah agar centroid tidak tertarik ke pusat array.
    if (strength >= 200) {
      totalStrength += strength;
      weightedStrength += (static_cast<float>(i) - 3.5f) * strength;
    }
  }
  sensors.found = sensors.activeCount > 0;
  if (totalStrength > 0) sensors.position = weightedStrength / totalStrength;
}

void beginCalibration() {
  stopRobot();
  calibrated = false;
  lastDirection = 0;
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    calibrationMin[i] = 4095;
    calibrationMax[i] = 0;
  }
  calibrationStartedMs = millis();
  state = RobotState::CALIBRATING;
  Serial.println("CAL: geser manual SEMUA sensor melewati putih dan hitam selama 8 detik. Motor berhenti.");
}

void updateCalibration() {
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    calibrationMin[i] = min(calibrationMin[i], sensors.raw[i]);
    calibrationMax[i] = max(calibrationMax[i], sensors.raw[i]);
  }
  if (static_cast<uint32_t>(millis() - calibrationStartedMs) < Config::CALIBRATION_DURATION_MS) return;
  calibrated = true;
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) {
    Serial.printf("S%u min=%u max=%u\n", i + 1, calibrationMin[i], calibrationMax[i]);
    if (calibrationMax[i] - calibrationMin[i] < Config::MIN_CALIBRATION_SPAN) calibrated = false;
  }
  state = RobotState::IDLE;
  Serial.println(calibrated ? "CAL OK. Letakkan di garis, kirim r untuk start." : "CAL GAGAL. Periksa wiring/ketinggian lalu ulangi c.");
}

void processCommands() {
  for (uint8_t processed = 0; processed < 16 && Serial.available(); ++processed) {
    const char command = Serial.read();
    if (command == 's') {
      stopRobot();
      Serial.println("STOP");
    } else if (command == 'c' && state != RobotState::FAULT) {
      beginCalibration();
    } else if (command == 'r') {
      if (state == RobotState::RUNNING || state == RobotState::SEARCHING) continue;
      if (!motorsReady || !calibrated || state == RobotState::CALIBRATING) {
        Serial.println("START ditolak: PWM harus siap dan kalibrasi harus selesai.");
        continue;
      }
      readSensors();
      if (!sensors.found || sensors.groups != 1 || sensors.activeCount >= 6) {
        Serial.println("START ditolak: letakkan robot di satu garis sempit.");
        continue;
      }
      resetPid();
      lastDirection = sensors.position < -0.15f ? -1 : (sensors.position > 0.15f ? 1 : 0);
      previousControlUs = micros();
      state = RobotState::RUNNING;
      Serial.println("RUN");
    } else if (command == 'd') telemetryEnabled = !telemetryEnabled;
  }
}

void updateTracking(float dt) {
  if (!sensors.found) {
    if (state != RobotState::SEARCHING) {
      searchStartedMs = millis();
      resetPid();
      state = RobotState::SEARCHING;
    }
    if (lastDirection == 0 || static_cast<uint32_t>(millis() - searchStartedMs) >= Config::SEARCH_TIMEOUT_MS) {
      stopRobot();
      Serial.println("STOP: garis hilang. Letakkan kembali di garis lalu kirim r.");
      return;
    }
    driveMotors(lastDirection * Config::SEARCH_PWM, -lastDirection * Config::SEARCH_PWM);
    return;
  }
  // Cabang terpisah dan area hitam lebar memerlukan aturan lintasan khusus.
  if (sensors.groups > 1 || sensors.activeCount >= 6) {
    stopRobot();
    Serial.println("STOP: pola persimpangan/garis lebar ambigu.");
    return;
  }
  state = RobotState::RUNNING;
  const float error = sensors.position;
  if (error < -0.15f) lastDirection = -1;
  else if (error > 0.15f) lastDirection = 1;
  if (!pidInitialized) {
    previousError = error;
    pidInitialized = true;
  }
  const float alpha = dt / (Config::DERIVATIVE_TIME_CONSTANT_S + dt);
  filteredDerivative += alpha * ((error - previousError) / dt - filteredDerivative);
  previousError = error;
  const int targetSpeed = static_cast<int>(Config::BASE_PWM - (Config::BASE_PWM - Config::CORNER_PWM) * min(fabsf(error) / 3.5f, 1.0f));
  const float candidateIntegral = constrain(integral + error * dt, -Config::INTEGRAL_LIMIT, Config::INTEGRAL_LIMIT);
  const float candidateCorrection = Config::KP * error + Config::KI * candidateIntegral + Config::KD * filteredDerivative;
  if (fabsf(candidateCorrection) <= Config::MAX_PWM - targetSpeed) integral = candidateIntegral;
  const float correction = Config::KP * error + Config::KI * integral + Config::KD * filteredDerivative;
  driveMotors(static_cast<int>(targetSpeed + correction), static_cast<int>(targetSpeed - correction));
}

void printTelemetry() {
  if (!telemetryEnabled || state == RobotState::RUNNING || state == RobotState::SEARCHING) return;
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - previousTelemetryMs) < 250) return;
  previousTelemetryMs = now;
  Serial.printf("state=%u cal=%u raw=", static_cast<unsigned>(state), calibrated ? 1U : 0U);
  for (uint8_t i = 0; i < Config::SENSOR_COUNT; ++i) Serial.printf("%u%s", sensors.raw[i], i == 7 ? "" : ",");
  Serial.printf(" pos=%.2f active=%u groups=%u L=%d R=%d\n", sensors.position, sensors.activeCount, sensors.groups, commandedLeft, commandedRight);
}

void setup() {
  Serial.begin(115200);
  const uint8_t motorPins[] = {Config::LEFT_IN1, Config::LEFT_IN2, Config::RIGHT_IN1, Config::RIGHT_IN2};
  for (const uint8_t pin : motorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  motorsReady = true;
  for (const uint8_t pin : motorPins) {
    if (!ledcAttach(pin, 20000, 8)) motorsReady = false;
  }
  if (!motorsReady) {
    for (const uint8_t pin : motorPins) {
      ledcDetach(pin);
      digitalWrite(pin, LOW);
    }
    state = RobotState::FAULT;
    Serial.println("FAULT: inisialisasi PWM gagal. Restart setelah diperiksa.");
  }
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (const uint8_t pin : Config::SENSOR_PINS) pinMode(pin, INPUT);
  previousControlUs = micros();
  Serial.println("QTR-8A + DRV8833 | c=kalibrasi, r=start, s=stop, d=telemetri idle");
  Serial.println("Sensor kiri->kanan: 36,39,34,35,32,33,13,4. Jangan aktifkan Wi-Fi (ADC2).");
}

void loop() {
  processCommands();
  const uint32_t now = micros();
  const uint32_t elapsed = now - previousControlUs;
  if (elapsed < Config::CONTROL_INTERVAL_US) return;
  previousControlUs = now;
  if ((state == RobotState::RUNNING || state == RobotState::SEARCHING) && elapsed > Config::CONTROL_TIMEOUT_US) {
    stopRobot();
    Serial.println("STOP: jeda kontrol terlalu lama.");
  }
  readSensors();
  if (state == RobotState::CALIBRATING) updateCalibration();
  else if (state == RobotState::RUNNING || state == RobotState::SEARCHING) updateTracking(elapsed * 1.0e-6f);
  printTelemetry();
}
