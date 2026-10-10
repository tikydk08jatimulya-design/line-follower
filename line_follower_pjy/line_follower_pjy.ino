
#include <Arduino.h>

// =====================================================
// HIGH SPEED LINE FOLLOWER
// ESP32-S + DRV8833 + HYS301 8 CHANNEL
// Jalur: garis hitam di atas permukaan putih
// =====================================================

// ---------- PIN MOTOR DRV8833 ----------
constexpr uint8_t L_IN1 = 25;
constexpr uint8_t L_IN2 = 26;
constexpr uint8_t R_IN1 = 27;
constexpr uint8_t R_IN2 = 14;

// ---------- PIN SENSOR: KIRI -> KANAN ----------
constexpr uint8_t SENSOR_COUNT = 8;

const uint8_t sensorPins[SENSOR_COUNT] = {
  32, 33, 18, 19, 21, 22, 23, 13
};

// Sesuaikan setelah pengujian.
// LOW berarti sensor mendeteksi garis hitam.
constexpr uint8_t LINE_LEVEL = LOW;

// Balik menjadi true jika arah motor terbalik.
constexpr bool INVERT_LEFT  = false;
constexpr bool INVERT_RIGHT = false;

// ---------- PARAMETER KECEPATAN ----------
int baseSpeed = 105;
constexpr int MAX_PWM = 210;
constexpr int MIN_CORNER_SPEED = 65;
constexpr int SEARCH_PWM = 75;

// ---------- PARAMETER PID ----------
// Error posisi berkisar kira-kira -3.5 sampai +3.5.
float Kp = 38.0f;
float Ki = 0.0f;
float Kd = 0.45f;

// ---------- PARAMETER FILTER ----------
constexpr float DERIVATIVE_FILTER = 0.30f;
constexpr float INTEGRAL_LIMIT = 1.0f;

// ---------- BOBOT SENSOR ----------
const float weights[SENSOR_COUNT] = {
  -3.5f, -2.5f, -1.5f, -0.5f,
   0.5f,  1.5f,  2.5f,  3.5f
};

// ---------- STATUS ----------
float error = 0.0f;
float previousError = 0.0f;
float integral = 0.0f;
float filteredDerivative = 0.0f;

int lastDirection = 0; // -1 kiri, +1 kanan
bool everFoundLine = false;

uint32_t previousMicros = 0;
uint32_t previousPrint = 0;

// ---------- MOTOR CONTROL ----------
// Nilai positif = maju.
// Nilai negatif = mundur.
// Pastikan roda terangkat saat pengujian awal.
void setMotor(uint8_t pin1, uint8_t pin2,
              int pwm, bool invert) {
  pwm = constrain(pwm, -255, 255);

  if (invert) pwm = -pwm;

  if (pwm > 0) {
    analogWrite(pin1, pwm);
    analogWrite(pin2, 0);
  } else if (pwm < 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, -pwm);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

void driveMotors(int left, int right) {
  setMotor(L_IN1, L_IN2, left, INVERT_LEFT);
  setMotor(R_IN1, R_IN2, right, INVERT_RIGHT);
}

void stopMotors() {
  driveMotors(0, 0);
}

// ---------- SENSOR READING ----------
struct SensorResult {
  uint8_t mask;
  uint8_t count;
  bool found;
  float position;
};

SensorResult readSensors() {
  SensorResult result = {};
  float sum = 0.0f;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    bool active = (digitalRead(sensorPins[i]) == LINE_LEVEL);

    if (active) {
      result.mask |= (1U << i);
      result.count++;
      sum += weights[i];
    }
  }

  result.found = (result.count > 0);

  if (result.found) {
    result.position = sum / result.count;
  }

  return result;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  stopMotors();

  previousMicros = micros();

  Serial.println("ESP32 Line Follower");
  Serial.println("Sensor order: left -> right");
  Serial.println("Motors stopped; ready for testing.");
}

// ---------- MAIN LOOP ----------
void loop() {
  uint32_t now = micros();
  float dt = (uint32_t)(now - previousMicros) * 1.0e-6f;

  if (dt < 0.002f) return;

  previousMicros = now;

  SensorResult s = readSensors();

  // ==============================================
  // 1. GARIS HILANG
  // ==============================================
  if (!s.found) {
    integral = 0.0f;
    filteredDerivative = 0.0f;

    if (!everFoundLine) {
      stopMotors();
    } else if (lastDirection < 0) {
      // Cari ke kiri
      driveMotors(-SEARCH_PWM, SEARCH_PWM);
    } else if (lastDirection > 0) {
      // Cari ke kanan
      driveMotors(SEARCH_PWM, -SEARCH_PWM);
    } else {
      stopMotors();
    }

    printStatus(s, 0, 0, "LOST");
    return;
  }

  everFoundLine = true;
  error = s.position;

  if (error < -0.15f) lastDirection = -1;
  if (error >  0.15f) lastDirection = 1;

  // ==============================================
  // 2. PID DENGAN DERIVATIF TERFILTER
  // ==============================================
  integral += error * dt;
  integral = constrain(integral,
                       -INTEGRAL_LIMIT,
                        INTEGRAL_LIMIT);

  float rawDerivative = (error - previousError) / dt;

  filteredDerivative =
      (1.0f - DERIVATIVE_FILTER) * filteredDerivative
      + DERIVATIVE_FILTER * rawDerivative;

  float correction =
      Kp * error
      + Ki * integral
      + Kd * filteredDerivative;

  previousError = error;

  // ==============================================
  // 3. KECEPATAN ADAPTIF
  // ==============================================
  float absError = fabs(error);
  int targetSpeed = baseSpeed;

  // Turunkan kecepatan secara bertahap
  // ketika garis menjauh dari pusat sensor.
  if (absError > 2.5f) {
    targetSpeed = MIN_CORNER_SPEED;
  } else if (absError > 1.5f) {
    targetSpeed = min(targetSpeed, 80);
  } else if (absError > 0.8f) {
    targetSpeed = min(targetSpeed, 95);
  }

  // Pola sangat lebar mungkin persimpangan,
  // garis tebal, atau bagian lintasan khusus.
  // Perlambat tanpa memilih cabang secara acak.
  if (s.count >= 6) {
    targetSpeed = min(targetSpeed, 80);
  }

  // ==============================================
  // 4. KENDALI MOTOR
  // ==============================================
  int leftPWM = (int)(targetSpeed + correction);
  int rightPWM = (int)(targetSpeed - correction);

  // Izinkan putaran balik hanya ketika koreksi besar.
  leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
  rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

  driveMotors(leftPWM, rightPWM);

  printStatus(s, leftPWM, rightPWM, "TRACK");
}

// ---------- SERIAL MONITOR ----------
void printStatus(const SensorResult &s,
                 int leftPWM,
                 int rightPWM,
                 const char *mode) {
  if (millis() - previousPrint < 100) return;
  previousPrint = millis();

  Serial.print(mode);
  Serial.print(" | Mask=");

  for (int i = SENSOR_COUNT - 1; i >= 0; i--) {
    Serial.print((s.mask >> i) & 1U);
  }

  Serial.print(" | Pos=");
  Serial.print(s.position, 2);
  Serial.print(" | L=");
  Serial.print(leftPWM);
  Serial.print(" | R=");
  Serial.println(rightPWM);
}
