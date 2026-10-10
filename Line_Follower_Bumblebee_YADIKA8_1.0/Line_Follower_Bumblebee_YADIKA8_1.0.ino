#include <Arduino.h>

// PENGATURAN PIN ESP32-S (Sesuaikan dengan wiring Anda)
#define speedPinR          5   // Pin PWM Kanan
#define RightMotorDirPin1  4   // Arah Motor Kanan
#define RightMotorDirPin2  16  // Arah Motor Kanan

#define speedPinL          17  // Pin PWM Kiri
#define LeftMotorDirPin1   18  // Arah Motor Kiri
#define LeftMotorDirPin2   19  // Arah Motor Kiri

#define IRSensorL          34  // Pin Sensor IR Kiri (Input Only ESP32)
#define IRSensorM          35  // Pin Sensor IR Tengah (Input Only ESP32)
#define IRSensorR          32  // Pin Sensor IR Kanan (Input Only ESP32)

// PARAMETER PID (Silakan tune nilai ini sesuai mekanik robot Anda)
float Kp = 45.0;   // Proportional: Koreksi instan berdasarkan jarak error
float Ki = 0.0;    // Integral: Mengoreksi akumulasi error minor
float Kd = 25.0;   // Derivative: Meredam overshoot (getaran zig-zag)

// PARAMETER KECEPATAN (0 - 255)
int MAX_SPEED = 180;   // Kecepatan maksimal di jalur lurus
int BASE_SPEED = 130;  // Kecepatan dasar saat manuver

// Variabel Global PID
int lastError = 0;
float P, I, D, PID_value;
int lastDirection = 0; // 1 = Kanan, -1 = Kiri (Untuk memori jalur)

void setup() {
  
  Serial.begin(115200);

  // Setup Pin Motor
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  // Setup Pin Sensor
  pinMode(IRSensorL, INPUT);
  pinMode(IRSensorM, INPUT);
  pinMode(IRSensorR, INPUT);

  stopRobot();
  delay(1000); // Jeda sebelum mulai jalan
}

void loop() {
  // 1. Membaca kondisi sensor digital (Asumsi: Garis HITAM = HIGH, Lantai PUTIH = LOW)
  int L = digitalRead(IRSensorL);
  int M = digitalRead(IRSensorM);
  int R = digitalRead(IRSensorR);

  int error = 0;
  bool onLine = true;

  // 2. Kalkulasi Error Menggunakan Bobot Posisi (Weighted Error)
  if (L == LOW && M == HIGH && R == LOW) {
    error = 0;   // Tepat di tengah garis
  } 
  else if (L == HIGH && M == HIGH && R == LOW) {
    error = -1;  // Sedikit melenceng ke kanan (garis ada di kiri-tengah)
    lastDirection = -1;
  } 
  else if (L == HIGH && M == LOW && R == LOW) {
    error = -2;  // Melenceng jauh ke kanan (garis hanya di kiri)
    lastDirection = -1;
  } 
  else if (L == LOW && M == HIGH && R == HIGH) {
    error = 1;   // Sedikit melenceng ke kiri (garis ada di kanan-tengah)
    lastDirection = 1;
  } 
  else if (L == LOW && M == LOW && R == HIGH) {
    error = 2;   // Melenceng jauh ke kiri (garis hanya di kanan)
    lastDirection = 1;
  } 
  else if (L == HIGH && M == HIGH && R == HIGH) {
    error = 0;   // Persimpangan penuh, tetap maju lurus
  } 
  else if (L == LOW && M == LOW && R == LOW) {
    onLine = false; // Kehilangan garis sepenuhnya!
  }

  // 3. Logika Pintar: Jika Kehilangan Garis (Lost Line Recovery)
  if (!onLine) {
    // Memanfaatkan memori arah terakhir untuk memutar robot kembali ke garis
    if (lastDirection == -1) {
      spinLeft(BASE_SPEED, BASE_SPEED);  // Cari ke kiri jika terakhir condong kiri
    } else {
      spinRight(BASE_SPEED, BASE_SPEED); // Cari ke kanan jika terakhir condong kanan
    }
    return; // Lewati kalkulasi PID sementara waktu mencari garis
  }

  // 4. Kalkulasi Formula PID
  P = error;
  I = I + error;
  D = error - lastError;
  
  // Batasi nilai Integral agar tidak meluap (Anti Windup)
  I = constrain(I, -50, 50); 

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;

  // 5. Distribusi Kecepatan ke Motor Kiri dan Kanan
  int speedLeft = BASE_SPEED + PID_value;
  int speedRight = BASE_SPEED - PID_value;

  // Pastikan nilai PWM motor tetap berada di rentang aman 0 - MAX_SPEED
  speedLeft = constrain(speedLeft, 0, MAX_SPEED);
  speedRight = constrain(speedRight, 0, MAX_SPEED);

  // 6. Eksekusi Pergerakan Motor
  controlMotors(speedLeft, speedRight);
}

// ================= FUNGSI KONTROL MOTOR DRIVER =================

void controlMotors(int speedL, int speedR) {
  // Motor Kiri Maju
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speedL);

  // Motor Kanan Maju
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, speedR);
}

void spinRight(int speedL, int speedR) {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speedL);
  analogWrite(speedPinR, speedR);
}

void spinLeft(int speedL, int speedR) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, speedL);
  analogWrite(speedPinR, speedR);
}

void stopRobot() {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
}
