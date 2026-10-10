#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Konfigurasi Wi-Fi Access Point
const char* ssid = "PenyiramTanaman_ESP8266";
const char* password = "password123";  // minimal 8 karakter

ESP8266WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);    // alamat I2C 0x27, LCD 16x2

const int sensorPin = A0;   // Pin sensor kelembaban tanah	
const int relayPin = D5;    // Pin pengontrol relay

// Batasan kelembaban (Persentase)
const int batasKering = 30; // Pompa menyala jika di bawah 30%
const int batasBasah = 70;  // Pompa mati jika di atas 70%

bool modeManual = false;       // Status mode (otomatis / manual)
bool statusPompaManual = HIGH; // Status relay saat manual (HIGH = mati)

int sensorValue = 0;           // Nilai mentah sensor (0-1023)
int moisturePercent = 0;       // Kelembaban dalam persen
unsigned long waktuBaca = 0;   // Penanda waktu pembacaan sensor

// Membaca sensor (rata-rata 10 kali supaya stabil)
void bacaSensor() {
  long total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(sensorPin);
    delay(2);
  }
  sensorValue = total / 10;
  moisturePercent = map(sensorValue, 1023, 0, 0, 100);
  moisturePercent = constrain(moisturePercent, 0, 100);
}

bool pompaHidup() {
  return (digitalRead(relayPin) == LOW);  // Asumsi relay active LOW
}

// Tampilkan data ke LCD
void tampilLCD() {
  char baris1[17];
  char baris2[17];

  snprintf(baris1, sizeof(baris1), "Lembab:%3d%% %s", moisturePercent, modeManual ? "MAN" : "OTO");
  snprintf(baris2, sizeof(baris2), "Pompa : %-5s", pompaHidup() ? "NYALA" : "MATI");

  lcd.setCursor(0, 0);
  lcd.print(baris1);
  lcd.print("   ");   // hapus sisa karakter lama
  lcd.setCursor(0, 1);
  lcd.print(baris2);
  lcd.print("   ");
}

// Arahkan browser kembali ke halaman utama
void kembaliKeHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleRoot() {
  String html = "<html lang='id'>";
  html += "<head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Kontrol Pompa Tanaman</title>";
  html += "<meta http-equiv='refresh' content='4'>"; // Refresh otomatis tiap 4 detik
  html += "<style>body{font-family:Arial; text-align:center; margin-top:30px; background-color:#f4f4f9;}";
  html += ".card{background:white; padding:20px; border-radius:10px; display:inline-block; box-shadow: 0px 0px 10px #aaa; max-width:350px; width:100%;}";
  html += ".value{font-size:42px; color:#2e7d32; font-weight:bold; margin: 10px 0;}";
  html += ".kecil{font-size:13px; color:#777;}";
  html += ".btn{display:inline-block; padding:10px 20px; margin:5px; font-size:16px; color:white; text-decoration:none; border-radius:5px; border:none; cursor:pointer;}";
  html += ".btn-on{background-color:#d9534f;} .btn-off{background-color:#5cb85c;} .btn-mode{background-color:#f0ad4e;}";
  html += "</style></head><body>";
  html += "<div class='card'>";
  html += "<h2>Siram Tanaman Otomatis</h2>";
  html += "<p>Kelembaban Tanah:</p>";
  html += "<div class='value'>" + String(moisturePercent) + "%</div>";
  html += "<p class='kecil'>Nilai sensor: " + String(sensorValue) + " | Batas: " + String(batasKering) + "% - " + String(batasBasah) + "%</p>";
  html += "<p>Mode: <strong>" + String(modeManual ? "Manual" : "Otomatis") + "</strong></p>";
  html += "<p>Pompa: <strong>" + String(pompaHidup() ? "MENYALA" : "MATI") + "</strong></p>";

  // Tombol ganti mode
  html += "<a class='btn btn-mode' href='/mode'>Ganti ke Mode " + String(modeManual ? "Otomatis" : "Manual") + "</a><br>";

  // Tombol pompa hanya muncul saat mode manual
  if (modeManual) {
    html += "<a class='btn btn-on' href='/on'>Pompa ON</a>";
    html += "<a class='btn btn-off' href='/off'>Pompa OFF</a>";
  }

  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

// Ganti mode otomatis <-> manual
void handleMode() {
  modeManual = !modeManual;
  if (modeManual) {
    statusPompaManual = digitalRead(relayPin);  // pertahankan kondisi pompa terakhir
  }
  tampilLCD();
  kembaliKeHome();
}

// Pompa ON (otomatis pindah ke mode manual)
void handlePompaOn() {
  modeManual = true;
  statusPompaManual = LOW;                      // active LOW = ON
  digitalWrite(relayPin, statusPompaManual);
  tampilLCD();
  kembaliKeHome();
}

// Pompa OFF (otomatis pindah ke mode manual)
void handlePompaOff() {
  modeManual = true;
  statusPompaManual = HIGH;                     // HIGH = OFF
  digitalWrite(relayPin, statusPompaManual);
  tampilLCD();
  kembaliKeHome();
}

void setup() {
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);                 // Pompa mati saat awal (relay active LOW)

  // LCD
  Wire.begin(D2, D1);                           // SDA = D2, SCL = D1
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Siram Tanaman");
  lcd.setCursor(0, 1);
  lcd.print("Menyiapkan...");

  // Jalankan ESP8266 sebagai Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println();
  Serial.print("Access Point aktif: ");
  Serial.println(ssid);
  Serial.print("Buka alamat: http://");
  Serial.println(WiFi.softAPIP());              // biasanya 192.168.4.1

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/on", handlePompaOn);
  server.on("/off", handlePompaOff);
  server.begin();
  Serial.println("Web server dimulai");

  // Tampilkan alamat web di LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wi-Fi: Penyiram");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.softAPIP());
  delay(3000);
  lcd.clear();

  bacaSensor();
  tampilLCD();
}

void loop() {
  server.handleClient();

  // Baca sensor, kontrol pompa, dan perbarui LCD tiap 1 detik
  if (millis() - waktuBaca >= 1000) {
    waktuBaca = millis();
    bacaSensor();

    // Logika Kontrol Otomatis (jalan terus walau web tidak dibuka)
    if (!modeManual) {
      if (moisturePercent < batasKering) {
        digitalWrite(relayPin, LOW);    // Relay Aktif (pompa nyala)
      } else if (moisturePercent > batasBasah) {
        digitalWrite(relayPin, HIGH);   // Relay Mati (pompa mati)
      }
    }

    tampilLCD();

    Serial.print("Sensor: ");
    Serial.print(sensorValue);
    Serial.print(" | Kelembaban: ");
    Serial.print(moisturePercent);
    Serial.print("% | Mode: ");
    Serial.print(modeManual ? "Manual" : "Otomatis");
    Serial.print(" | Pompa: ");
    Serial.println(pompaHidup() ? "MENYALA" : "MATI");
  }
}
