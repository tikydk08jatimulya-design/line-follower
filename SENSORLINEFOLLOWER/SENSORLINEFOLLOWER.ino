// Pin sensor dari S8 (paling kiri) sampai S1 (paling kanan)
const int sensorPins[8] = {
  13, 14, 27, 26, 25, 33, 32, 4
};

// Menggunakan GPIO 18 untuk mengaktifkan LED IR pada sensor HY-S301
const int pinEnableIR = 18; 

int lastValues[8] = {0};
bool hasChanged[8] = {false};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Mengaktifkan LED inframerah pada sensor
  pinMode(pinEnableIR, OUTPUT);
  digitalWrite(pinEnableIR, HIGH); 
  
  Serial.println("=== MULAI TES SENSOR HY-S301 ===");
  Serial.println("Pin IR (GPIO 18) diposisikan HIGH. LED Sensor harusnya menyala.");
  Serial.println("Silakan gerakkan sensor di atas garis HITAM dan PUTIH bergantian.");
  Serial.println("----------------------------------------------------------------");

  // Inisialisasi pin sensor dengan INPUT_PULLUP agar pembacaan sinyal lebih stabil
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
    lastValues[i] = digitalRead(sensorPins[i]);
  }
}

void loop() {
  // 1. TAMPILAN NILAI REAL-TIME (S8 di kiri sampai S1 di kanan)
  Serial.print("Data [S8..S1]: ");
  for (int i = 0; i < 8; i++) {
    int currentValue = digitalRead(sensorPins[i]);
    
    if (currentValue != lastValues[i]) {
      hasChanged[i] = true;
      lastValues[i] = currentValue;
    }
    
    Serial.print("S");
    Serial.print(8 - i);
    Serial.print(":");
    Serial.print(currentValue);
    Serial.print("  ");
  }
  Serial.println();

  // 2. DIAGNOSIS KONDISI SENSOR (Muncul setiap 2 detik)
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 2000) {
    lastCheck = millis();
    
    Serial.println("\n--- STATUS DIAGNOSIS SENSOR ---");
    for (int i = 0; i < 8; i++) {
      Serial.print("Sensor S");
      Serial.print(8 - i);
      Serial.print(" (GPIO ");
      Serial.print(sensorPins[i]);
      Serial.print("): ");
      
      if (hasChanged[i]) {
        Serial.println("✅ NORMAL (Merespon Hitam/Putih)");
      } else {
        Serial.println("⚠️ BELUM MERESPON / DIAM (Putar trimpot/dekatkan ke garis)");
      }
    }
    Serial.println("--------------------------------\n");
  }

  // Jeda diubah ke 100ms agar data di Serial Monitor pembacaannya lebih responsif
  delay(300); 
}