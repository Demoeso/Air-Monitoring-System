
#include <LiquidCrystal.h>
#include <EEPROM.h>

// ================== Part 1: Sensor & LED Pins ==================

// Sensors
const int tempSensorPin = A0;  // TMP36
const int humSensorPin  = A1;  // HIH-5030

// LEDs for temperature
const int tempRedLedPin  = 5;  // "too hot"
const int tempBlueLedPin = 3;  // "too cold"

// LEDs for humidity
const int humRedLedPin   = 6;  // "too humid"
const int humBlueLedPin  = 4;  // "too dry"

// ================== Part 2: LCD & Switch Pins ==================
LiquidCrystal lcd(12, 11, 7, 8, 9, 10);  

// Toggle switch for °C/°F
const int switchPin = 2;  // Using internal pull-up

// ================== Extra: Memory Reset Button ==================
// We'll use digital pin 13 for a reset button (with internal pull-up)
const int resetButtonPin = 13;

// ================== Part 3: EEPROM Ring Buffer ==================
#define MAX_SAMPLES 168

// Storing two int16_t values per sample: temperature (tenths of °C) + humidity (tenths of %)
int16_t eepromCount; // how many samples are stored (0..168)
int16_t eepromHead;  // index of the oldest sample (0..167)

// EEPROM Addresses
const int ADDR_COUNT = 0;  // 2 bytes
const int ADDR_HEAD  = 2;  // 2 bytes
const int ADDR_DATA  = 4;  // from 4..(4 + 168*4 - 1) => 4..675

// ========== Part 2 (RAM Arrays) for average/variance calculation ==========
float temperatureData[MAX_SAMPLES];
float humidityData[MAX_SAMPLES];
int totalSamples = 0;  // same as eepromCount, but in RAM

// ========== Thresholds for LED logic (in °C and %RH) ==========
float HOT_THRESHOLD   = 30.0; // °C
float COLD_THRESHOLD  = 0.0;  // °C
float DRY_THRESHOLD   = 20.0; // %
float HUMID_THRESHOLD = 70.0; // %


// --------------------------------------------------------------
// Helper: Write a 16-bit integer to EEPROM 
void eepromWriteInt16(int addr, int16_t value) {
  EEPROM.write(addr,   (byte)(value & 0xFF));
  EEPROM.write(addr+1, (byte)((value >> 8) & 0xFF));
}

// Helper: Read a 16-bit integer from EEPROM 
int16_t eepromReadInt16(int addr) {
  int16_t low  = EEPROM.read(addr);
  int16_t high = EEPROM.read(addr+1);
  return (int16_t)((high << 8) | (low & 0xFF));
}

// --------------------------------------------------------------
// Initialize eepromCount, eepromHead from EEPROM
void initEepromData() {
  eepromCount = eepromReadInt16(ADDR_COUNT);
  eepromHead  = eepromReadInt16(ADDR_HEAD);

  // Validate (in case EEPROM was never written or has junk)
  if (eepromCount < 0 || eepromCount > MAX_SAMPLES) {
    eepromCount = 0;
  }
  if (eepromHead < 0 || eepromHead >= MAX_SAMPLES) {
    eepromHead = 0;
  }
}

// --------------------------------------------------------------
// Store eepromCount, eepromHead to EEPROM
void saveEepromMeta() {
  eepromWriteInt16(ADDR_COUNT, eepromCount);
  eepromWriteInt16(ADDR_HEAD,  eepromHead);
}

// --------------------------------------------------------------
// Write one sample (tempTenths, humTenths) into the ring buffer
void storeSampleInEeprom(int16_t tempTenths, int16_t humTenths) {
  // If not full, increment count. If full, overwrite oldest => increment head
  if (eepromCount < MAX_SAMPLES) {
    eepromCount++;
  } else {
    // Move head forward (overwriting oldest)
    eepromHead = (eepromHead + 1) % MAX_SAMPLES;
  }

  // Where to write new sample: (head + count - 1) % MAX_SAMPLES
  int writeIndex = (eepromHead + eepromCount - 1) % MAX_SAMPLES;

  // Calculate byte address in EEPROM (4 bytes per record)
  int sampleAddr = ADDR_DATA + writeIndex * 4;

  // Write temperature, then humidity
  eepromWriteInt16(sampleAddr,     tempTenths);
  eepromWriteInt16(sampleAddr + 2, humTenths);

  // Update metadata in EEPROM
  saveEepromMeta();
}

// --------------------------------------------------------------
// Read all samples from EEPROM into temperatureData/humidityData arrays
void loadAllSamplesFromEeprom() {
  totalSamples = eepromCount; 
  for (int i = 0; i < totalSamples; i++) {
    // The i-th sample in chronological order is at index = (head + i) % MAX_SAMPLES
    int indexInRing = (eepromHead + i) % MAX_SAMPLES;
    int sampleAddr  = ADDR_DATA + indexInRing * 4;

    int16_t tempTenths = eepromReadInt16(sampleAddr);
    int16_t humTenths  = eepromReadInt16(sampleAddr + 2);

    // Convert back to float
    temperatureData[i] = tempTenths / 10.0;
    humidityData[i]    = humTenths  / 10.0;
  }
}

// ===================== PART 1: Sensor & Calculation =====================

// TMP36 => returns temperature in Celsius
float readTemperatureC() {
  int rawValue = analogRead(tempSensorPin);
  float voltage = rawValue * (5.0 / 1023.0);
  return (voltage - 0.5) * 100.0;
}

// HIH-5030 => approximate humidity
float readHumidity() {
  int rawValue = analogRead(humSensorPin);
  float voltage = rawValue * (5.0 / 1023.0);
  return (voltage / 5.0) * 100.0;
}

// Compute average & variance of a float array
void computeStats(const float data[], int count, float &avg, float &var) {
  if (count == 0) {
    avg = 0.0;
    var = 0.0;
    return;
  }
  float sum = 0.0;
  for (int i = 0; i < count; i++) {
    sum += data[i];
  }
  avg = sum / count;

  float sumSq = 0.0;
  for (int i = 0; i < count; i++) {
    float diff = data[i] - avg;
    sumSq += diff * diff;
  }
  var = sumSq / count;
}

// =========================== SETUP ===========================
void setup() {
  // LED pins
  pinMode(tempRedLedPin,  OUTPUT);
  pinMode(tempBlueLedPin, OUTPUT);
  pinMode(humRedLedPin,   OUTPUT);
  pinMode(humBlueLedPin,  OUTPUT);
  
  // Turn them off initially
  digitalWrite(tempRedLedPin,  LOW);
  digitalWrite(tempBlueLedPin, LOW);
  digitalWrite(humRedLedPin,   LOW);
  digitalWrite(humBlueLedPin,  LOW);

  // Switch pin for °C/°F toggle
  pinMode(switchPin, INPUT_PULLUP);

  // Memory reset button pin
  pinMode(resetButtonPin, INPUT_PULLUP);

  // LCD
  lcd.begin(20, 4);

  // Serial debug
  Serial.begin(9600);
  Serial.println("Air Monitoring - Part 3 (EEPROM ring buffer)");

  // 1) Read eepromCount/eepromHead from EEPROM
  initEepromData();
  
  // 2) Load all samples from EEPROM into RAM arrays
  loadAllSamplesFromEeprom();

  // Display a start message
  lcd.setCursor(0, 0);
  lcd.print("Air Monitor Pt3");
  lcd.setCursor(0, 1);
  lcd.print("EEPROM Loading...");
  delay(2000);
}

// =========================== LOOP ===========================
void loop() {
  // ----- Check Reset Button -----
  if (digitalRead(resetButtonPin) == LOW) {
    // Reset EEPROM ring buffer metadata and RAM arrays
    eepromCount = 0;
    eepromHead  = 0;
    saveEepromMeta();
    totalSamples = 0;
    for (int i = 0; i < MAX_SAMPLES; i++) {
      temperatureData[i] = 0;
      humidityData[i] = 0;
    }
    Serial.println("Memory Reset Triggered");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Memory Reset");
    delay(2000); // Display reset message for 2 seconds
  }
  
  // ----- Sensor Reading -----
  float temperatureC = readTemperatureC();
  float humidity     = readHumidity();

  // ----- Store New Sample into EEPROM Ring Buffer -----
  int16_t tempTenths = (int16_t)(temperatureC * 10.0 + 0.5);
  int16_t humTenths  = (int16_t)(humidity     * 10.0 + 0.5);
  storeSampleInEeprom(tempTenths, humTenths);

  // ----- Reload all samples from EEPROM into RAM -----
  loadAllSamplesFromEeprom();

  // ----- Compute Statistics -----
  float avgTempC, varTempC;
  computeStats(temperatureData, totalSamples, avgTempC, varTempC);
  float avgHum, varHum;
  computeStats(humidityData, totalSamples, avgHum, varHum);

  // ----- Check Toggle Switch for °C/°F -----
  bool isFahrenheit = (digitalRead(switchPin) == LOW);
  float displayTemp    = temperatureC;
  float displayAvgTemp = avgTempC;
  float displayVarT    = varTempC; // variance in Celsius by default
  
  if (isFahrenheit) {
    displayTemp    = (temperatureC * 9.0 / 5.0) + 32.0;
    displayAvgTemp = (avgTempC     * 9.0 / 5.0) + 32.0;
    // Variance converts by (9/5) squared
    displayVarT    = varTempC * (9.0/5.0) * (9.0/5.0);
  }

  // ----- LED Logic in °C -----
  if (temperatureC > HOT_THRESHOLD) {
    digitalWrite(tempRedLedPin, HIGH);
    digitalWrite(tempBlueLedPin, LOW);
  } else if (temperatureC < COLD_THRESHOLD) {
    digitalWrite(tempRedLedPin, LOW);
    digitalWrite(tempBlueLedPin, HIGH);
  } else {
    digitalWrite(tempRedLedPin, LOW);
    digitalWrite(tempBlueLedPin, LOW);
  }

  if (humidity < DRY_THRESHOLD) {
    digitalWrite(humRedLedPin, LOW);
    digitalWrite(humBlueLedPin, HIGH);
  } else if (humidity > HUMID_THRESHOLD) {
    digitalWrite(humRedLedPin, HIGH);
    digitalWrite(humBlueLedPin, LOW);
  } else {
    digitalWrite(humRedLedPin, LOW);
    digitalWrite(humBlueLedPin, LOW);
  }

  // ----- Update LCD Display -----
  lcd.clear();
  
  // Line 0: Title
  lcd.setCursor(0, 0);
  lcd.print("Monitoring");

  // Line 1: Current Temperature & Humidity
  lcd.setCursor(0, 1);
  lcd.print("T=");
  lcd.print(displayTemp, 1);
  lcd.print(isFahrenheit ? "F" : "C");
  lcd.print(" H=");
  lcd.print(humidity, 0);
  lcd.print("%");

  // Line 2: Temperature Average & Variance
  lcd.setCursor(0, 2);
  lcd.print("T Avg=");
  lcd.print(displayAvgTemp, 1);
  lcd.print(" Var=");
  lcd.print(displayVarT, 0);

  // Line 3: Humidity Average & Variance
  lcd.setCursor(0, 3);
  lcd.print("H Avg=");
  lcd.print(avgHum, 0);
  lcd.print("% Var=");
  lcd.print(varHum, 0);

  // ----- Debug Print to Serial -----
  Serial.print("Now stored = "); Serial.print(eepromCount);
  Serial.print(" samples; Head="); Serial.print(eepromHead);
  if (isFahrenheit) {
    Serial.print("; T(F)=");  Serial.print(displayTemp);
    Serial.print("; TavgF="); Serial.print(displayAvgTemp);
    Serial.print("; VarT(F)="); Serial.print(displayVarT);
  } else {
    Serial.print("; T(C)=");  Serial.print(temperatureC);
    Serial.print("; TavgC="); Serial.print(avgTempC);
    Serial.print("; VarT(C)="); Serial.print(varTempC);
  }
  Serial.print("; H(%)=");  Serial.print(humidity);
  Serial.print("; Havg=");   Serial.print(avgHum);
  Serial.print("; VarH=");   Serial.println(varHum);

  // ----- Wait for Next Reading -----
  delay(3000);
}