#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Encoder.h> // Include Paul Stoffregen's Encoder Library

// ---------------- DEBUG MODE ----------------
#define DEBUG 0   // set to 0 to disable all Serial prints

#if DEBUG
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT(x)   Serial.print(x)
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
#endif

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const uint8_t displayInterval = 30; // ms (~33 FPS)

// ---------------- Encoder ----------------
#define inputCLK 2  // Interrupt pin
#define inputDT 3   // Interrupt pin
#define inputSW A3

// Initialize the Encoder
Encoder myEnc(inputCLK, inputDT);

// Base state tracking matching the basic example design
long oldPosition = 0;

// ---------------- PWM & State ----------------
#define FAN_PWM 9

int8_t pwmPercent = 75;
int8_t stepSize = 5;

// ---------------- Display cache ----------------
int8_t oldPWM = -1;
int8_t oldStep = -1;

void setup() {
  #if DEBUG
  Serial.begin(9600);
  DEBUG_PRINTLN("System starting...");
  #endif

  Wire.setClock(400000); // 400kHz fast mode

  // Pullup on Encoder pins
  pinMode(inputCLK, INPUT_PULLUP);
  pinMode(inputDT, INPUT_PULLUP);
  pinMode(inputSW, INPUT_PULLUP);
 
  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
#if DEBUG
    DEBUG_PRINTLN("SSD1306 failed");
#endif
    for (;;);
  }

  display.clearDisplay();
  display.display();

  // PWM Setup
  pinMode(FAN_PWM, OUTPUT);
  analogWrite(FAN_PWM, map(pwmPercent, 0, 100, 0, 255));

  // Sync baseline to 0
  myEnc.write(0);

  // Draw static header once
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Fan Control");
  display.display();
}

unsigned long lastDisplayUpdate = 0;

void loop() {
  readEncoder();
  handleButton();
  updateFan();
  
  if (millis() - lastDisplayUpdate > displayInterval) {
    printToDisplay();
    lastDisplayUpdate = millis();
  }
}

// ---------------- Encoder ----------------
void readEncoder() {
  long newPosition = myEnc.read();
  
  if (newPosition != oldPosition) {
    // Calculate how many raw counts the library has moved
    long clicks = (newPosition - oldPosition);
    
    // NOTE: Change the '2' below to '4' if your basic test code 
    // shows increments of 4 units per physical click on the Serial monitor.
    if (abs(clicks) >= 2) { 
      
      if (clicks > 0) {
        pwmPercent += stepSize;
        DEBUG_PRINTLN("CW rotation");
      } else {
        pwmPercent -= stepSize;
        DEBUG_PRINTLN("CCW rotation");
      }

      // Keep within hardware boundaries
      pwmPercent = constrain(pwmPercent, 0, 100);
      
      // Update our baseline position to the current state cleanly
      oldPosition = newPosition; 
    }
  }
}

// ---------------- Button ----------------
void handleButton() {
  static unsigned long lastPress = 0;

  if (digitalRead(inputSW) == LOW) {
    if (millis() - lastPress > 300) {
      stepSize = (stepSize == 1) ? 5 : 1;

      DEBUG_PRINT("Step size changed to: ");
      DEBUG_PRINTLN(stepSize);

      lastPress = millis();
    }
  }
}

// ---------------- PWM update ----------------
void updateFan() {
  static int8_t lastAppliedPWM = -1; 
  
  if (pwmPercent != lastAppliedPWM) {
    lastAppliedPWM = pwmPercent;
    analogWrite(FAN_PWM, map(pwmPercent, 0, 100, 0, 255));
  }
}

// ---------------- OLED ----------------
void printToDisplay() {
  if (pwmPercent != oldPWM || stepSize != oldStep) {

    oldPWM = pwmPercent;
    oldStep = stepSize;

    display.fillRect(75, 14, 53, 16, SSD1306_BLACK);  // Value area
    display.fillRect(35, 24, 40, 8, SSD1306_BLACK);   // Step area

    display.setTextColor(SSD1306_WHITE);

    // PWM big value
    display.setTextSize(2);
    display.setCursor(80, 14);
    display.print(pwmPercent);
    display.print("%");

    // Step line
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print("Step: ");
    display.print(stepSize);

    display.display();
  }
}
