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
const int displayInterval = 30; // ms (~33 FPS)

// ---------------- Encoder ----------------
#define inputCLK 2  // Best on Interrupt pin
#define inputDT 3   // Best on Interrupt pin
#define inputSW A3

// Initialize the Encoder
Encoder myEnc(inputCLK, inputDT);

// Tracking raw positions instead of forced grids
long lastRawPosition = 0;
unsigned long lastEncoderMoveTime = 0;
const unsigned long encoderDebounceDelay = 10; // ms to ignore contact bounce noise

// ---------------- PWM ----------------
#define FAN_PWM 9

int pwmPercent = 75;
int stepSize = 5;

// ---------------- Display cache ----------------
int oldPWM = -1;
int oldStep = -1;

// ---------------- Encoder state ----------------
int Length = 75; // maps to PWM (0–100)
int oldLength = -1;

void setup() {
  #if DEBUG
  Serial.begin(9600);
  DEBUG_PRINTLN("System starting...");
  #endif

  Wire.setClock(400000); // 400kHz fast mode

  // Pullup on Encoder pins
  pinMode(inputCLK, INPUT_PULLUP);
  pinMode(inputDT, INPUT_PULLUP);
 
  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
#if DEBUG
    DEBUG_PRINTLN("SSD1306 failed");
#endif
    for (;;);
  }

  display.clearDisplay();
  display.display();

  // Encoder Switch
  pinMode(inputSW, INPUT_PULLUP);

  // PWM
  pinMode(FAN_PWM, OUTPUT);
  analogWrite(FAN_PWM, map(pwmPercent, 0, 100, 0, 255));

  // Initialize Encoder position internal state
  myEnc.write(0);
  lastRawPosition = 0;

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
  long currentRawPosition = myEnc.read();
  long change = currentRawPosition - lastRawPosition;

  // Change threshold: Cheap encoders change by 2 units per physical click. 
  // If yours is still slightly sluggish, you can change '2' to '1' below.
  if (abs(change) >= 2) {
    unsigned long now = millis();

    if (now - lastEncoderMoveTime > encoderDebounceDelay) {
      
      if (change > 0) {
        Length += stepSize;
        DEBUG_PRINTLN("CW rotation");
      } else {
        Length -= stepSize;
        DEBUG_PRINTLN("CCW rotation");
      }

      // Keep within bounds
      Length = constrain(Length, 0, 100);
      
      lastEncoderMoveTime = now;
      lastRawPosition = currentRawPosition; // Update baseline tracking
    } else {
      // If it was noise within the debounce window, reset encoder to suppress it
      myEnc.write(lastRawPosition);
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
  if (Length != pwmPercent) {
    pwmPercent = Length;
    analogWrite(FAN_PWM, map(pwmPercent, 0, 100, 0, 255));
  }
}

// ---------------- OLED ----------------
void printToDisplay() {
  if (Length != oldLength || stepSize != oldStep) {

    oldLength = Length;
    oldStep = stepSize;

    // Only clear small regions (not full screen)
    display.fillRect(0, 14, 128, 18, SSD1306_BLACK);  // value area
    display.fillRect(0, 24, 128, 8, SSD1306_BLACK);   // step area

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
