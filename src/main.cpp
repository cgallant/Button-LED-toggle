// Include required libraries for Arduino, I2C communication, and peripheral drivers
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_TLC5947.h>

// I2C pin configuration for ESP32
const int SDA_PIN = 8;
const int SCL_PIN = 9;

// TLC5947 LED driver control pins
const int TLC_LAT = 15;  // Latch pin - loads data into output registers
const int TLC_OE = 16;   // Output Enable pin - enables/disables LED outputs
const int TLC_CLK = 18;  // Clock pin - synchronizes data transfer
const int TLC_DIN = 39;  // Data input pin - receives PWM data

// System configuration - 8 MCP23017 chips, each with 16 buttons/LEDs
const int NUM_PAIRS = 8;
const uint8_t MCP_ADDRESSES[NUM_PAIRS] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27}; // I2C addresses for each MCP23017
const int TLC_BASE_CHANNEL[NUM_PAIRS] = {0, 24, 48, 72, 96, 120, 144, 168};                // Starting TLC channel for each MCP (24 channels per chip)

// Initialize hardware objects
Adafruit_MCP23X17 mcp[NUM_PAIRS];                    // Array of MCP23017 I/O expanders (button inputs)
Adafruit_TLC5947 tlc(8, TLC_CLK, TLC_DIN, TLC_LAT);  // TLC5947 LED driver (8 chips daisy-chained = 192 channels)

// State tracking arrays
bool ledState[NUM_PAIRS][16] = {false};         // Current on/off state for each LED
bool lastButtonState[NUM_PAIRS][16] = {false};  // Previous button state for edge detection
bool mcpFound[NUM_PAIRS] = {false};             // Tracks which MCP chips were detected

// Maps MCP23017 pin numbers to TLC5947 channel numbers
// Each MCP has 16 pins (GPA0-7 and GPB0-7), each pair gets 24 TLC channels
int pinToChannel(int pair, int pin) {
  int base = TLC_BASE_CHANNEL[pair];  // Get the starting channel for this MCP pair
  if (pin >= 8) {
    // GPB pins (8-15) map to TLC channels 0-7 relative to base
    return base + (pin - 8);
  } else {
    // GPA pins (0-7) map to TLC channels 8-15 relative to base
    return base + 8 + pin;
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial0.begin(115200);
  delay(5000);  // Wait for serial port to stabilize
  while (!Serial0) {
    delay(10);  // Wait for serial connection
  }
  delay(5000);
  Serial0.println("=== 8-Pair Button-LED Toggle ===");

  // Initialize I2C bus with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize each MCP23017 I/O expander
  for (int i = 0; i < NUM_PAIRS; i++) {
    mcpFound[i] = mcp[i].begin_I2C(MCP_ADDRESSES[i], &Wire);  // Attempt to initialize at specified I2C address
    if (mcpFound[i]) {
      // MCP found - configure all 16 pins as inputs with internal pull-up resistors
      for (int pin = 0; pin < 16; pin++) {
        mcp[i].pinMode(pin, INPUT_PULLUP);  // Pull-up so buttons read LOW when pressed
      }
      Serial0.print("MCP at 0x");
      Serial0.print(MCP_ADDRESSES[i], HEX);
      Serial0.println(" ready");
    } else {
      // MCP not detected on I2C bus
      Serial0.print("MCP at 0x");
      Serial0.print(MCP_ADDRESSES[i], HEX);
      Serial0.println(" NOT FOUND");
    }
  }

  // Initialize TLC5947 LED driver
  pinMode(TLC_OE, OUTPUT);
  digitalWrite(TLC_OE, LOW);  // Enable outputs (active LOW)
  tlc.begin();
  // Turn off all 192 LED channels (8 TLC5947 chips × 24 channels each)
  for (int i = 0; i < 192; i++) {
    tlc.setPWM(i, 0);  // Set PWM to 0 (off)
  }
  tlc.write();  // Write data to TLC chips
  Serial0.println("TLC5947 ready - LEDs off");

  // Perform startup flash sequence to verify all LEDs are working
  Serial0.println("Startup flash...");
  for (int flash = 0; flash < 6; flash++) {
    Serial0.print("Flash ");
    Serial0.print(flash + 1);
    Serial0.println(" ON");
    // All LEDs ON at full brightness
    for (int i = 0; i < 192; i++) {
      tlc.setPWM(i, 4095);  // 4095 = max brightness (12-bit PWM)
    }
    tlc.write();
    delay(500);  // Increased delay for long chain

    Serial0.print("Flash ");
    Serial0.print(flash + 1);
    Serial0.println(" OFF");
    // All LEDs OFF
    for (int i = 0; i < 192; i++) {
      tlc.setPWM(i, 0);
    }
    tlc.write();
    delay(500);  // Increased delay for long chain
  }
  Serial0.println("Ready!");
  Serial0.println("Press buttons to toggle LEDs!");
}

void loop() {
  // Debug: Type a channel number in serial monitor to test that LED
  if (Serial0.available()) {
    int testChannel = Serial0.parseInt();
    if (testChannel >= 0 && testChannel < 192) {
      Serial0.print("Testing channel ");
      Serial0.println(testChannel);
      tlc.setPWM(testChannel, 4095);  // Full brightness
      tlc.write();
      delay(2000);  // Keep on for 2 seconds
      tlc.setPWM(testChannel, 0);  // Turn off
      tlc.write();
    }
  }

  // Scan all MCP23017 chips for button presses
  for (int pair = 0; pair < NUM_PAIRS; pair++) {
    if (!mcpFound[pair]) continue;  // Skip this MCP if it wasn't detected

    // Check all 16 pins on this MCP
    for (int pin = 0; pin < 16; pin++) {
      bool pressed = !mcp[pair].digitalRead(pin);  // Invert because INPUT_PULLUP reads LOW when pressed

      // Detect rising edge (button just pressed, not held)
      if (pressed && !lastButtonState[pair][pin]) {
        // Toggle the LED state
        ledState[pair][pin] = !ledState[pair][pin];

        // Update the corresponding TLC channel
        int channel = pinToChannel(pair, pin);
        tlc.setPWM(channel, ledState[pair][pin] ? 4095 : 0);  // Full brightness or off
        tlc.write();  // Send update to TLC chips

        // Print debug information
        Serial0.print("MCP 0x");
        Serial0.print(MCP_ADDRESSES[pair], HEX);
        Serial0.print(" Pin ");
        Serial0.print(pin);
        Serial0.print(" -> Channel ");
        Serial0.print(channel);
        Serial0.print(" -> ");
        Serial0.println(ledState[pair][pin] ? "ON" : "OFF");
      }

      // Update button state for next iteration
      lastButtonState[pair][pin] = pressed;
    }
  }

  // Small delay to debounce buttons and reduce CPU usage
  delay(50);
}