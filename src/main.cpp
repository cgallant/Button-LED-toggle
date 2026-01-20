#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_TLC5947.h>

const int SDA_PIN = 8;
const int SCL_PIN = 9;
const int TLC_LAT = 15;
const int TLC_OE = 16;
const int TLC_CLK = 18;
const int TLC_DIN = 39;

const int NUM_PAIRS = 4;
const uint8_t MCP_ADDRESSES[NUM_PAIRS] = {0x20, 0x21, 0x22, 0x23};
const int TLC_BASE_CHANNEL[NUM_PAIRS] = {0, 24, 48, 72};

Adafruit_MCP23X17 mcp[NUM_PAIRS];
Adafruit_TLC5947 tlc(4, TLC_CLK, TLC_DIN, TLC_LAT);

bool ledState[NUM_PAIRS][16] = {false};
bool lastButtonState[NUM_PAIRS][16] = {false};
bool mcpFound[NUM_PAIRS] = {false};

int pinToChannel(int pair, int pin) {
  int base = TLC_BASE_CHANNEL[pair];
  if (pin >= 8) {
    // GPB pins (8-15) map to TLC channels 0-7
    return base + (pin - 8);
  } else {
    // GPA pins (0-7) map to TLC channels 8-15
    return base + 8 + pin;
  }
}

void setup() {
  Serial0.begin(115200);
  delay(5000);
  while (!Serial0) {
    delay(10);
  }
  delay(2000);
  Serial0.println("=== 4-Pair Button-LED Toggle ===");

  Wire.begin(SDA_PIN, SCL_PIN);

  // Init each MCP23017
  for (int i = 0; i < NUM_PAIRS; i++) {
    mcpFound[i] = mcp[i].begin_I2C(MCP_ADDRESSES[i], &Wire);
    if (mcpFound[i]) {
      for (int pin = 0; pin < 16; pin++) {
        mcp[i].pinMode(pin, INPUT_PULLUP);
      }
      Serial0.print("MCP at 0x");
      Serial0.print(MCP_ADDRESSES[i], HEX);
      Serial0.println(" ready");
    } else {
      Serial0.print("MCP at 0x");
      Serial0.print(MCP_ADDRESSES[i], HEX);
      Serial0.println(" NOT FOUND");
    }
  }

  // Init TLC5947
  pinMode(TLC_OE, OUTPUT);
  digitalWrite(TLC_OE, LOW);
  tlc.begin();
  for (int i = 0; i < 96; i++) {
    tlc.setPWM(i, 0);
  }
  tlc.write();
  Serial0.println("TLC5947 ready - LEDs off");
Serial0.println("Startup flash...");
  for (int flash = 0; flash < 6; flash++) {
    // All LEDs ON
    for (int i = 0; i < 96; i++) {
      tlc.setPWM(i, 4095);
    }
    tlc.write();
    delay(250);
    
    // All LEDs OFF
    for (int i = 0; i < 96; i++) {
      tlc.setPWM(i, 0);
    }
    tlc.write();
    delay(250);
  }
  Serial0.println("Ready!");
  Serial0.println("Press buttons to toggle LEDs!");
}

void loop() {
  for (int pair = 0; pair < NUM_PAIRS; pair++) {
    if (!mcpFound[pair]) continue;

    for (int pin = 0; pin < 16; pin++) {
      bool pressed = !mcp[pair].digitalRead(pin);

      if (pressed && !lastButtonState[pair][pin]) {
        ledState[pair][pin] = !ledState[pair][pin];

        int channel = pinToChannel(pair, pin);
        tlc.setPWM(channel, ledState[pair][pin] ? 4095 : 0);
        tlc.write();

        Serial0.print("MCP 0x");
        Serial0.print(MCP_ADDRESSES[pair], HEX);
        Serial0.print(" Pin ");
        Serial0.print(pin);
        Serial0.print(" -> Channel ");
        Serial0.print(channel);
        Serial0.print(" -> ");
        Serial0.println(ledState[pair][pin] ? "ON" : "OFF");
      }

      lastButtonState[pair][pin] = pressed;
    }
  }

  delay(50);
}