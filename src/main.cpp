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

Adafruit_MCP23X17 mcp;
Adafruit_TLC5947 tlc(4, TLC_CLK, TLC_DIN, TLC_LAT);

bool ledState[16] = {false};
bool lastButtonState[16] = {false};

void setup()
{
  Serial0.begin(115200);
  delay(2000);
  Serial0.println("Button-LED Toggle Test");

  Wire.begin(SDA_PIN, SCL_PIN);
  mcp.begin_I2C(0x21, &Wire);
  for (int pin = 0; pin < 16; pin++)
  {
    mcp.pinMode(pin, INPUT_PULLUP);
  }
  Serial0.println("MCP23017 ready");

  pinMode(TLC_OE, OUTPUT);
  digitalWrite(TLC_OE, LOW);
  tlc.begin();
  for (int i = 0; i < 96; i++)
  {
    tlc.setPWM(i, 0);
  }
  tlc.write();
  Serial0.println("TLC5947 ready - LEDs off");

  Serial0.println("Press buttons to toggle LEDs!");
}

void loop()
{
  for (int pin = 0; pin < 16; pin++)
  {
    bool pressed = !mcp.digitalRead(pin);

    if (pressed && !lastButtonState[pin])
    {
      ledState[pin] = !ledState[pin];

      int channel;
      if (pin >= 8)
      {
        // GPB pins (8-15) map to TLC channels 24-31
        channel = 24 + (pin - 8);
      }
      else
      {
        // GPA pins (0-7) map to TLC channels 32-39
        channel = 32 + pin;
      }
      tlc.setPWM(channel, ledState[pin] ? 4095 : 0);
      tlc.write();

      Serial0.print("Button pin ");
      Serial0.print(pin);
      Serial0.print(" -> LED channel ");
      Serial0.print(channel);
      Serial0.print(" -> ");
      Serial0.println(ledState[pin] ? "ON" : "OFF");
    }

    lastButtonState[pin] = pressed;
  }

  delay(50);
}