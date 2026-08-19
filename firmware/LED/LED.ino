// 8x8 WS2812B status indicator - Arduino Nano
//
// Hardware:
//   Matrix DIN -> D9 (through 330-470 ohm, placed at the matrix end)
//   Matrix VCC -> 5 V rail        Matrix GND -> supply GND *and* Nano GND
//   1000 uF across VCC/GND at the panel.
//   D7 -> override input. HIGH = forced RED, outranks everything.
//
// Serial @115200, one ASCII char per command:
//   '0' off   '1' RED   '2' YELLOW   '3' GREEN
// Before the first command arrives, the panel flashes RED at a 2 s interval.

#include <Adafruit_NeoPixel.h>

#define LED_PIN            6
#define NUM_PIXELS         64
#define PIXEL_TYPE         (NEO_GRB + NEO_KHZ800)   // 3-pin panel = 3-byte GRB
#define BRIGHTNESS         255      // see power note at the bottom before raising this
#define OVERRIDE_PIN       7
#define FLASH_INTERVAL_MS  2000    // 2 s on, 2 s off

// 0 = literal spec: flashing stops for good at the first command.
// Set to e.g. 5000 to resume flashing after 5 s of serial silence.
#define COMMS_TIMEOUT_MS   0

Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, PIXEL_TYPE);

char state = '1';
bool haveCmd = false;                 // no command received yet -> flashing
unsigned long lastCmdMs   = 0;
unsigned long lastFlashMs = 0;
bool flashOn = false;

uint32_t shownColor = 0xFFFFFFFFUL;   // sentinel: nothing pushed yet
unsigned long lastShowMs = 0;
#define REFRESH_INTERVAL_MS 1000      // keepalive so a glitched pixel self-heals

// show() disables interrupts for ~2.4 ms at 64 pixels, and loop() calls this every pass.
// Skipping the wire when nothing changed keeps the UART from dropping command bytes.
void fill(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t c = pixels.Color(r, g, b);
  unsigned long now = millis();
  if (c == shownColor && (now - lastShowMs) < REFRESH_INTERVAL_MS) return;

  for (int i = 0; i < NUM_PIXELS; i++) pixels.setPixelColor(i, c);
  pixels.show();
  shownColor = c;
  lastShowMs = now;
}

void setup() {
  Serial.begin(115200);
  pinMode(OVERRIDE_PIN, INPUT);   // external divider provides the pull-down
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  fill(255, 0, 0);                // RED at power-on
  lastFlashMs = millis();
  flashOn = true;
}

void loop() {
  // Read serial first. Reading does not touch the panel, so the override still wins.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '0' || c == '1' || c == '2' || c == '3') {
      state = c;
      haveCmd = true;
      lastCmdMs = millis();
      Serial.println(state);      // echo so the host can confirm the link
    }
  }

  // ---------- D7 override ----------
  if (digitalRead(OVERRIDE_PIN) == HIGH) {
    fill(255, 0, 0);
    return;
  }

  // ---------- waiting for a command ----------
  bool waiting = !haveCmd;
#if COMMS_TIMEOUT_MS > 0
  if (haveCmd && (millis() - lastCmdMs > COMMS_TIMEOUT_MS)) waiting = true;
#endif

  if (waiting) {
    if (millis() - lastFlashMs >= FLASH_INTERVAL_MS) {
      lastFlashMs = millis();
      flashOn = !flashOn;
    }
    if (flashOn) fill(255, 0, 0); else fill(0, 0, 0);
    return;
  }

  // ---------- commanded state ----------
  switch (state) {
    case '0': fill(0,   0,   0); break;   // off
    case '1': fill(255, 0,   0); break;   // red
    case '2': fill(255, 170, 0); break;   // yellow
    case '3': fill(0,   255, 0); break;   // green
  }
}

/*
  POWER
  64 WS2812B at BRIGHTNESS 60: roughly 300 mA solid red, 800 mA solid white.
  At BRIGHTNESS 255 it is ~1.3 A red and ~3.5 A white. Give the panel its own 5 V
  supply rather than the Nano's 5 V pin, and tie the grounds together.

  D7 AS WIRED
  HIGH = killed, and the wire is what holds it HIGH, so a cut wire or dead relay reads
  LOW and the panel keeps showing the last commanded colour. If you want open-circuit
  faults to land on RED instead: pinMode(OVERRIDE_PIN, INPUT_PULLUP) and wire the relay
  to pull D7 to GND while thruster power is live. The HIGH == killed test stays as is.

  ORIENTATION
  Nothing here is position-dependent - every pixel gets the same colour, so panel
  layout and zigzag wiring do not matter.
*/
