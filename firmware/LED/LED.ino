// Graey status LED strip - RobotX 2026
// 8x NeoPixel GRBW on D6 (same hardware the old modemLights sketch drove).
//
// Serial protocol @115200, one ASCII char per command:
//   '0' off   '1' RED (emergency motor off)   '2' YELLOW (manual)   '3' GREEN (autonomous)
// If no command arrives for COMMS_TIMEOUT_MS the strip flashes red/off, so a dead
// Jetson can never leave a stale GREEN showing on the prequal video.

#include <Adafruit_NeoPixel.h>

#define LED_PIN         6
#define NUM_PIXELS      8
#define BRIGHTNESS      150
#define COMMS_TIMEOUT_MS 3000

Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRBW + NEO_KHZ800);

char state = '1';                 // start RED - safest default
unsigned long lastCmdMs = 0;
bool flashOn = false;
unsigned long lastFlashMs = 0;

void fill(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  for (int i = 0; i < NUM_PIXELS; i++) pixels.setPixelColor(i, r, g, b, w);
  pixels.show();
}

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  fill(255, 0, 0, 0);             // RED until told otherwise
  lastCmdMs = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '0' || c == '1' || c == '2' || c == '3') {
      state = c;
      lastCmdMs = millis();
      Serial.println(state);      // echo so the host can confirm the link
    }
  }

  if (millis() - lastCmdMs > COMMS_TIMEOUT_MS) {
    if (millis() - lastFlashMs > 300) {
      lastFlashMs = millis();
      flashOn = !flashOn;
      if (flashOn) fill(255, 0, 0, 0); else fill(0, 0, 0, 0);
    }
    return;
  }

  switch (state) {
    case '0': fill(0,   0,   0, 0); break;
    case '1': fill(255, 0,   0, 0); break;   // red
    case '2': fill(255, 170, 0, 0); break;   // yellow
    case '3': fill(0,   255, 0, 0); break;   // green
  }
}
