#include "debouncing.hpp"

bool led_status = false;

unsigned long previousMillis = 0;
unsigned long interval = 50;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint switch1_pin = 0;
const uint switch_reset = 1;

Debouncing PSW_1(switch1_pin, INPUT_PULLUP);
Debouncing Reset(switch_reset, INPUT_PULLUP);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(16, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    led_status = !led_status;
    printf("LED status: %d\n", led_status);
    digitalWrite(LED_PIN, led_status);
    previousMillis = currentMillis;
  }

  if (PSW_1.press()) {
    digitalWrite(16, led_status);
  } else {
    digitalWrite(16, led_status);
  }

}

int main() {
  stdio_init_all();
  setup();
  while (true) {
    if (Reset.press()) {
      reset();
    }
    loop();
  }
  return 0;
}
