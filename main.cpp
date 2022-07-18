#include "debouncing.hpp"

Midi midi;
System system;
USB_SERIAL usb_serial;
Temperature temp;


bool led_status = false;

unsigned long previousMillis = 0;
unsigned long interval = 50;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint switch_reset = 2;


Debouncing Reset(switch_reset, INPUT_PULLUP);


void setup() {

  initializeADC();
  temp.Begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(26, ANALOG);


  midi.Begin(uart0, 0, 1);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    led_status = !led_status;
    digitalWrite(LED_PIN, led_status);
    previousMillis = currentMillis;
  }

  midi.sendNoteOn(60, 127);
  delay(10);
  midi.sendNoteOff(60, 127);
  delay(10);

  usb_serial.println(analogRead(26));

  usb_serial.print("temperature -> ");

  usb_serial.print(temp.read('C'));
  usb_serial.println("C");

}

int main() {
  stdio_init_all();
  setup();
  while (true) {
    if (Reset.press()) {
      system.reset();
    }
    loop();
  }
  return 0;
}
