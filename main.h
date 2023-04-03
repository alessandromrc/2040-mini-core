const uint LED_PIN = PICO_DEFAULT_LED_PIN;

void setup() {
  initializeADC();
  temp.Begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(16, OUTPUT);
  midi.Begin(uart0, 16, 17);
  core.launch_task(loop1);
  rtc.Begin();
  rtc.setDateTime(2022, 11, 29, 5, 12, 1, 00);
  serial.Begin(uart0, 115200, 1, 0);
}

void loop1() {
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    // usb_serial.println(system.arithmetic_shift_right(100, 5));
  }
}

Note note = Notes(Notes::List::C, 8);

void loop() {

}
