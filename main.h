const uint LED_PIN = PICO_DEFAULT_LED_PIN;

const uint EncoderPin[] = {3, 4};
void gpio_callback(uint gpio, uint32_t events);
Encoder encoder(EncoderPin, gpio_callback);

void gpio_callback(uint gpio, uint32_t events) { encoder.update(gpio); }

void loop1();

void setup() {
  initializeADC();
  temp.Begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(16, OUTPUT);
  midi.Begin(uart0, 16, 17);
  core.launch_task(loop1);
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

void enc_change(void) { usb_serial.println(encoder.getValue()); }

void loop() {
  // midi.sendControlChange(0, 100);
  encoder.on_change(&enc_change);

  writeFrequency(21, math.abs(encoder.getValue()));
  usb_serial.println(sys.get_sdk_version());
}
