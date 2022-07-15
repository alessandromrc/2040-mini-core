#include <stdio.h>
#include "hardware/resets.h"
#include "pico/stdlib.h"

#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))

#define HIGH true
#define LOW false

#define INPUT 1
#define OUTPUT 2
#define INPUT_PULLUP 3
#define INPUT_PULLDOWN 4

unsigned long millis() { return (unsigned long)time_us_32() / 1000; }

unsigned long micros() { return (unsigned long)time_us_32(); }

void pinMode(int pin, int mode) {
  gpio_init(pin);
  if (mode == INPUT)
    gpio_set_dir(pin, GPIO_IN);
  else if (mode == OUTPUT)
    gpio_set_dir(pin, GPIO_OUT);
  else if (mode == INPUT_PULLUP) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_pulls(pin, true, false);
  } else if (mode == INPUT_PULLDOWN) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_pulls(pin, false, true);
  }
}

void digitalWrite(int pin, int value) {
  if (value == HIGH)
    gpio_put(pin, HIGH);
  else if (value == LOW) 
    gpio_put(pin, LOW);
}

bool digitalRead(int pin) { return gpio_get(pin); }

void delay(int time) { sleep_ms(time); }

void delayMicroseconds(int time) { sleep_us(time); }

void reset() { AIRCR_Register = 0x5FA0004; }

void pinFunction(int pin, gpio_function function) {
  gpio_set_function(pin, function);
}

