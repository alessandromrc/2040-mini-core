#include "debouncing.hpp"

Midi midi;
System system;
USB_SERIAL usb_serial;
Temperature temp;
Core core;
Interrupt interrupt;

bool led_status = false;

unsigned long previousMillis = 0;
unsigned long interval = 50;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint switch_reset = 2;

Debouncing Reset(switch_reset, INPUT_PULLUP);


void loop1() {
    while(true)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
        //usb_serial.println(system.arithmetic_shift_right(100, 5));
    }
}


const uint EncoderPin[] = {3, 4};


void gpio_callback(uint gpio, uint32_t events);


Encoder encoder(EncoderPin, gpio_callback);


void gpio_callback(uint gpio, uint32_t events)
{
    encoder.update(gpio);
}

void setup()
{
    initializeADC();
    temp.Begin();
    pinMode(LED_PIN, OUTPUT);
    pinMode(16, OUTPUT);
    pinMode(26, ANALOG);
    midi.Begin(uart0, 0, 1);
    core.launch_task(loop1);
}






void loop()
{
    /*
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        led_status = !led_status;
        digitalWrite(LED_PIN, led_status);
        previousMillis = currentMillis;
    }
    */
    //usb_serial.println(clock_get_hz(clk_sys));

    writeFrequency(21, system.abs(encoder.getValue()));

    usb_serial.println(system.abs(encoder.getValue()));
}

int main()
{
    stdio_init_all();
    setup();
    while (true) {
        if (Reset.press()) {
            system.reset();
        }
        loop();
        tight_loop_contents();
    }
    return 0;
}
