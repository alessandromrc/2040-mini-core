#include "core.hpp"

bool led_status = false;

unsigned long previousMillis = 0;
unsigned long interval = 50;

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

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

    writeFrequency(21, math.abs(encoder.getValue()));
}

int main()
{
    stdio_init_all();
    setup();
    while (true) {
        try {
        if (Reset.press()) {
            sys.reset();
        }
        loop();
        } catch(exception_handler_t handler)
        {
            usb_serial.println(exception.get_current_exception());
        }
        tight_loop_contents();
    }
    return 0;
}
