#include <stdio.h>
#include "hardware/resets.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#define AIRCR_Register (*((volatile uint32_t *)(PPB_BASE + 0x0ED0C)))
#define HIGH true
#define LOW false
#define INPUT 1
#define OUTPUT 2
#define INPUT_PULLUP 3
#define INPUT_PULLDOWN 4
#define ANALOG 5

double map(double input, double input_start, double input_end, double output_start, double output_end)
{
	return (input - input_start) *(output_end - output_start) / (input_end - input_start) + output_start;
}

unsigned long millis()
{
	return (unsigned long) time_us_32() / 1000;
}

unsigned long micros()
{
	return (unsigned long) time_us_32();
}

void delay(int time)
{
	sleep_ms(time);
}

void delayMicroseconds(int time)
{
	sleep_us(time);
}

void pinMode(int pin, int mode)
{
	gpio_init(pin);
	if (mode == INPUT)
		gpio_set_dir(pin, GPIO_IN);
	else if (mode == OUTPUT)
		gpio_set_dir(pin, GPIO_OUT);
	else if (mode == INPUT_PULLUP)
	{
		gpio_set_dir(pin, GPIO_IN);
		gpio_set_pulls(pin, true, false);
	}
	else if (mode == INPUT_PULLDOWN)
	{
		gpio_set_dir(pin, GPIO_IN);
		gpio_set_pulls(pin, false, true);
	}
	else if (mode == ANALOG)
	{
		adc_gpio_init(pin);
		adc_select_input(0);
	}
}

void initializeADC()
{
	adc_init();
}

int analogReadRaw(int pin)
{
	if (pin == 26)
	{
		adc_select_input(0);
	}
	else if (pin == 27)
	{
		adc_select_input(1);
	}
	else if (pin == 28)
	{
		adc_select_input(2);
	}

	return adc_read();
}

float analogReadVoltage(int pin)
{
	return analogReadRaw(pin) *(3.3f / (1 << 12));
}

int analogRead(int pin)
{
	return (int)(map(analogReadRaw(pin), 0, 4095, 0, 1023));
}

void digitalWrite(int pin, int value)
{
	if (value == HIGH)
		gpio_put(pin, HIGH);
	else if (value == LOW)
		gpio_put(pin, LOW);
}

bool digitalRead(int pin)
{
	return gpio_get(pin);
}

void pinFunction(int pin, gpio_function
	function)
{
	gpio_set_function(pin, function);
}

class System
{
	public:
		void reset()
		{
			AIRCR_Register = 0x5FA0004;
		}
};

class Serial
{
	private:
		uart_inst *UART_ID = 0;
	public:
		void Begin(uart_inst *uartID, int Baudrate, int txPin, int rxPin)
		{
			uart_init(uartID, Baudrate);
			UART_ID = uartID;
			pinFunction(txPin, GPIO_FUNC_UART);
			pinFunction(rxPin, GPIO_FUNC_UART);
		}

	void writeRaw(char character)
	{
		uart_putc_raw(UART_ID, character);
	}

	void putc(char character)
	{
		uart_putc(UART_ID, character);
	}

	void writeString(const char *string)
	{
		uart_puts(UART_ID, string);
	}
};

class Midi
{
	private:
		int baudrate = 31250;
	Serial serial;
	enum MidiType: uint8_t
	{
		InvalidType = 0x00,	///< For notifying errors
			NoteOff = 0x80,	///< Channel Message - Note Off
			NoteOn = 0x90,	///< Channel Message - Note On
			AfterTouchPoly = 0xA0,	///< Channel Message - Polyphonic AfterTouch
			ControlChange = 0xB0,	///< Channel Message - Control Change / Channel Mode
			ProgramChange = 0xC0,	///< Channel Message - Program Change
			AfterTouchChannel =
			0xD0,	///< Channel Message - Channel (monophonic) AfterTouch
			PitchBend = 0xE0,	///< Channel Message - Pitch Bend
			SystemExclusive = 0xF0,	///< System Exclusive
			SystemExclusiveStart = SystemExclusive,	///< System Exclusive Start
			TimeCodeQuarterFrame =
			0xF1,	///< System Common - MIDI Time Code Quarter Frame
			SongPosition = 0xF2,	///< System Common - Song Position Pointer
			SongSelect = 0xF3,	///< System Common - Song Select
			Undefined_F4 = 0xF4,
			Undefined_F5 = 0xF5,
			TuneRequest = 0xF6,	///< System Common - Tune Request
			SystemExclusiveEnd = 0xF7,	///< System Exclusive End
			Clock = 0xF8,	///< System Real Time - Timing Clock
			Undefined_F9 = 0xF9,
			Tick = Undefined_F9,	///< System Real Time - Timing Tick (1 tick = 10
			///< milliseconds)
			Start = 0xFA,	///< System Real Time - Start
			Continue = 0xFB,	///< System Real Time - Continue
			Stop = 0xFC,	///< System Real Time - Stop
			Undefined_FD = 0xFD,
			ActiveSensing = 0xFE,	///< System Real Time - Active Sensing
			SystemReset = 0xFF,	///< System Real Time - System Reset
	};
	public:
		void Begin(uart_inst *uartID, int txPin, int rxPin)
		{
			serial.Begin(uart0, baudrate, 0, 1);
		}

	void sendNoteOn(int pitch, int velocity)
	{
		serial.writeRaw(MidiType::NoteOn);
		serial.writeRaw(pitch);
		serial.writeRaw(velocity);
	}

	void sendNoteOff(int pitch, int velocity)
	{
		serial.writeRaw(MidiType::NoteOff);
		serial.writeRaw(pitch);
		serial.writeRaw(velocity);
	}

	void sendAftertouchPoly(int key, int touch)
	{
		serial.writeRaw(MidiType::AfterTouchPoly);
		serial.writeRaw(key);
		serial.writeRaw(touch);
	}

	void sendControlChange(int controller, int value)
	{
		serial.writeRaw(MidiType::ControlChange);
		serial.writeRaw(controller);
		serial.writeRaw(value);
	}

	void patchChange(int instrument)
	{
		serial.writeRaw(MidiType::ControlChange);
		serial.writeRaw(instrument);
	}

	void channelPressure(int pressure)
	{
		serial.writeRaw(MidiType::AfterTouchChannel);
		serial.writeRaw(pressure);
	}

	void pitchBend(int bend_lsb, int bend_msb)
	{
		serial.writeRaw(MidiType::PitchBend);
		serial.writeRaw(bend_lsb);
		serial.writeRaw(bend_msb);
	}

	void sendInvalidType()
	{
		serial.writeRaw(MidiType::InvalidType);
	}

	void systemReset()
	{
		serial.writeRaw(MidiType::SystemReset);
	}

	void activeSensing()
	{
		serial.writeRaw(MidiType::ActiveSensing);
	}
};

class USB_SERIAL
{
	public:
		void print(const char *str)
		{
			printf("%s", str);
		}

	void print(int value)
	{
		printf("%d", value);
	}

	void print(unsigned int value)
	{
		printf("%u", value);
	}

	void print(long value)
	{
		printf("%ld", value);
	}

	void print(unsigned long value)
	{
		printf("%lu", value);
	}

	void print(float value)
	{
		printf("%f", value);
	}

	void print(double value)
	{
		printf("%f", value);
	}

	void println(const char *str)
	{
		printf("%s\n", str);
	}

	void println(int value)
	{
		printf("%d\n", value);
	}

	void println(unsigned int value)
	{
		printf("%u\n", value);
	}

	void println(long value)
	{
		printf("%ld\n", value);
	}

	void println(unsigned long value)
	{
		printf("%lu\n", value);
	}

	void println(float value)
	{
		printf("%f\n", value);
	}

	void println(double value)
	{
		printf("%f\n", value);
	}

	void println()
	{
		printf("\n");
	}
};

class Temperature
{
	private:

		float read_onboard_temperature(const char unit)
		{
			/*12-bit conversion, assume max value == ADC_VREF == 3.3 V */
			const float conversionFactor = 3.3f / (1 << 12);

			float adc = (float) adc_read() *conversionFactor;
			float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

			if (unit == 'C')
			{
				return tempC;
			}
			else if (unit == 'F')
			{
				return tempC *9 / 5 + 32;
			}

			return -1.0f;
		}

	public:

		void Begin()
		{
			adc_set_temp_sensor_enabled(true);
		}

	float read(const char unit)
	{
		adc_select_input(4);
		return read_onboard_temperature(unit);
	}
};
