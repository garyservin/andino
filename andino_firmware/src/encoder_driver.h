// AiEsp32RotaryEncoder.h
// based on https://github.com/marcmerlin/IoTuz code - extracted and modified Encoder code

#ifndef _ENCODER_DRIVER_h
#define _ENCODER_DRIVER_h

#include "Arduino.h"
//#include <FunctionalInterrupt.h>

// Rotary Encocer
#define ENCODER_DRIVER_DEFAULT_A_PIN 25
#define ENCODER_DRIVER_DEFAULT_B_PIN 26
#define ENCODER_DRIVER_DEFAULT_STEPS 2

class AiEsp32RotaryEncoder
{

private:
#if defined(ESP8266)
#else
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif
	volatile long encoder0Pos = 0;

	volatile int8_t lastMovementDirection = 0; // 1 right; -1 left
	volatile unsigned long lastMovementAt = 0;
	unsigned long rotaryAccelerationCoef = 150;

	bool _circleValues = false;
	bool isEnabled = true;

	uint8_t encoderAPin = ENCODER_DRIVER_DEFAULT_A_PIN;
	uint8_t encoderBPin = ENCODER_DRIVER_DEFAULT_B_PIN;
	long encoderSteps = ENCODER_DRIVER_DEFAULT_STEPS;

	long _minEncoderValue = -1 << 15;
	long _maxEncoderValue = 1 << 15;

	int8_t old_AB;
	long lastReadEncoder0Pos;

	int8_t enc_states[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
	void (*ISR_callback)();

public:
	AiEsp32RotaryEncoder(
		uint8_t encoderAPin = ENCODER_DRIVER_DEFAULT_A_PIN,
		uint8_t encoderBPin = ENCODER_DRIVER_DEFAULT_B_PIN,
		uint8_t encoderSteps = ENCODER_DRIVER_DEFAULT_STEPS);
	void setBoundaries(long minValue = -100, long maxValue = 100, bool circleValues = false);
	int correctionOffset=0;
	bool areEncoderPinsPulldownforEsp32 = true;
#if defined(ESP8266)
	ICACHE_RAM_ATTR void readEncoder_ISR();
#else
	void IRAM_ATTR readEncoder_ISR();

#endif

	void setup(void (*ISR_callback)(void));
	void begin();
	void reset(long newValue = 0);
	void enable();
	void disable();
	long read();
	void setValue(long newValue);
	long encoderChanged();
	unsigned long getAcceleration() { return this->rotaryAccelerationCoef; }
	void setAcceleration(unsigned long acceleration) { this->rotaryAccelerationCoef = acceleration; }
	void disableAcceleration() { setAcceleration(0); }
};
#endif
