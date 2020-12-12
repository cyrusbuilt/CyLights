#ifndef _LIGHT_CONTROLLER_H
#define _LIGHT_CONTROLLER_H

#include <Arduino.h>
#include "Adafruit_MCP23017.h"
#include "outputmap.h"

/**
 * Possible light (outlet) states.
 */
enum class LightState: uint8_t {
	OFF = LOW,
	ON = HIGH
};

/**
 * Possible light (outlet) IDs.
 */
enum class LightSelect: uint8_t {
	ONE = 1,
	TWO = 2,
	THREE = 3,
	FOUR = 4,
	FIVE = 5
};

/**
 * This is an abstraction of the light controller circuitry. This includes the MCP23017 I2C I/O
 * expander, the AQW214 PhotoMOS relays, and status LEDs.
 */
class LightController {
public:
	LightController(Adafruit_MCP23017 *busController);
	~LightController();
	bool detect();
	void init();
	LightState getState(LightSelect light);
	void setState(LightSelect light, LightState state);
	bool isOn(LightSelect light);
	bool isOff(LightSelect light);
	void turnOn(LightSelect light);
	void turnOff(LightSelect light);
	void allLightsOn();
	void allLightsOff();
private:
	uint8_t getLightPinAddress(LightSelect light, LightState state);
	uint8_t getLedAddress(LightSelect light);
	Adafruit_MCP23017 *_busController;
};

#endif