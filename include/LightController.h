#ifndef _LIGHT_CONTROLLER_H
#define _LIGHT_CONTROLLER_H

#include <Arduino.h>
#include "Adafruit_MCP23017.h"
#include "outputmap.h"

/*! \enum LightState
	\brief Possible light (outlet) states.
 */
enum class LightState: uint8_t {
	OFF = LOW,
	ON = HIGH
};

/*! \enum LightSelect
 	\brief Possible light (outlet) IDs.
 */
enum class LightSelect: uint8_t {
	ONE = 1,
	TWO = 2,
	THREE = 3,
	FOUR = 4,
	FIVE = 5
};

/*! \class LightController
    \brief This is an abstraction of the light controller circuitry. This includes the MCP23017 I2C I/O
    expander, the AQW214 PhotoMOS relays, and status LEDs.
 */
class LightController {
public:
	/*!
		\brief Default constructor.
		\param busController A pointer to the MCP23017 control library instance.
	*/
	LightController(Adafruit_MCP23017 *busController);

	//! Destructor.
	~LightController();

	/*!
		\brief Detect the controller presence.
		\return true if found; Otherwise, false.
	*/
	bool detect();

	/*!
		\brief Initializes the controller.
	*/
	void init();

	/*!
		\brief Gets the state of the specified light.
		\param light The light to get the state of.
		\return The state of the specified light.
	*/
	LightState getState(LightSelect light);

	/*!
		\brief Sets the state of the specified light.
		\param light The light to change the state of.
		\param state The state to set.
	*/
	void setState(LightSelect light, LightState state);

	/*!
		\brief Checks to see if the specified light is on.
		\param light The light to check.
		\return true if the ligth is on; Otherwise, false.
	*/
	bool isOn(LightSelect light);

	/*!
		\brief Checks to see if the specified light is off.
		\param light The light to check.
		\return true if the light is off; Otherwise, false.
	*/
	bool isOff(LightSelect light);

	/*!
		\brief Turns the specified light on.
		\param light The light to turn on.
	*/
	void turnOn(LightSelect light);

	/*!
		\brief Turns the specified light off.
		\param light The light to turn off.
	*/
	void turnOff(LightSelect light);

	/*!
		\brief Turns all the lights on.
	*/
	void allLightsOn();

	/*!
		\brief Turns all the lights off.
	*/
	void allLightsOff();
private:
	uint8_t getLightPinAddress(LightSelect light, LightState state);
	uint8_t getLedAddress(LightSelect light);
	Adafruit_MCP23017 *_busController;
};

#endif