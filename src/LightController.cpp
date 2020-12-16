#include "LightController.h"

LightController::LightController(Adafruit_MCP23017 *busController) {
	this->_busController = busController;
}

LightController::~LightController() {
	delete this->_busController;
}

uint8_t LightController::getLightPinAddress(LightSelect light, LightState state) {
	uint8_t result = -1;
	switch (light) {
		case LightSelect::ONE:
			switch(state) {
				case LightState::ON:
					result = LIGHT_1_ON_RELAY;
					break;
				case LightState::OFF:
					result = LIGHT_2_OFF_RELAY;
					break;
				default:
					break;
			}
			break;
		case LightSelect::TWO:
			switch (state) {
				case LightState::ON:
					result = LIGHT_2_ON_RELAY;
					break;
				case LightState::OFF:
					result = LIGHT_2_OFF_RELAY;
					break;
				default:
					break;
			}
			break;
		case LightSelect::THREE:
			switch (state) {
				case LightState::ON:
					result = LIGHT_3_ON_RELAY;
					break;
				case LightState::OFF:
					result = LIGHT_3_OFF_RELAY;
					break;
				default:
					break;
			}
			break;
		case LightSelect::FOUR:
			switch (state) {
				case LightState::ON:
					result = LIGHT_4_ON_RELAY;
					break;
				case LightState::OFF:
					result = LIGHT_4_OFF_RELAY;
					break;
				default:
					break;
			}
		case LightSelect::FIVE:
			switch (state) {
				case LightState::ON:
					result = LIGHT_5_ON_RELAY;
					break;
				case LightState::OFF:
					result = LIGHT_5_OFF_RELAY;
					break;
			}
			break;
		default:
			break;
	}

	return result;
}

uint8_t LightController::getLedAddress(LightSelect light) {
	uint8_t result = -1;
	switch (light) {
		case LightSelect::ONE:
			result = CHAN_1_LED;
			break;
		case LightSelect::TWO:
			result = CHAN_2_LED;
			break;
		case LightSelect::THREE:
			result = CHAN_3_LED;
			break;
		case LightSelect::FOUR:
			result = CHAN_4_LED;
			break;
		case LightSelect::FIVE:
			result = CHAN_5_LED;
			break;
	}

	return result;
}

bool LightController::detect() {
	uint8_t address = this->getLightPinAddress(LightSelect::ONE, LightState::ON);
	this->_busController->pinMode(address, INPUT);
	this->_busController->pullUp(address, LOW);
	return this->_busController->digitalRead(address) == LOW;
}

void LightController::init() {
	for (uint8_t i = 1; i <= 5; i++) {
		uint8_t relayOnAddress = this->getLightPinAddress((LightSelect)i, LightState::ON);
		this->_busController->pinMode(relayOnAddress, OUTPUT);
		this->_busController->digitalWrite(relayOnAddress, LOW);

		uint8_t relayOffAddress = this->getLightPinAddress((LightSelect)i, LightState::OFF);
		this->_busController->pinMode(relayOffAddress, LOW);
		this->_busController->digitalWrite(relayOffAddress, LOW);

		uint8_t ledAddress = this->getLedAddress((LightSelect)i);
		this->_busController->pinMode(ledAddress, OUTPUT);
		this->_busController->digitalWrite(ledAddress, LOW);
	}
}

LightState LightController::getState(LightSelect light) {
	uint8_t ledAddress = this->getLedAddress(light);
	return (LightState)this->_busController->digitalRead(ledAddress);
}

void LightController::setState(LightSelect light, LightState state) {
	if (this->getState(light) != state) {
		uint8_t lightPinAddress = this->getLightPinAddress(light, state);
		this->_busController->digitalWrite(lightPinAddress, (uint8_t)state);

		uint8_t ledAddress = this->getLedAddress(light);
		this->_busController->digitalWrite(ledAddress, HIGH);
		
		delay(1000);
		this->_busController->digitalWrite(lightPinAddress, LOW);
	}
}

bool LightController::isOn(LightSelect light) {
	return this->getState(light) == LightState::ON;
}

bool LightController::isOff(LightSelect light) {
	return this->getState(light) == LightState::OFF;
}

void LightController::turnOn(LightSelect light) {
	this->setState(light, LightState::ON);
}

void LightController::turnOff(LightSelect light) {
	this->setState(light, LightState::OFF);
}

void LightController::allLightsOn() {
	for (uint8_t i = 1; i <= 5; i++) {
		this->turnOn((LightSelect)i);
	}
}

void LightController::allLightsOff() {
	for (uint8_t i = 1; i <= 5; i++) {
		this->turnOff((LightSelect)i);
	}
}