/**
 * main.cpp
 * CyLights v1.1
 * 
 * Author:
 *  Chris (Cyrus) Brunner
 * 
 * This is the firmware for CyLights, an IoT integration system designed to
 * be used with a modified version of the Etekcity 5-outlet remote control
 * system. CyLights is meant to essentially convert the remote control into
 * an RF transmitter peripheral which is then powered and controlled by CyLights
 * itself and can then be controlled over MQTT and can be integrated with
 * home automation systems such as OpenHab.
 */

#ifndef ESP8266
    #error This firmware is only compatible with ESP8266 controllers.
#endif

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <FS.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <time.h>
#include "Adafruit_MCP23017.h"
#include "TelemetryHelper.h"
#include "LED.h"
#include "Relay.h"
#include "PubSubClient.h"
#include "TaskScheduler.h"
#include "ResetManager.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "ArduinoJson.h"
#include "LightController.h"
#include "Console.h"
#include "config.h"

#define VERSION "1.2"

#define PRIMARY_I2C_ADDRESS 0
#define I2C_ADDRESS_OFFSET 32

// Workaround to allow an MQTT packet size greater than the default of 128.
#ifdef MQTT_MAX_PACKET_SIZE
#undef MQTT_MAX_PACKET_SIZE
#endif
#define MQTT_MAX_PACKET_SIZE 200

// Pin definitions
#define PIN_ACT_LED 14

// Forward declarations
void onCheckWiFi();
void onCheckMqtt();
void failSafe();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
void onSyncClock();

// Global vars
#ifdef ENABLE_MDNS
    #include <ESP8266mDNS.h>
    MDNSResponder mdns;
#endif
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
LED actLED(PIN_ACT_LED, NULL);
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Scheduler taskMan;
Adafruit_MCP23017 bus;
LightController controller(&bus);
config_t config;
bool filesystemMounted = false;
bool primaryExpanderFound = false;
volatile SystemState sysState = SystemState::BOOTING;

/**
 * Synchronize the local system clock via NTP. Note: This does not take DST
 * into account. Currently, you will have to adjust the CLOCK_TIMEZONE define
 * manually to account for DST when needed.
 */
void onSyncClock() {
    configTime(CLOCK_TIMEZONE * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print(F("INIT: Waiting for NTP time sync..."));
    delay(500);
    while (!time(nullptr)) {
        ESPCrashMonitor.iAmAlive();
        Serial.print(F("."));
        delay(500);
    }

    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);

    Serial.println(F("DONE"));
    Serial.print(F("INFO: Current time: "));
    Serial.println(asctime(timeinfo));
}

/**
 * Publishes the system's current state to statusChannel.
 */
void publishSystemState() {
    if (mqttClient.connected()) {
        actLED.on();

        DynamicJsonDocument doc(200);
        doc["client_id"] = config.hostname;
        doc["systemState"] = (uint8_t)sysState;
        doc["firmwareVersion"] = VERSION;

        JsonObject lightObj;
        JsonArray lightStates = doc.createNestedArray("lights");
        for (uint8_t i = 1; i <= 5; i++) {
            lightObj = lightStates.createNestedObject();
            lightObj["id"] = i;
            lightObj["state"] = (uint8_t)controller.getState((LightSelect)i);
        }

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(config.mqttTopicStatus.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        actLED.off();
    }
}

/**
 * Disables the system and publishes the system's state to statusChannel,
 * then reboots the MCU.
 */
void reboot() {
    Serial.println(F("INFO: Rebooting... "));
    Serial.flush();
    delay(1000);
    sysState = SystemState::DISABLED;
    publishSystemState();
    ResetManager.softReset();
}

/**
 * Prints network information details to the serial console.
 */
void printNetworkInfo() {
    Serial.print(F("INFO: Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("INFO: Gateway: "));
    Serial.println(WiFi.gatewayIP());
    Serial.print(F("INFO: Subnet mask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("INFO: DNS server: "));
    Serial.println(WiFi.dnsIP());
    Serial.print(F("INFO: MAC address: "));
    Serial.println(WiFi.macAddress());
    #ifdef DEBUG
    WiFi.printDiag(Serial);
    #endif
}

/**
 * Scan for available networks and dump each discovered network to the console.
 */
void getAvailableNetworks() {
    ESPCrashMonitor.defer();
    Serial.println(F("INFO: Scanning WiFi networks..."));
    int numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        Serial.print(F("ID: "));
        Serial.print(i);
        Serial.print(F("\tNetwork name: "));
        Serial.print(WiFi.SSID(i));
        Serial.print(F("\tSignal strength:"));
        Serial.println(WiFi.RSSI(i));
    }
    Serial.println(F("----------------------------------"));
}

/**
 * Stores the in-memory configuration to a JSON file stored in SPIFFS.
 * If the file does not yet exist, it will be created (see CONFIG_FILE_PATH).
 * Errors will be reported to the serial console if the filesystem is not
 * mounted or if the file could not be opened for writing. Currently only
 * stores network configuration settings (IP, WiFi, etc).
 */
void saveConfiguration() {
    Serial.print(F("INFO: Saving configuration to: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.println(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }
    
    StaticJsonDocument<350> doc;
    doc["hostname"] = config.hostname;
    doc["useDHCP"] = config.useDhcp;
    doc["ip"] = config.ip.toString();
    doc["gateway"] = config.gw.toString();
    doc["subnetMask"] = config.sm.toString();
    doc["dnsServer"] = config.dns.toString();
    doc["wifiSSID"] = config.ssid;
    doc["wifiPassword"] = config.password;
    doc["mqttBroker"] = config.mqttBroker;
    doc["mqttPort"] = config.mqttPort;
    doc["mqttControlChannel"] = config.mqttTopicControl;
    doc["mqttStatusChannel"] = config.mqttTopicStatus;
    doc["mqttUsername"] = config.mqttUsername;
    doc["mqttPassword"] = config.mqttPassword;
    #ifdef ENABLE_OTA
        doc["otaPort"] = config.otaPort;
        doc["otaPassword"] = config.otaPassword;
    #endif

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "w");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to open config file for writing."));
        doc.clear();
        return;
    }

    serializeJsonPretty(doc, configFile);
    doc.clear();
    configFile.flush();
    configFile.close();
    Serial.println(F("DONE"));
}

void setConfigurationDefaults() {
    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = defHostname;
    config.ip = defaultIp;
    config.mqttBroker = MQTT_BROKER;
    config.mqttPassword = "";
    config.mqttPort = MQTT_PORT;
    config.mqttTopicControl = MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = MQTT_TOPIC_STATUS;
    config.mqttUsername = "";
    config.password = DEFAULT_PASSWORD;
    config.sm = defaultSm;
    config.ssid = DEFAULT_SSID;
    config.useDhcp = false;
    config.clockTimezone = CLOCK_TIMEZONE;
    config.dns = defaultDns;
    config.gw = defaultGw;

    #ifdef ENABLE_OTA
        config.otaPassword = OTA_PASSWORD;
        config.otaPort = OTA_HOST_PORT;
    #endif
}

void printWarningAndContinue(const __FlashStringHelper *message) {
    Serial.println();
    Serial.println(message);
    Serial.print(F("INFO: Continuing... "));
}

/**
 * Loads the configuration from CONFIG_FILE_PATH into memory and uses that as
 * the running configuration. Will report errors to the serial console and
 * revert to the default configuration under the following conditions:
 * 1) The filesystem is not mounted.
 * 2) The config file does not exist in SPIFFS. In this case a new file
 * will be created and populated with the default configuration.
 * 3) The config file exists, but could not be opened for reading.
 * 4) The config file is too big ( > 1MB).
 * 5) The config file could not be deserialized to a JSON structure.
 */
void loadConfiguration() {
    Serial.print(F("INFO: Loading config file: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.print(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    if (!SPIFFS.exists(CONFIG_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("WARN: Config file does not exist. Creating with default config..."));
        saveConfiguration();
        return;
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "r");
    if (!configFile) {
        Serial.println("FAIL");
        Serial.println(F("ERROR: Unable to open config file. Using default config."));
        return;
    }

    size_t size = configFile.size();
    uint16_t freeMem = ESP.getMaxFreeBlockSize() - 512;
    if (size > freeMem) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: Not enough free memory to load document. Size = "));
        Serial.print(size);
        Serial.print(F(", Free = "));
        Serial.println(freeMem);
        configFile.close();
        return;
    }

    DynamicJsonDocument doc(freeMem);
    DeserializationError error = deserializeJson(doc, configFile);
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to parse config file to JSON. Using default config."));
        return;
    }

    doc.shrinkToFit();
    configFile.close();

    String chipId = String(ESP.getChipId(), HEX);
    String defHostname = String(DEVICE_NAME) + "_" + chipId;

    config.hostname = doc.containsKey("hostname") ? doc["hostname"].as<String>() : defHostname;
    config.useDhcp = doc.containsKey("isDhcp") ? doc["isDhcp"].as<bool>() : false;
    
    if (doc.containsKey("ip")) {
        if (!config.ip.fromString(doc["ip"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.ip = defaultIp;
    }

    if (doc.containsKey("gateway")) {
        if (!config.gw.fromString(doc["gateway"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
        }
    }
    else {
        config.gw = defaultGw;
    }

    if (doc.containsKey("subnetmask")) {
        if (!config.sm.fromString(doc["subnetmask"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid subnet mask in configuration. Falling back to factory default."));
        }
    }
    else {
        config.sm = defaultSm;
    }

    if (doc.containsKey("dns")) {
        if (!config.dns.fromString(doc["dns"].as<String>())) {
            printWarningAndContinue(F("WARN: Invalid DNS IP in configuration. Falling back to factory default."));
        }
    }
    else {
        config.dns = defaultDns;
    }

    config.ssid = doc.containsKey("wifiSSID") ? doc["wifiSSID"].as<String>() : DEFAULT_SSID;
    config.password = doc.containsKey("wifiPassword") ? doc["wifiPassword"].as<String>() : DEFAULT_PASSWORD;
    config.mqttBroker = doc.containsKey("mqttBroker") ? doc["mqttBroker"].as<String>() : MQTT_BROKER;
    config.mqttPort = doc.containsKey("mqttPort") ? doc["mqttPort"].as<int>() : MQTT_PORT;
    config.mqttTopicControl = doc.containsKey("mqttControlChannel") ? doc["mqttControlChannel"].as<String>() : MQTT_TOPIC_CONTROL;
    config.mqttTopicStatus = doc.containsKey("mqttStatusChannel") ? doc["mqttStatusChannel"].as<String>() : MQTT_TOPIC_STATUS;
    config.mqttUsername = doc.containsKey("mqttUsername") ? doc["mqttUsername"].as<String>() : "";
    config.mqttPassword = doc.containsKey("mqttPassword") ? doc["mqttPassword"].as<String>() : "";

    #ifdef ENABLE_OTA
        config.otaPort = doc.containsKey("otaPort") ? doc["otaPort"].as<uint16_t>() : MQTT_PORT;
        config.otaPassword = doc.containsKey("otaPassword") ? doc["otaPassword"].as<String>() : OTA_PASSWORD;
    #endif

    doc.clear();
    Serial.println(F("DONE"));
}

/**
 * Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
    actLED.off();
    sysState = SystemState::NORMAL;
    publishSystemState();
}

/**
 * Confirms with the user that they wish to do a factory restore. If so, then
 * clears the current configuration file in SPIFFS, then reboots. Upon reboot,
 * a new config file will be generated with default values.
 */
void doFactoryRestore() {
    Serial.println();
    Serial.println(F("Are you sure you wish restore to factory default? (Y/n)?"));
    Console.waitForUserInput();
    String str = Console.getInputString();
    if (str == "Y" || str == "y") {
        Serial.print(F("INFO: Clearing current config... "));
        if (filesystemMounted) {
            if (SPIFFS.remove(CONFIG_FILE_PATH)) {
                Serial.println(F("DONE"));
                Serial.print(F("INFO: Removed file: "));
                Serial.println(CONFIG_FILE_PATH);

                Serial.print(F("INFO: Rebooting in "));
                for (uint8_t i = 5; i >= 1; i--) {
                    Serial.print(i);
                    Serial.print(F(" "));
                    delay(1000);
                }

                reboot();
            }
            else {
                Serial.println(F("FAIL"));
                Serial.println(F("ERROR: Failed to delete configuration file."));
            }
        }
        else {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Fileystem not mounted."));
        }
    }

    Serial.println();
}

/**
 * Handles incoming system control requests. This executes the specified
 * command if intended for this system, then responds by publishing the
 * system state to statusChannel.
 * @param command The command to execute.
 */
void handleControlRequest(ControlCommand command) {
    // When system is the "disabled" state, the only command it will accept
    // is "enable". All other commands are ignored.
    if (sysState == SystemState::DISABLED && command != ControlCommand::ENABLE) {
        // We can't process this command because we are disabled.
        Serial.print(F("WARN: Ignoring command "));
        Serial.print((uint8_t)command);
        Serial.print(F(" because the system is currently disabled."));
        return;
    }

    switch (command) {
        case ControlCommand::ALL_OFF:
            controller.allLightsOff();
            break;
        case ControlCommand::ALL_ON:
            controller.allLightsOn();
            break;
        case ControlCommand::DISABLE:
            Serial.println(F("WARN: Disabling system."));
            sysState = SystemState::DISABLED;
            break;
        case ControlCommand::ENABLE:
            Serial.println(F("INFO: Enabling system."));
            sysState = SystemState::NORMAL;
            break;
        case ControlCommand::LIGHT1_OFF:
            controller.setState(LightSelect::ONE, LightState::OFF);
            break;
        case ControlCommand::LIGHT1_ON:
            controller.setState(LightSelect::ONE, LightState::ON);
            break;
        case ControlCommand::LIGHT2_OFF:
            controller.setState(LightSelect::TWO, LightState::OFF);
            break;
        case ControlCommand::LIGHT2_ON:
            controller.setState(LightSelect::TWO, LightState::ON);
            break;
        case ControlCommand::LIGHT3_OFF:
            controller.setState(LightSelect::THREE, LightState::OFF);
            break;
        case ControlCommand::LIGHT3_ON:
            controller.setState(LightSelect::THREE, LightState::ON);
            break;
        case ControlCommand::LIGHT4_OFF:
            controller.setState(LightSelect::FOUR, LightState::OFF);
            break;
        case ControlCommand::LIGHT4_ON:
            controller.setState(LightSelect::FOUR, LightState::ON);
            break;
        case ControlCommand::LIGHT5_OFF:
            controller.setState(LightSelect::FIVE, LightState::OFF);
            break;
        case ControlCommand::LIGHT5_ON:
            controller.setState(LightSelect::FIVE, LightState::ON);
            break;
        case ControlCommand::REBOOT:
            reboot();
            break;
        case ControlCommand::REQUEST_STATUS:
            break;
        default:
            Serial.print(F("WARN: Unknown command: "));
            Serial.println((uint8_t)command);
            break;
    }

    publishSystemState();
}

/**
 * Handles incoming messages from the controlChannel channel the
 * system is subscribed to.
 * @param topic The topic the message was received on.
 * @param payload The message received.
 * @param length The length of the message.
 */
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    Serial.print(F("INFO: [MQTT] Message arrived: ["));
    Serial.print(topic);
    Serial.print(F("] "));

    // It's a lot easier to deal with if we just convert the payload
    // to a string first.
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.println(msg);

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.print(F("ERROR: Failed to parse MQTT message to JSON: "));
        Serial.println(error.c_str());
        doc.clear();
        return;
    }

    if (doc.containsKey("client_id")) {
        String id = doc["client_id"].as<String>();
        id.toUpperCase();
        if (!id.equals(config.hostname)) {
            Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
            doc.clear();
            return;
        }
    }
    else {
        Serial.println(F("WARN: MQTT message does not contain client ID. Ignoring..."));
        doc.clear();
        return;
    }

    if (!doc.containsKey("command")) {
        Serial.println(F("WARN: MQTT message does not contain a control command. Ignoring..."));
        doc.clear();
        return;
    }

    ControlCommand cmd = (ControlCommand)doc["command"].as<uint8_t>();
    handleControlRequest(cmd);
}

/**
 * Attempts to re-connect to the MQTT broker if then connection is
 * currently broken.
 * @return true if a connection to the MQTT broker is either already
 * established, or was successfully re-established; Otherwise, false.
 * If the connection is re-established, then will also re-subscribe to
 * the status channel.
 */
bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
        Serial.print(F("INFO: Attempting to establish MQTT connection to "));
        Serial.print(config.mqttBroker);
        Serial.print(F(" on port: "));
        Serial.print(config.mqttPort);
        Serial.println(F("..."));
        
        bool didConnect = false;
        if (config.mqttUsername.length() > 0 && config.mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(config.hostname.c_str(), config.mqttUsername.c_str(), config.mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(config.hostname.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to channel: "));
            Serial.println(config.mqttTopicControl);
            mqttClient.subscribe(config.mqttTopicControl.c_str());

            Serial.print(F("INFO: Publishing to channel: "));
            Serial.println(config.mqttTopicStatus);
        }
        else {
            String failReason = TelemetryHelper::getMqttStateDesc(mqttClient.state());
            Serial.print(F("ERROR: Failed to connect to MQTT broker: "));
            Serial.println(failReason);
            return false;
        }
    }

    return true;
}

/**
 * Callback method for the MQTT check task. This will check to see if still
 * connected to the MQTT broker and then attempt to reconnect if not.
 */
void onCheckMqtt() {
    Serial.println(F("INFO: Checking MQTT connection status..."));
    if (reconnectMqttClient()) {
        Serial.println(F("INFO: Successfully reconnected to MQTT broker."));
        publishSystemState();
    }
    else {
        Serial.println(F("ERROR: MQTT connection lost and reconnect failed."));
        Serial.print(F("INFO: Retrying connection in "));
        Serial.print(CHECK_MQTT_INTERVAL % 1000);
        Serial.println(F(" seconds."));
    }
}

/**
 * Initializes the hardware serial port.
 */
void initSerial() {
    Serial.begin(SERIAL_BAUD);
    #ifdef DEBUG
    const bool serDebug = true;
    #else
    const bool serDebug = false;
    #endif
    Serial.setDebugOutput(serDebug);
    Serial.print(F("INIT: CyLights v"));
    Serial.print(VERSION);
    Serial.println(F(" booting..."));
}

void scanBusDevices() {
    byte error;
    byte address;
    int devices = 0;

    // NOTE: We can have a max of only (8) MCP23017 chips connected to the bus at a time.
    // This gives a range of addresses 0 - 7 (which really translates to 32 - 39).
    Serial.println(F("INFO: Beginning I2C bus scan ..."));
    for (address = 32; address < 40; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            devices++;
            Serial.print(F("INFO: I2C device found at address 0x"));
            if (address < 16) {
                Serial.print(F("0"));
            }

            Serial.print(address, HEX);
            Serial.println(F("!"));
            uint8_t realAddress = address - I2C_ADDRESS_OFFSET;
            if (!primaryExpanderFound && realAddress == PRIMARY_I2C_ADDRESS) {
                primaryExpanderFound = true;
            }
            // TODO we may need this if we ever want to support expansion.
            // else {
            //     devicesFound.push_back(realAddress);
            // }
        }
        else if (error == 4) {
            Serial.print(F("ERROR: Unknown error at address 0x"));
            if (address < 16) {
                Serial.print(F("0"));
            }

            Serial.println(address, HEX);
        }
    }

    if (devices == 0) {
        Serial.println(F("ERROR: No devices found!"));
    }
    else {
        Serial.println(F("INFO: Scan complete."));
    }
}

/**
 * Initializes all output pins.
 */
void initOutputs() {
    Serial.print(F("INIT: Initializing outputs... "));
    actLED.init();
    actLED.on();
    mcp.begin(PIN_SDA, PIN_SCL);
    if (!mcp.detected()) {
        Serial.println(F("FAIL"));
        Serial.print(F("ERROR: I2C device not found at address: "));
        Serial.println(MCP23016_ADDRESS, HEX);
        return;
    }

    allLightsOff();
    Serial.println(F("DONE"));
}

/**
 * Initialize the SPIFFS filesystem.
 */
void initFilesystem() {
    Serial.print(F("INIT: Initializing SPIFFS and mounting filesystem... "));
    if (!SPIFFS.begin()) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to mount filesystem."));
        return;
    }

    filesystemMounted = true;
    Serial.println(F("DONE"));
    loadConfiguration();
}

/**
 * Initializes the MDNS responder (if enabled).
 */
void initMDNS() {
    #ifdef ENABLE_MDNS
        Serial.print(F("INIT: Starting MDNS responder... "));
        if (WiFi.status() == WL_CONNECTED) {
            ESPCrashMonitor.defer();
            delay(500);
            if (!mdns.begin(config.hostname)) {
                Serial.println(F(" FAILED"));
                return;
            }
            
            #ifdef ENABLE_OTA
                mdns.addService(config.hostname, "ota", config.otaPort);
            #endif
            Serial.println(F(" DONE"));
        }
        else {
            Serial.println(F(" FAILED"));
        }
    #endif
}

/**
 * Initializes the MQTT client.
 */
void initMQTT() {
    Serial.print(F("INIT: Initializing MQTT client... "));
    mqttClient.setServer(config.mqttBroker.c_str(), config.mqttPort);
    mqttClient.setCallback(onMqttMessage);
    Serial.println(F("DONE"));
    if (reconnectMqttClient()) {
        delay(500);
        publishSystemState();
    }
}

/**
 * Attempt to connect to the configured WiFi network. This will break any existing connection first.
 */
void connectWifi() {
    Serial.println(F("DEBUG: Setting mode..."));
    WiFi.mode(WIFI_STA);
    Serial.println(F("DEBUG: Disconnect and clear to prevent auto connect..."));
    WiFi.persistent(false);
    WiFi.disconnect(true);
    ESPCrashMonitor.defer();

    delay(1000);
    if (config.useDhcp) {
        // If set to all zeros, then the SDK assumes DHCP.
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        // If actual IP set, then disables DHCP and assumes static.
        WiFi.config(config.ip, config.gw, config.sm, config.dns);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(config.ssid, config.password);
    Serial.println(F("DEBUG: Waiting for connection..."));
    
    const int maxTries = 20;
    int currentTry = 0;
    while ((WiFi.status() != WL_CONNECTED) && (currentTry < maxTries)) {
        ESPCrashMonitor.iAmAlive();
        currentTry++;
        actLED.blink(500);
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("ERROR: Failed to connect to WiFi!"));
        failSafe();
    }
    else {
        printNetworkInfo();
    }
}

/**
 * Enter fail-safe mode. This will suspend all tasks, disable relay activation,
 * and propmpt the user for configuration.
 */
void failSafe() {
    ESPCrashMonitor.defer();
    Serial.println();
    Serial.println(F("ERROR: Entering failsafe (config) mode..."));
    taskMan.disableAll();
    actLED.on();
    sysState = SystemState::DISABLED;
    publishSystemState();
    Console.enterCommandInterpreter();
}

/**
 * Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to SSID: "));
    Serial.print(config.ssid);
    Serial.println(F("..."));
    
    connectWifi();
}

/**
 * Initializes the OTA update listener if enabled.
 */
void initOTA() {
    #ifdef ENABLE_OTA
        Serial.print(F("INIT: Starting OTA updater... "));
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setPort(config.otaPort);
            ArduinoOTA.setHostname(config.hostname.c_str());
            ArduinoOTA.setPassword(config.otaPassword.c_str());
            ArduinoOTA.onStart([]() {
                sysState = SystemState::UPDATING;
                // Handles start of OTA update. Determines update type.
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                }
                else {
                    type = "filesystem";
                }
                Serial.println("INFO: Starting OTA update (type: " + type + ") ...");
            });
            ArduinoOTA.onEnd([]() {
                // Handles update completion.
                Serial.println(F("INFO: OTA updater finished."));
            });
            ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
                // Reports update progress.
                ESPCrashMonitor.iAmAlive();
                Serial.printf("INFO: OTA Update Progress: %u%%\r", (progress / (total / 100)));
            });
            ArduinoOTA.onError([](ota_error_t error) {
                // Handles OTA update errors.
                Serial.printf("ERROR: OTA update error [%u]: ", error);
                switch(error) {
                    case OTA_AUTH_ERROR:
                        Serial.println(F("Auth failed."));
                        break;
                    case OTA_BEGIN_ERROR:
                        Serial.println(F("Begin failed."));
                        break;
                    case OTA_CONNECT_ERROR:
                        Serial.println(F("Connect failed."));
                        break;
                    case OTA_RECEIVE_ERROR:
                        Serial.println(F("Receive failed."));
                        break;
                    case OTA_END_ERROR:
                        Serial.println(F("End failed."));
                        break;
                }
            });

            #ifdef ENABLE_MDNS
                const bool useMDNS = true;
            #else
                const bool useMDNS = false;
            #endif

            ArduinoOTA.begin(useMDNS);
            Serial.println(F("DONE"));
        }
        else {
            Serial.println(F("FAIL"));
        }
    #endif
}

/**
 * Initializes the task manager and all recurring tasks.
 */
void initTaskManager() {
    Serial.print(F("INIT: Initializing task scheduler... "));

    taskMan.init();
    taskMan.addTask(tCheckWifi);
    taskMan.addTask(tCheckMqtt);
    
    tCheckWifi.enableDelayed(30000);
    tCheckMqtt.enableDelayed(1000);
    Serial.println(F("DONE"));
}

/**
 * Callback routine for checking WiFi connectivity.
 */
void onCheckWiFi() {
    Serial.println(F("INFO: Checking WiFi connectivity..."));
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WARN: Lost connection. Attempting reconnect..."));
        connectWifi();
        if (WiFi.status() == WL_CONNECTED) {
            initMDNS();
            initOTA();
            initMQTT();
        }
    }
}

/**
 * Initializes the crash monitor and dump any previous crash data to the serial console.
 */
void initCrashMonitor() {
    Serial.print(F("INIT: Initializing crash monitor... "));
    ESPCrashMonitor.disableWatchdog();
    Serial.println(F("DONE"));
    ESPCrashMonitor.dump(Serial);
    delay(100);
}

/**
 * Bootstrap routine. This is called once at boot and initializes all
 * subsystems.
 */
void setup() {
	initSerial();
    initCrashMonitor();
    initOutputs();
    initFilesystem();
    initWiFi();
    initMDNS();
    initOTA();
    initMQTT();    
    initTaskManager();
    sysState = SystemState::NORMAL;
    Serial.println(F("INIT: Boot sequence complete."));
    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
}

/**
 * The main program loop.
 */
void loop() {
    ESPCrashMonitor.iAmAlive();
    checkInterrupt();
    taskMan.execute();
    #ifdef ENABLE_MDNS
        mdns.update();
    #endif
    #ifdef ENABLE_OTA
        ArduinoOTA.handle();
    #endif
    mqttClient.loop();
}
