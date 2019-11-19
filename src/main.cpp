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
#include <time.h>
#include "TelemetryHelper.h"
#include "LED.h"
#include "Relay.h"
#include "PubSubClient.h"
#include "TaskScheduler.h"
#include "ResetManager.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "CyMCP23016.h"
#include "ArduinoJson.h"
#include "config.h"

#define VERSION "1.1"

// Workaround to allow an MQTT packet size greater than the default of 128.
#ifdef MQTT_MAX_PACKET_SIZE
#undef MQTT_MAX_PACKET_SIZE
#endif
#define MQTT_MAX_PACKET_SIZE 200

// Pin definitions
#define PIN_ACT_LED 14
#define PIN_SDA 4
#define PIN_SCL 5
#define PIN_L1_LED MCP23016_PIN_GPIO0_2
#define PIN_L2_LED MCP23016_PIN_GPIO0_3
#define PIN_L3_LED MCP23016_PIN_GPIO0_4
#define PIN_L4_LED MCP23016_PIN_GPIO0_5
#define PIN_L5_LED MCP23016_PIN_GPIO0_6
#define PIN_L1_ON MCP23016_PIN_GPIO1_5
#define PIN_L1_OFF MCP23016_PIN_GPIO1_0
#define PIN_L2_ON MCP23016_PIN_GPIO1_1
#define PIN_L2_OFF MCP23016_PIN_GPIO1_2
#define PIN_L3_ON MCP23016_PIN_GPIO1_3
#define PIN_L3_OFF MCP23016_PIN_GPIO1_7
#define PIN_L4_ON MCP23016_PIN_GPIO1_4
#define PIN_L4_OFF MCP23016_PIN_GPIO1_6
#define PIN_L5_ON MCP23016_PIN_GPIO0_0
#define PIN_L5_OFF MCP23016_PIN_GPIO0_1

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
CyMCP23016 mcp;
PubSubClient mqttClient(wifiClient);
LED actLED(PIN_ACT_LED, NULL);
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Scheduler taskMan;
String ssid = DEFAULT_SSID;
String password = DEFAULT_PASSWORD;
String hostName = DEVICE_NAME;
String mqttBroker = MQTT_BROKER;
String controlChannel = MQTT_TOPIC_CONTROL;
String statusChannel = MQTT_TOPIC_STATUS;
String mqttUsername = "";
String mqttPassword = "";
String serverFingerprintPath;
String caCertificatePath;
String fingerprintString;
int mqttPort = MQTT_PORT;
bool isDHCP = false;
bool filesystemMounted = false;
bool connSecured = false;
volatile SystemState sysState = SystemState::BOOTING;
#ifdef ENABLE_OTA
    int otaPort = OTA_HOST_PORT;
    String otaPassword = OTA_PASSWORD;
#endif
LightState lastStates[5] = {
    LightState::OFF,
    LightState::OFF,
    LightState::OFF,
    LightState::OFF,
    LightState::OFF
};

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
 * Gets an IPAddress value from the specified string.
 * @param value The string containing the IP.
 * @return The IP address.
 */
IPAddress getIPFromString(String value) {
    unsigned int ip[4];
    unsigned char buf[value.length()];
    value.getBytes(buf, value.length());
    const char* ipBuf = (const char*)buf;
    sscanf(ipBuf, "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]);
    return IPAddress(ip[0], ip[1], ip[2], ip[3]);
}

/**
 * Publishes the system's current state to statusChannel.
 */
void publishSystemState() {
    if (mqttClient.connected()) {
        DynamicJsonDocument doc(200);
        doc["client_id"] = hostName;
        doc["systemState"] = (uint8_t)sysState;
        doc["firmwareVersion"] = VERSION;

        JsonObject lightObj;
        JsonArray lightStates = doc.createNestedArray("lights");
        for (uint8_t i = 1; i <= 5; i++) {
            lightObj = lightStates.createNestedObject();
            lightObj["id"] = i;
            lightObj["state"] = (uint8_t)lastStates[i - 1];
        }

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(statusChannel.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }
        doc.clear();
    }
}

/**
 * Toggles the state of a light (or whatever is plugged in to the remote
 * outlet) on or off. This will set the specified state and then publish
 * the new state to statusChannel, but only if the state is actually
 * changing.
 * @param light The light to set the state for.
 * @param state The state to set (on or off).
 */
void toggleLight(Light light, LightState state) {
    // We don't do anything if the state for this
    // light isn't actually changing.
    if (state == lastStates[(uint8_t)light - 1]) {
        return;
    }

    uint8_t onPin = PIN_L1_ON;
    uint8_t offPin = PIN_L1_OFF;
    uint8_t ledPin = PIN_L1_LED;
    switch (light) {
        case Light::ONE:
            break;
        case Light::TWO:
            onPin = PIN_L2_ON;
            offPin = PIN_L2_OFF;
            ledPin = PIN_L2_LED;
            break;
        case Light::THREE:
            onPin = PIN_L3_ON;
            offPin = PIN_L3_OFF;
            ledPin = PIN_L3_LED;
            break;
        case Light::FOUR:
            onPin = PIN_L4_ON;
            offPin = PIN_L4_OFF;
            ledPin = PIN_L4_LED;
            break;
        case Light::FIVE:
            onPin = PIN_L5_ON;
            offPin = PIN_L5_OFF;
            ledPin = PIN_L5_LED;
            break;
        default:
            break;
    }

    Serial.print(F("INFO: Switching light "));
    Serial.print((uint8_t)light);
    Serial.println(state == LightState::ON ? F("ON") : F("OFF"));
    lastStates[(uint8_t)light - 1] = state;
    mcp.digitalWrite(state == LightState::ON ? onPin : offPin, HIGH);
    mcp.digitalWrite(ledPin, state == LightState::ON ? HIGH : LOW);
    delay(BUTTON_PULSE_DELAY);
    mcp.digitalWrite(state == LightState::ON ? onPin : offPin, LOW);
    publishSystemState();
}

/**
 * Turns ALL lights (outlets) on.
 */
void allLightsOn() {
    for (uint8_t i = 1; i <= 5; i++) {
        toggleLight((Light)i, LightState::ON);
    }
}

/**
 * Turns ALL lights (outlets) off.
 */
void allLightsOff() {
    for (uint8_t i = 1; i <= 5; i++) {
        toggleLight((Light)i, LightState::OFF);
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
 * Waits for user input from the serial console.
 */
void waitForUserInput() {
    while (Serial.available() < 1) {
        ESPCrashMonitor.iAmAlive();
        delay(50);
    }
}

/**
 * Gets string input from the serial console. Echos each character back
 * as the user types it.
 * @returns Whatever the user entered at the console.
 */
String getInputString() {
    char c;
    String result = "";
    bool gotEndMarker = false;
    while (!gotEndMarker) {
        ESPCrashMonitor.iAmAlive();
        if (Serial.available() > 0) {
            c = Serial.read();
            if (c == '\n') {
                gotEndMarker = true;
                break;
            }

            Serial.print(c);
            result += c;
        }
    }

    return result;
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
    WiFi.printDiag(Serial);
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
    doc["hostname"] = hostName;
    doc["useDHCP"] = isDHCP;
    doc["ip"] = ip.toString();
    doc["gateway"] = gw.toString();
    doc["subnetMask"] = sm.toString();
    doc["dnsServer"] = dns.toString();
    doc["wifiSSID"] = ssid;
    doc["wifiPassword"] = password;
    doc["mqttBroker"] = mqttBroker;
    doc["mqttPort"] = mqttPort;
    doc["mqttControlChannel"] = controlChannel;
    doc["mqttStatusChannel"] = statusChannel;
    doc["mqttUsername"] = mqttUsername;
    doc["mqttPassword"] = mqttPassword;
    doc["serverFingerprintPath"] = serverFingerprintPath;
    doc["caCertificatePath"] = caCertificatePath;
    #ifdef ENABLE_OTA
        doc["otaPort"] = otaPort;
        doc["otaPassword"] = otaPassword;
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
    if (size > 1024) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Config file size is too large. Using default config."));
        configFile.close();
        return;
    }

    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);
    configFile.close();

    StaticJsonDocument<350> doc;
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to parse config file to JSON. Using default config."));
        return;
    }

    hostName = doc["hostname"].as<String>();
    isDHCP = doc["useDHCP"].as<bool>();
    if (!ip.fromString(doc["ip"].as<String>())) {
        Serial.println(F("WARN: Invalid IP in configuration. Falling back to factory default."));
    }

    if (!gw.fromString(doc["gateway"].as<String>())) {
        Serial.println(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
    }

    if (!sm.fromString(doc["subnetMask"].as<String>())) {
        Serial.println(F("WARN: Invalid subnet mask in configuration. Falling back to factory default."));
    }

    if (!dns.fromString(doc["dnsServer"].as<String>())) {
        Serial.println(F("WARN: Invalid DNS server in configuration. Falling back to factory default."));
    }

    ssid = doc["wifiSSID"].as<String>();
    password = doc["wifiPassword"].as<String>();
    mqttBroker = doc["mqttBroker"].as<String>();
    mqttPort = doc["mqttPort"].as<int>();
    controlChannel = doc["mqttControlChannel"].as<String>();
    statusChannel = doc["mqttStatusChannel"].as<String>();
    mqttUsername = doc["mqttUsername"].as<String>();
    mqttPassword = doc["mqttPassword"].as<String>();
    serverFingerprintPath = doc["serverFingerprintPath"].as<String>();
    caCertificatePath = doc["caCertificatePath"].as<String>();
    #ifdef ENABLE_OTA
        otaPort = doc["otaPort"].as<int>();
        otaPassword = doc["otaPassword"].as<String>();
    #endif

    doc.clear();
    buf.release();
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
    waitForUserInput();
    String str = getInputString();
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
 * @param id The system ID (host name) the control command is intended for.
 * @param command The command to execute.
 */
void handleControlRequest(String id, ControlCommand command) {
    id.toUpperCase();
    if (!id.equals(hostName)) {
        Serial.println(F("INFO: Control message not intended for this host. Ignoring..."));
        return;
    }

    if (sysState == SystemState::DISABLED &&
        command != ControlCommand::ENABLE) {
        // We can't process this command because we are disabled.
        Serial.print(F("WARN: Ignoring command "));
        Serial.print((uint8_t)command);
        Serial.print(F(" because the system is currently disabled."));
        return;
    }

    switch (command) {
        case ControlCommand::ALL_OFF:
            allLightsOff();
            break;
        case ControlCommand::ALL_ON:
            allLightsOn();
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
            toggleLight(Light::ONE, LightState::OFF);
            break;
        case ControlCommand::LIGHT1_ON:
            toggleLight(Light::ONE, LightState::ON);
            break;
        case ControlCommand::LIGHT2_OFF:
            toggleLight(Light::TWO, LightState::OFF);
            break;
        case ControlCommand::LIGHT2_ON:
            toggleLight(Light::TWO, LightState::ON);
            break;
        case ControlCommand::LIGHT3_OFF:
            toggleLight(Light::THREE, LightState::OFF);
            break;
        case ControlCommand::LIGHT3_ON:
            toggleLight(Light::THREE, LightState::ON);
            break;
        case ControlCommand::LIGHT4_OFF:
            toggleLight(Light::FOUR, LightState::OFF);
            break;
        case ControlCommand::LIGHT4_ON:
            toggleLight(Light::FOUR, LightState::ON);
            break;
        case ControlCommand::LIGHT5_OFF:
            toggleLight(Light::FIVE, LightState::OFF);
            break;
        case ControlCommand::LIGHT5_ON:
            toggleLight(Light::FIVE, LightState::ON);
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
 * Loads the SSL certificates and server fingerprint necessary to establish
 * a connection the the MQTT broker over TLS.
 * @return true if the certificates and server fingerprint were successfully
 * loaded; Otherwise, false.
 */
bool loadCertificates() {
    Serial.print(F("INFO: Loading SSL certificates... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return false;
    }

    if (!SPIFFS.exists(caCertificatePath)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: CA certificate does not exist."));
        return false;
    }

    File ca = SPIFFS.open(caCertificatePath, "r");
    if (!ca) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Could not open CA certificate."));
        return false;
    }

    String caContents = ca.readString();
    ca.close();
    X509List caCertX509(caContents.c_str());

    wifiClient.setTrustAnchors(&caCertX509);
    wifiClient.allowSelfSignedCerts();

    if (!SPIFFS.exists(serverFingerprintPath)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Server fingerprint file path does not exist."));
        return false;
    }

    File fp = SPIFFS.open(serverFingerprintPath, "r");
    if (!fp) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Could not open fingerprint file."));
        return false;
    }

    String val;
    if (fp.available()) {
        String fileContent = fp.readString();
        val = fileContent.substring(fileContent.lastIndexOf("=") + 1);
        val.replace(':', ' ');
    }

    fp.close();
    if (val.length() > 0) {
        fingerprintString = val;
        wifiClient.setFingerprint(fingerprintString.c_str());
    }
    else {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to read server fingerprint."));
        return false;
    }
    
    Serial.println(F("DONE"));
    return true;
}

/**
 * Verifies a connection can be made to the MQTT broker over TLS.
 * @return true if a connection to the MQTT broker over TLS was established
 * successfully; Otherwise, false.
 */
bool verifyTLS() {
    // Because it can take longer than expected to establish an
    // encrypted connection the MQTT broker, we need to disable
    // the watchdog to prevent reboot due to watchdog timeout during
    // connection, then re-enable when we are done.
    ESPCrashMonitor.disableWatchdog();

    // Currently, we sync the clock any time we need to verify TLS. This is
    // because in a future version, this will be required in order to validate
    // public CA certificates.
    onSyncClock();

    Serial.print(F("INFO: Verifying connectivity over TLS... "));
    bool result = wifiClient.connect(mqttBroker, mqttPort);
    if (result) {
        wifiClient.stop();
        Serial.println(F("DONE"));
    }
    else {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: TLS connection failed."));
    }

    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
    return result;
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

    String id = doc["client_id"].as<String>();
    ControlCommand cmd = (ControlCommand)doc["command"].as<uint8_t>();
    doc.clear();
    handleControlRequest(id, cmd);
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
        Serial.print(mqttBroker);
        Serial.print(F(" on port: "));
        Serial.print(mqttPort);
        Serial.println(F("..."));
        if (!connSecured) {
            connSecured = verifyTLS();
            if (!connSecured) {
                Serial.println(F("ERROR: Unable to establish TLS connection to host."));
                Serial.println(F("ERROR: Invalid certificate or SSL negotiation failed."));
                return false;
            }
        }

        bool didConnect = false;
        if (mqttUsername.length() > 0 && mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(hostName.c_str(), mqttUsername.c_str(), mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(hostName.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to channel: "));
            Serial.println(controlChannel);
            mqttClient.subscribe(controlChannel.c_str());

            Serial.print(F("INFO: Publishing to channel: "));
            Serial.println(statusChannel);
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
    Serial.print(F("INIT: CyLights v"));
    Serial.print(VERSION);
    Serial.println(F(" booting..."));
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
            if (!mdns.begin(hostName)) {
                Serial.println(F(" FAILED"));
                return;
            }
            
            #ifdef ENABLE_OTA
                mdns.addService(hostName, "ota", otaPort);
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
    mqttClient.setServer(mqttBroker.c_str(), mqttPort);
    mqttClient.setCallback(onMqttMessage);
    Serial.println(F("DONE"));
    if (reconnectMqttClient()) {
        delay(500);
        publishSystemState();
    }
}

/**
 * Prompts the user with a configuration screen and waits for
 * user input.
 */
void promptConfig() {
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F("= Command menu:              ="));
    Serial.println(F("=                            ="));
    Serial.println(F("= r: Reboot                  ="));
    Serial.println(F("= c: Configure network       ="));
    Serial.println(F("= m: Configure MQTT settings ="));
    Serial.println(F("= s: Scan wireless networks  ="));
    Serial.println(F("= n: Connect to new network  ="));
    Serial.println(F("= w: Reconnect to WiFi       ="));
    Serial.println(F("= e: Resume normal operation ="));
    Serial.println(F("= g: Get network info        ="));
    Serial.println(F("= f: Save config changes     ="));
    Serial.println(F("= z: Restore default config  ="));
    Serial.println(F("=                            ="));
    Serial.println(F("=============================="));
    Serial.println();
    Serial.println(F("Enter command choice (r/c/s/n/w/e/g/f/z): "));
    waitForUserInput();
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
    if (isDHCP) {
        // If set to all zeros, then the SDK assumes DHCP.
        WiFi.config(0U, 0U, 0U, 0U);
    }
    else {
        // If actual IP set, then disables DHCP and assumes static.
        WiFi.config(ip, gw, sm, dns);
    }

    Serial.println(F("DEBUG: Beginning connection..."));
    WiFi.begin(ssid, password);
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
 * Prompts the user for (and the applies) new MQTT configuration settings.
 */
void configureMQTT() {
    mqttClient.unsubscribe(controlChannel.c_str());
    mqttClient.disconnect();

    Serial.print(F("Current MQTT broker = "));
    Serial.println(mqttBroker);
    Serial.println(F("Enter MQTT broker address:"));
    waitForUserInput();
    mqttBroker = getInputString();
    Serial.println();
    Serial.print(F("New broker = "));
    Serial.println(mqttBroker);

    Serial.print(F("Current port = "));
    Serial.println(mqttPort);
    Serial.println(F("Enter MQTT broker port:"));
    waitForUserInput();
    String str = getInputString();
    mqttPort = str.toInt();
    Serial.println();
    Serial.print(F("New port = "));
    Serial.println(mqttPort);

    Serial.print(F("Current control channel = "));
    Serial.println(controlChannel);
    Serial.println(F("Enter MQTT control channel:"));
    waitForUserInput();
    controlChannel = getInputString();
    Serial.println();
    Serial.print(F("New control channel = "));
    Serial.println(controlChannel);

    Serial.print(F("Current status channel = "));
    Serial.println(statusChannel);
    Serial.println(F("Enter MQTT status channel:"));
    waitForUserInput();
    statusChannel = getInputString();
    Serial.println();
    Serial.print(F("New status channel = "));
    Serial.println(statusChannel);
    initMQTT();

    Serial.println();
}

/**
 * Prompts the user for, and configures static IP settings.
 */
void configureStaticIP() {
    isDHCP = false;
    Serial.println(F("Enter IP address: "));
    waitForUserInput();
    ip = getIPFromString(getInputString());
    Serial.print(F("New IP: "));
    Serial.println(ip);

    Serial.println(F("Enter gateway: "));
    waitForUserInput();
    gw = getIPFromString(getInputString());
    Serial.print(F("New gateway: "));
    Serial.println(gw);

    Serial.println(F("Enter subnet mask: "));
    waitForUserInput();
    sm = getIPFromString(getInputString());
    Serial.print(F("New subnet mask: "));
    Serial.println(sm);

    Serial.println(F("Enter DNS server: "));
    waitForUserInput();
    dns = getIPFromString(getInputString());
    Serial.print(F("New DNS server: "));
    Serial.println(dns);

    WiFi.config(ip, gw, sm, dns);  // If actual IP set, then disables DHCP and assumes static.
}

/**
 * Prompts the user for and then attempts to connect to a new
 * WiFi network.
 */
void configureWiFiNetwork() {
    Serial.println(F("Enter new SSID: "));
    waitForUserInput();
    ssid = getInputString();
    Serial.print(F("SSID = "));
    Serial.println(ssid);

    Serial.println(F("Enter new password: "));
    waitForUserInput();
    password = getInputString();
    Serial.print(F("Password = "));
    Serial.println(password);

    connectWifi();
}

/**
 * Checks commands entered by the user via serial input and carries out
 * the specified action if valid.
 */
void checkCommand() {
    String str = "";
    char incomingByte = Serial.read();
    switch (incomingByte) {
        case 'r':
            // Reset the controller.
            reboot();
            break;
        case 's':
            // Scan for available networks.
            getAvailableNetworks();
            promptConfig();
            checkCommand();
            break;
        case 'c':
            // Set hostname.
            Serial.print(F("Current host name: "));
            Serial.println(hostName);
            Serial.println(F("Set new host name: "));
            waitForUserInput();
            hostName = getInputString();
            initMDNS();

            // Change network mode.
            Serial.println(F("Choose network mode (d = DHCP, t = Static):"));
            waitForUserInput();
            checkCommand();
            break;
        case 'd':
            // Switch to DHCP mode.
            if (isDHCP) {
                Serial.println(F("INFO: DHCP mode already set. Skipping..."));
                Serial.println();
            }
            else {
                isDHCP = true;
                Serial.println(F("INFO: Set DHCP mode."));
                WiFi.config(0U, 0U, 0U, 0U);
            }
            promptConfig();
            checkCommand();
            break;
        case 't':
            // Switch to static IP mode. Request IP settings.
            configureStaticIP();
            promptConfig();
            checkCommand();
            break;
        case 'w':
            // Attempt to reconnect to WiFi.
            onCheckWiFi();
            if (WiFi.status() == WL_CONNECTED) {
                printNetworkInfo();
                resumeNormal();
            }
            else {
                Serial.println(F("ERROR: Still no network connection."));
                promptConfig();
                checkCommand();
            }
            break;
        case 'n':
            // Connect to a new wifi network.
            configureWiFiNetwork();
            promptConfig();
            checkCommand();
            break;
        case 'e':
            // Resume normal operation.
            resumeNormal();
            break;
        case 'g':
            // Get network info.
            printNetworkInfo();
            promptConfig();
            checkCommand();
            break;
        case 'f':
            // Save configuration changes and restart services.
            saveConfiguration();
            WiFi.disconnect(true);
            onCheckWiFi();
            promptConfig();
            checkCommand();
            break;
        case 'z':
            // Reset config to factory default.
            doFactoryRestore();
            promptConfig();
            checkCommand();
            break;
        case 'm':
            // Set MQTT settings.
            configureMQTT();
            promptConfig();
            checkCommand();
            break;
        default:
            // Specified command is invalid.
            Serial.println(F("WARN: Unrecognized command."));
            promptConfig();
            checkCommand();
            break;
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
    promptConfig();
    checkCommand();
}

/**
 * Check for the user to press the 'i' key at the serial console to
 * interrupt normal operation and present the configuration menu.
 */
void checkInterrupt() {
    if (Serial.available() > 0 && Serial.read() == 'i') {
        failSafe();
    }
}

/**
 * Initializes the WiFi network interface.
 */
void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    getAvailableNetworks();
    
    Serial.print(F("INFO: Connecting to SSID: "));
    Serial.print(ssid);
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
            ArduinoOTA.setPort(otaPort);
            ArduinoOTA.setHostname(hostName.c_str());
            ArduinoOTA.setPassword(otaPassword.c_str());
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
                bool useMDNS = true;
            #else
                bool useMDNS = false;
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

    if (loadCertificates()) {
        if (verifyTLS()) {
            connSecured = true;
            initMQTT();
        }
    }
    
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
