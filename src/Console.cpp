#include "Console.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"

ConsoleClass::ConsoleClass() {}

void ConsoleClass::onRebootCommand(void (*rebootHandler)()) {
    this->rebootHandler = rebootHandler;
}

void ConsoleClass::onScanNetworks(void (*scanHandler)()) {
    this->scanHandler = scanHandler;
}

void ConsoleClass::setHostname(String hostname) {
    this->_hostname = hostname;
}

void ConsoleClass::onHostnameChange(void (*hostnameChangeHandler)(const char* newHostname)) {
    this->hostnameChangeHandler = hostnameChangeHandler;
}

void ConsoleClass::onDhcpConfig(void (*dhcpHandler)()) {
    this->dhcpHandler = dhcpHandler;
}

void ConsoleClass::onStaticConfig(void (*staticHandler)(IPAddress ip, IPAddress sm, IPAddress gw, IPAddress dns)) {
    this->staticHandler = staticHandler;
}

void ConsoleClass::onReconnectCommand(void (*reconnectHandler)()) {
    this->reconnectHandler = reconnectHandler;
}

void ConsoleClass::onWifiConfigCommand(void (*wifiConfigHandler)(String ssid, String password)) {
    this->wifiConfigHandler = wifiConfigHandler;
}

void ConsoleClass::onResumeCommand(void (*resumeHandler)()) {
    this->resumeHandler = resumeHandler;
}

void ConsoleClass::onGetNetInfoCommand(void (*netInfoHandler)()) {
    this->netInfoHandler = netInfoHandler;
}

void ConsoleClass::onSaveConfigCommand(void (*saveConfigHandler)()) {
    this->saveConfigHandler = saveConfigHandler;
}

void ConsoleClass::onMqttConfigCommand(void (*mqttChangeHandler)(String newBroker, int newPort, String newUsername, String newPassword, String newConChan, String newStatChan)) {
    this->mqttChangeHandler = mqttChangeHandler;
}

void ConsoleClass::onConsoleInterrupt(void (*interruptHandler)()) {
    this->interruptHandler = interruptHandler;
}

void ConsoleClass::onFactoryRestore(void (*factoryRestoreHandler)()) {
    this->factoryRestoreHandler = factoryRestoreHandler;
}

void ConsoleClass::onAllLightsOn(void (*allLightsOnHandler)()) {
    this->allLightsOnHandler = allLightsOnHandler;
}

void ConsoleClass::onAllLightsOff(void (*allLightsOffHandler)()) {
    this->allLightsOffHandler = allLightsOffHandler;
}

void ConsoleClass::setMqttConfig(String broker, int port, String username, String password, String conChan, String statChan) {
    this->_mqttBroker = broker;
    this->_mqttPort = port;
    this->_mqttUsername = username;
    this->_mqttPassword = password;
    this->_mqttControlChannel = conChan;
    this->_mqttStatusChannel = statChan;
}

IPAddress ConsoleClass::getIPFromString(String value) {
    unsigned int ip[4];
    unsigned char buf[value.length()];
    value.getBytes(buf, value.length());
    const char* ipBuf = (const char*)buf;
    sscanf(ipBuf, "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]);
    return IPAddress(ip[0], ip[1], ip[2], ip[3]);
}

void ConsoleClass::waitForUserInput() {
    while (Serial.available() < 1) {
        ESPCrashMonitor.iAmAlive();
        delay(50);
    }
}

String ConsoleClass::getInputString(bool isPassword) {
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

            Serial.print(isPassword ? '*' : c);
            result += c;
        }
    }

    return result;
}

void ConsoleClass::configureStaticIP() {
    Serial.println(F("Enter IP address: "));
    this->waitForUserInput();
    IPAddress ip = this->getIPFromString(this->getInputString());
    Serial.print(F("New IP: "));
    Serial.println(ip);

    Serial.println(F("Enter gateway: "));
    this->waitForUserInput();
    IPAddress gw = this->getIPFromString(this->getInputString());
    Serial.print(F("New gateway: "));
    Serial.println(gw);

    Serial.println(F("Enter subnet mask: "));
    this->waitForUserInput();
    IPAddress sm = this->getIPFromString(this->getInputString());
    Serial.print(F("New subnet mask: "));
    Serial.println(sm);

    Serial.println(F("Enter DNS server: "));
    this->waitForUserInput();
    IPAddress dns = this->getIPFromString(this->getInputString());
    Serial.print(F("New DNS server: "));
    Serial.println(dns);

    if (this->staticHandler != NULL) {
        this->staticHandler(ip, sm, gw, dns);
    }
}

void ConsoleClass::configureWiFiNetwork() {
    Serial.println(F("Enter new SSID: "));
    this->waitForUserInput();
    String ssid = this->getInputString();
    Serial.print(F("SSID = "));
    Serial.println(ssid);

    Serial.println(F("Enter new password: "));
    this->waitForUserInput();
    String password = this->getInputString();
    Serial.print(F("Password = "));
    Serial.println(password);

    if (this->wifiConfigHandler != NULL) {
        this->wifiConfigHandler(ssid, password);
    }
}

void ConsoleClass::configMQTT() {
    Serial.print(F("Current MQTT broker = "));
    Serial.println(this->_mqttBroker);
    Serial.println(F("Enter MQTT broker address:"));
    this->waitForUserInput();
    this->_mqttBroker = this->getInputString();
    Serial.println();
    Serial.print(F("New broker = "));
    Serial.println(this->_mqttBroker);

    Serial.print(F("Current port = "));
    Serial.println(this->_mqttPort);
    Serial.println(F("Enter MQTT broker port:"));
    this->waitForUserInput();
    String str = this->getInputString();
    this->_mqttPort = str.toInt();
    Serial.println();
    Serial.print(F("New port = "));
    Serial.println(this->_mqttPort);

    Serial.print(F("Current control channel = "));
    Serial.println(this->_mqttControlChannel);
    Serial.println(F("Enter MQTT control channel:"));
    this->waitForUserInput();
    this->_mqttControlChannel = this->getInputString();
    Serial.println();
    Serial.print(F("New control channel = "));
    Serial.println(this->_mqttControlChannel);

    Serial.print(F("Current status channel = "));
    Serial.println(this->_mqttStatusChannel);
    Serial.println(F("Enter MQTT status channel:"));
    this->waitForUserInput();
    this->_mqttStatusChannel = this->getInputString();
    Serial.println();
    Serial.print(F("New status channel = "));
    Serial.println(this->_mqttStatusChannel);

    Serial.print(F("Current username: "));
    Serial.println(this->_mqttUsername);
    Serial.println(F("Enter new username, or just press enter to clear:"));
    this->waitForUserInput();
    this->_mqttUsername = this->getInputString();
    Serial.print(F("New MQTT username = "));
    Serial.println(this->_mqttUsername);

    Serial.print(F("Current password: "));
    for (uint8_t i = 0; i < this->_mqttPassword.length(); i++) {
        Serial.print(F("*"));
    }

    Serial.println();
    Serial.print(F("Enter new password, or just press enter to clear"));
    this->waitForUserInput();
    this->_mqttPassword = this->getInputString(true);

    if (this->mqttChangeHandler != NULL) {
        this->mqttChangeHandler(
            this->_mqttBroker, this->_mqttPort,
            this->_mqttUsername,
            this->_mqttPassword,
            this->_mqttControlChannel,
            this->_mqttStatusChannel
        );
    }
}

void ConsoleClass::displayMenu() {
    Serial.println();
    Serial.println(F("=============================="));
    Serial.println(F("= Command menu:              ="));
    Serial.println(F("=                            ="));
    Serial.println(F("= h: All Lights On           ="));
    Serial.println(F("= i: All Lights Off          ="));
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
    Serial.println(F("Enter command choice (r/c/m/s/n/w/e/g/f/z): "));
    this->waitForUserInput();
}

void ConsoleClass::enterCommandInterpreter() {
    this->displayMenu();
    this->checkCommand();
}

void ConsoleClass::checkCommand() {
    String str = "";
    char incomingByte = Serial.read();
    switch (incomingByte) {
        case 'h':
            if (this->allLightsOnHandler != NULL) {
                this->allLightsOnHandler();
            }

            Serial.println();
            this->enterCommandInterpreter();
            break;
        case 'i':
            if (this->allLightsOffHandler != NULL) {
                this->allLightsOffHandler();
            }

            Serial.println();
            this->enterCommandInterpreter();
            break;
        case 'r':
            // Reset the controller.
            if (this->rebootHandler != NULL) {
                this->rebootHandler();
            }
            break;
        case 's':
            // Scan for available networks.
            if (this->scanHandler != NULL) {
                this->scanHandler();
            }
            
            this->enterCommandInterpreter();
            break;
        case 'c':
            // Set hostname.
            Serial.print(F("Current host name: "));
            Serial.println(this->_hostname);
            Serial.println(F("Set new host name: "));
            this->waitForUserInput();

            str = this->getInputString();
            if (this->hostnameChangeHandler != NULL) {
                this->hostnameChangeHandler(str.c_str());
            }

            // Change network mode.
            this->_hostname = str;
            Serial.println(F("Choose network mode (d = DHCP, t = Static):"));
            this->waitForUserInput();
            this->checkCommand();
            break;
        case 'd':
            if (this->dhcpHandler != NULL) {
                this->dhcpHandler();
            }

            this->enterCommandInterpreter();
            break;
        case 't':
            // Switch to static IP mode. Request IP settings.
            this->configureStaticIP();
            this->enterCommandInterpreter();
            break;
        case 'w':
            if (this->reconnectHandler != NULL) {
                this->reconnectHandler();
            }
            break;
        case 'n':
            this->configureWiFiNetwork();
            this->enterCommandInterpreter();
            break;
        case 'e':
            if (this->resumeHandler != NULL) {
                this->resumeHandler();
            }
            break;
        case 'g':
            if (this->netInfoHandler != NULL) {
                this->netInfoHandler();
            }

            this->enterCommandInterpreter();
            break;
        case 'f':
            if (this->saveConfigHandler != NULL) {
                this->saveConfigHandler();
            }

            this->enterCommandInterpreter();
            break;
        case 'm':
            this->configMQTT();
            this->enterCommandInterpreter();
            break;
        case 'z':
            if (this->factoryRestoreHandler != NULL) {
                this->factoryRestoreHandler();
            }
            break;
        default:
            // Specified command is invalid.
            Serial.println(F("WARN: Unrecognized command."));
            this->enterCommandInterpreter();
            break;
    }
}

void ConsoleClass::checkInterrupt() {
    if (Serial.available() > 0 && Serial.read() == 'i') {
        if (this->interruptHandler != NULL) {
            this->interruptHandler();
        }
    }
}

ConsoleClass Console;