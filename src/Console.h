#ifndef CONSOLE_H
#define CONSOLE_H

#include <Arduino.h>
#include <IPAddress.h>

class ConsoleClass
{
public:
    ConsoleClass();
    IPAddress getIPFromString(String value);
    void waitForUserInput();
    String getInputString(bool isPassword = false);
    void enterCommandInterpreter();
    void onRebootCommand(void (*rebootHandler)());
    void onScanNetworks(void (*scanHandler)());
    void setHostname(String hostname);
    void setMqttConfig(String broker, int port, String username, String password, String conChan, String statChan);
    void onHostnameChange(void (*hostnameChangedHandler)(const char* newHostname));
    void onDhcpConfig(void (*dhcpHandler)());
    void onStaticConfig(void (*staticHandler)(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns));
    void onReconnectCommand(void (*reconnectHandler)());
    void onWifiConfigCommand(void (*wifiConfigHandler)(String newSsid, String newPassword));
    void onResumeCommand(void (*resumeHandler)());
    void onGetNetInfoCommand(void (*netinfoHandler)());
    void onSaveConfigCommand(void (*saveConfigHandler)());
    void onMqttConfigCommand(void (*mqttConfigHandler)(String newBroker, int newPort, String newUsername, String newPassword, String newConChan, String newStatChan));
    void onConsoleInterrupt(void (*interruptHandler)());
    void onFactoryRestore(void (*factoryRestoreHandler)());
    void onAllLightsOn(void (*allLightsOnHandler)());
    void onAllLightsOff(void (*allLightsOffHandler)());
    void checkInterrupt();

private:
    void displayMenu();
    void checkCommand();
    void configureStaticIP();
    void configureWiFiNetwork();
    void configMQTT();

    void (*rebootHandler)();
    void (*scanHandler)();
    void (*hostnameChangeHandler)(const char* newHostName);
    void (*dhcpHandler)();
    void (*staticHandler)(IPAddress newIp, IPAddress newSm, IPAddress newGw, IPAddress newDns);
    void (*reconnectHandler)();
    void (*wifiConfigHandler)(String newSsid, String newPassword);
    void (*resumeHandler)();
    void (*netInfoHandler)();
    void (*saveConfigHandler)();
    void (*mqttChangeHandler)(String newBroker, int newPort, String newUsername, String newPass, String newConChan, String newStatChan);
    void (*interruptHandler)();
    void (*factoryRestoreHandler)();
    void (*allLightsOnHandler)();
    void (*allLightsOffHandler)();
    
    String _hostname;
    String _mqttBroker;
    int _mqttPort;
    String _mqttUsername;
    String _mqttPassword;
    String _mqttControlChannel;
    String _mqttStatusChannel;
};

extern ConsoleClass Console;
#endif