#ifndef CONSOLE_H
#define CONSOLE_H

#include <Arduino.h>
#include <IPAddress.h>

/*! \class ConsoleClass
    \brief Provides a wrapper class around Serial that provides an interactive console for configuring and operating
    the device.
*/
class ConsoleClass
{
public:
    //! Default ctor.
    ConsoleClass();

    /*!
        \brief Gets an IP address from the specified string.
        \param value The string value to convert.
        \return An IPAddress object.
    */
    IPAddress getIPFromString(String value);

    //! Suspends execution until input is received from the user.
    void waitForUserInput();

    /*!
        \brief Gets the input string from the user.
        \param isPassword Indicates the input from the user is a password and the echoed text should be obfuscated (default is false).
        \return The string value received.
    */
    String getInputString(bool isPassword = false);

    //! Displays the menu interface and waits for user input.
    void enterCommandInterpreter();

    /*!
        \brief Sets the handler for the reboot command.
        \param rebootHandler A pointer to the handler method for the reboot command.
    */
    void onRebootCommand(void (*rebootHandler)());

    /*!
        \brief Sets the handler method for the scan networks command.
        \param scanHandler A pointer to the scan networks method.
    */
    void onScanNetworks(void (*scanHandler)());

    /*!
        \brief Sets the host name.
        \param hostname The host name to set.
    */
    void setHostname(String hostname);

    /*!
        \brief Sets the existing MQTT configuration.
        \param broker The IP or hostname of the MQTT broker.
        \param port The port on the MQTT broker to connect to.
        \param username The username to authenticate with the broker.
        \param password The password used for authentication.
        \param conChan The control topic name.
        \param statChan The status topic name.
    */
    void setMqttConfig(String broker, int port, String username, String password, String conChan, String statChan);

    /*!
        \brief Sets the handler for the host name change command.
        \param hostnameChangeHandler A pointer to the handler method for the host name change command.
    */
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

/*! \var Console */
extern ConsoleClass Console;
#endif