#ifndef config_h
#define config_h

#include <IPAddress.h>

// Configuration
#define DEBUG                                   // Uncomment to enable additional debug info.
#define ENABLE_OTA                              // Comment this line to disable OTA updates.
#define ENABLE_MDNS                             // Comment this line to disable the MDNS.
#define DEFAULT_SSID "your_ssid_here"           // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "your_password_here"   // Put your WiFi password here.
#define CLOCK_TIMEZONE -4                       // The timezone this device is located in. (For example, EST when observing DST = GMT-4, when not = GMT-5)
#define SERIAL_BAUD 115200                      // The BAUD rate (speed) of the serial port (console).
#define CONFIG_FILE_PATH "/config.json"         // The path to the config file in SPIFFS.
#define CHECK_WIFI_INTERVAL 30000               // How often to check WiFi status (milliseconds).
#define CHECK_MQTT_INTERVAL 35000               // How often to check connectivity to the MQTT broker.
#define CLOCK_SYNC_INTERVAL 3600000             // How often to sync the local clock with NTP (milliseconds).
#define MQTT_TOPIC_STATUS "cylights/status"     // The MQTT channel to publish status messages to.
#define MQTT_TOPIC_CONTROL "cylights/control"   // The MQTT channel to subscribe to for control messages.
#define DEVICE_NAME "CYLIGHTS"                  // The device name.
#define BUTTON_PULSE_DELAY 1000                 // How long to pulse an on/off button.
#define MQTT_BROKER "your_mqtt_host_here"       // The host name or IP of the MQTT broker to connect to.
#define MQTT_PORT 1883                          // The MQTT port to connect on (default is 1883).
#ifdef ENABLE_OTA
    #include <ArduinoOTA.h>
    #define OTA_HOST_PORT 8266                     // The OTA updater port.
    #define OTA_PASSWORD "your_ota_password_here"  // The OTA updater password.
#endif
IPAddress defaultIp(192, 168, 0, 220);                 // The default static host IP.
IPAddress defaultGw(192, 168, 0, 1);                   // The default static gateway IP.
IPAddress defaultSm(255, 255, 255, 0);                 // The default static subnet mask.
IPAddress defaultDns(defaultGw);

typedef struct {
    // Network stuff
    String hostname;
    String ssid;
    String password;
    IPAddress ip;
    IPAddress gw;
    IPAddress sm;
    IPAddress dns;
    bool useDhcp;

    uint8_t clockTimezone;

    // MQTT stuff
    String mqttTopicStatus;
    String mqttTopicControl;
    String mqttBroker;
    String mqttUsername;
    String mqttPassword;
    uint16_t mqttPort;

    // OTA stuff
    uint16_t otaPort;
    String otaPassword;
} config_t;

#endif