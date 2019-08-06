#include "TelemetryHelper.h"
#include "PubSubClient.h"

String TelemetryHelper::getMqttStateDesc(int state) {
    String desc = "";
    switch(state) {
        case MQTT_CONNECTION_TIMEOUT:
            desc = "Connection timed out. No response from server";
            break;
        case MQTT_CONNECTION_LOST:
            desc = "Connection lost.";
            break;
        case MQTT_CONNECT_FAILED:
            desc = "Network connection failed.";
            break;
        case MQTT_DISCONNECTED:
            desc = "Client disconnected cleanly.";
            break;
        case MQTT_CONNECTED:
            desc = "Client connected.";
            break;
        case MQTT_CONNECT_BAD_PROTOCOL:
            desc = "Bad protocol. Unsupported version.";
            break;
        case MQTT_CONNECT_BAD_CLIENT_ID:
            desc = "Server rejected client ID.";
            break;
        case MQTT_CONNECT_UNAVAILABLE:
            desc = "Server unavailable.";
            break;
        case MQTT_CONNECT_BAD_CREDENTIALS:
            desc = "Bad username or password.";
            break;
        case MQTT_CONNECT_UNAUTHORIZED:
            desc = "Client not authorized.";
            break;
        default:
            desc = "Unknown MQTT status code: " + String(state);
            break;
    }
    return desc;
}

String TelemetryHelper::getSystemStateString(SystemState state) {
    String sysState = "normal";
    switch(state) {
        case SystemState::BOOTING:
            sysState = "booting";
            break;
        case SystemState::DISABLED:
            sysState = "disabled";
            break;
        case SystemState::UPDATING:
            sysState = "updating";
            break;
        case SystemState::RECONNECTING:
            sysState = "reconnecting";
            break;
        case SystemState::CHECKIN:
            sysState = "MQTT check-in";
            break;
        case SystemState::NORMAL:
        default:
            break;
    }
    return sysState;
}
