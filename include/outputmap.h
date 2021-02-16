#ifndef _OUTPUT_MAP_H
#define _OUTPUT_MAP_H

// Pin definitions on the MCP23017
#define GPA0 0
#define GPA1 1
#define GPA2 2
#define GPA3 3
#define GPA4 4
#define GPA5 5
#define GPA6 6
#define GPA7 7
#define GPB0 8
#define GPB1 9
#define GPB2 10
#define GPB3 11
#define GPB4 12
#define GPB5 13
#define GPB6 14
#define GPB7 15

// Light relay addresses
#define LIGHT_1_ON_RELAY GPB1
#define LIGHT_1_OFF_RELAY GPB0
#define LIGHT_2_ON_RELAY GPB2
#define LIGHT_2_OFF_RELAY GPB3
#define LIGHT_3_ON_RELAY GPB4
#define LIGHT_3_OFF_RELAY GPB5
#define LIGHT_4_ON_RELAY GPB6
#define LIGHT_4_OFF_RELAY GPB7
#define LIGHT_5_ON_RELAY GPA1
#define LIGHT_5_OFF_RELAY GPA2

// Channel status LED addresses
#define CHAN_1_LED GPA3
#define CHAN_2_LED GPA4
#define CHAN_3_LED GPA5
#define CHAN_4_LED GPA6
#define CHAN_5_LED GPA7

#endif