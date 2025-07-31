#ifndef PROM_WIFI_H
#define PROM_WIFI_H

#include <cstdint>
#include <cstddef>

#define USE_STA

#ifdef USE_STA
    #define WIFI_IF WIFI_IF_STA
    #define WIFI_MODE WIFI_MODE_STA
#else
    #define WIFI_IF WIFI_IF_AP
    #define WIFI_MODE WIFI_MODE_AP
#endif

// callback states
#define PW_CALLBACK_DEFAULT -1
#define PW_CALLBACK_INIT 0
#define PW_CALLBACK_SIZEMISMATCH 1
#define PW_CALLBACK_PACKETINFO 2
#define PW_CALLBACK_UPDATEIMAGE 3
#define PW_CALLBACK_TESTMODE 4
#define PW_CALLBACK_PACKETSRECEIVED 5
#define PW_CALLBACK_PACKETSDROPPED 6
#define PW_CALLBACK_RECEIVED 7

void prom_wifi_setDisplayCallback(void (*callbackfunc)(uint16_t *buffer, uint16_t width, uint16_t height, size_t size));
void prom_wifi_setStateCallback(void (*statecallbackfunc)(uint8_t state_type, int16_t value));

void prom_wifi_sniffer_init();

#endif
