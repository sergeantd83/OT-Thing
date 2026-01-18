#ifndef _devstatus_h
#define _devstatus_h

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "freertos/FreeRTOS.h"
#ifdef NODO
inline bool WIRED_ETHERNET_PRESENT, OLED_PRESENT = false;
#endif
extern class DevStatus {
private:
    JsonDocument doc;
#ifdef NODO
    SemaphoreHandle_t xMutex; // Replace std::mutex
#else
    SemaphoreHandle_t mutex;
#endif
public:
    DevStatus();
    void lock();
    void unlock();
    JsonDocument &buildDoc();
    void getJson(String &str);
    uint32_t numWifiDiscon;
} devstatus;

#endif