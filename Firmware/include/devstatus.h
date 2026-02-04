#ifndef _devstatus_h
#define _devstatus_h

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "util.h"
#ifdef NODO
inline bool WIRED_ETHERNET_PRESENT, OLED_PRESENT = false;
#endif
extern class DevStatus {
friend class DevStatusLock;
private:
    JsonDocument doc;
    SemaphoreHandle_t mutex;
public:
    DevStatus();
    bool lock();
    void unlock();
    void buildDoc(JsonDocument &doc);
    uint32_t numWifiDiscon;
} devstatus;

#endif