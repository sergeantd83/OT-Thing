#ifndef _portal_h
#define _portal_h

#include <Arduino.h>
#include "util.h"

class Portal {
friend class WebSrvSem;
private:
    String confBuf;
    bool reboot;
    bool updateEnable;
    SemaphoreHandle_t mutex;
    bool checkUpdate {false};
    bool doUpdate {false};
public:
    Portal();
    void begin(bool configMode);
    void loop();
    void textAll(String text);
};

extern Portal portal;
#ifdef NODO
extern const IPAddress apAddress;
extern const IPAddress apMask;
#endif
#endif