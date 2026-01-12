#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Ticker.h>
#include "hwdef.h"
#include "portal.h"
#include "otcontrol.h"
#include "mqtt.h"
#include "devstatus.h"
#include "devconfig.h"
#include "command.h"
#include "sensors.h"
#include "HADiscLocal.h"
#include <esp_wifi.h>
#include "time.h"
#include "main.h"
#include "esp_task_wdt.h"

#ifdef DEBUG
    #include <ArduinoOTA.h>
#endif

#ifdef NODO
#include <EthernetESP32.h>
SPIClass SPI1(FSPI);
W5500Driver driver(GPIO_SPI_CS, GPIO_SPI_INT, GPIO_SPI_RST);
char buffer[64]; 
byte ethMac[6];
#endif

Ticker statusLedTicker;
volatile uint16_t statusLedData = 0x8000;
bool configMode = false;

#ifdef DEBUG
    NimBLECharacteristic *bleSerialTx;
    volatile bool bleClientConnected = false;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        bleClientConnected = true;
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        bleClientConnected = false;
    }
};
#endif

// Variables for dynamic network management
unsigned long lastNetworkCheck = 0;
const unsigned long netCheckInterval = 5000; // Check physical link every 5 seconds

void statusLedLoop() {
    static uint16_t mask = 0x8000;

    setLedStatus((statusLedData & mask) != 0);
    mask >>= 1;
    if (!mask)
        mask = 0x8000;
}

void wifiEvent(WiFiEvent_t event) {
#ifdef NODO
    if (configMode) return;
#endif
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: {
        String hn = devconfig.getHostname();
        WiFi.setHostname(hn.c_str());
        MDNS.begin(hn.c_str());
        MDNS.addService("http", "tcp", 80);
        break;
    }

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        // Only trigger reconnect if we aren't currently using Wired Ethernet
        if (!WIRED_ETHERNET_PRESENT) {
            devstatus.numWifiDiscon++;
            WiFi.reconnect();
        }
        break;

    default:
        break;
    }
}



#ifdef NODO
QueueHandle_t button_press_queue;
void IRAM_ATTR nodo_boot_button_interrupt(void *arg) {
  if ( gpio_get_level((gpio_num_t)GPIO_BOOT_BUTTON) == 0 ){
    int64_t t = esp_timer_get_time();
    xQueueSendFromISR(button_press_queue, &t, NULL);
  }
}
const int number_of_pages = 5;
int64_t last_c = 0;
int pageno = 0;

Adafruit_SSD1306 oled_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void displayNetworkStatus(unsigned long now) {
  if (!OLED_PRESENT)
      return; // Skip if no display detected

  static unsigned long on_time = 0;
  static int last_pageno_state = 0;

  int64_t c = 0;
  while (xQueueReceive(button_press_queue, &c, 0)) {
    // Debounce check
    if ( (c - last_c ) < 300000 ){
      last_c = c;
      continue;
    }
    last_c = c;
    
    // Cycle pages or turn on if it was off
    if (pageno == 0) {
        pageno = 1;
    } else {
        pageno++;
        if (pageno >= number_of_pages) pageno = 1;
    }
    on_time = now; // Reset timer on manual interaction
  } 

  // --- AUTOMATIC WAKE LOGIC ---
  // If pageno was just set to 1 (by manageNetwork OR button) and it was previously 0
  if (pageno > 0 && last_pageno_state == 0) {
      oled_display.ssd1306_command(SSD1306_DISPLAYON);
      oled_display.setTextSize(1);
      oled_display.setTextColor(SSD1306_WHITE);
      if (on_time == 0) on_time = now; // Only set if not already set by button
  }

  // --- TIMEOUT LOGIC ---
  if ( pageno > 0 && (now - on_time) > 30000 ) {
      oled_display.ssd1306_command(SSD1306_DISPLAYOFF);
      pageno = 0;
      on_time = 0;
  }
  
  last_pageno_state = pageno;

  if (pageno) { // display on, needs updating
    oled_display.clearDisplay();
    oled_display.setCursor(0, 0);
    
    if (pageno==1) {
      if (configMode) { // in config mode
        // Using snprintf to format the string into the buffer
        snprintf(buffer, sizeof(buffer), "SSID: %s", WiFi.softAPSSID().c_str());
        oled_display.println(buffer);
        
        oled_display.setCursor(0, 10);
        snprintf(buffer, sizeof(buffer), "http://%s/", WiFi.softAPIP().toString().c_str());
        oled_display.println(buffer);
      } else { // not config mode
        if (WIRED_ETHERNET_PRESENT) { // wired
            oled_display.println(F("Wired Network"));
            
            oled_display.setCursor(0, 10);
            snprintf(buffer, sizeof(buffer), "IP: %s", Ethernet.localIP().toString().c_str());
            oled_display.println(buffer);
        } else { // wifi
          snprintf(buffer, sizeof(buffer), "WiFi: %s", WiFi.SSID().c_str());
          oled_display.println(buffer);

          oled_display.setCursor(0, 10);
          if (WiFi.isConnected()) {
              snprintf(buffer, sizeof(buffer), "IP: %s", WiFi.localIP().toString().c_str());
              oled_display.println(buffer);
          } else {
              oled_display.println(F("Connecting..."));
          }
        }
        oled_display.setCursor(0,30);
        oled_display.println("Press Boot for more");
      } // config mode
    } else if ( pageno > 1 ) { 
      if (millis() > 5000) { 
          devstatus.lock(); 
          JsonDocument &doc = devstatus.buildDoc();

      if ( pageno == 2 || pageno == 3 ){
        snprintf(buffer, sizeof(buffer), "Heating circuit %d", pageno-1);
        oled_display.println(buffer);
        oled_display.println("");
        JsonArray array = doc["heatercircuit"].as<JsonArray>();
        if ( !array.isNull() && array.size() > (pageno-2) ){
          auto obj = array[pageno-2].as<JsonObject>();
          if ( !obj["roomsetpoint"].isNull() ){
            float rsp = obj["roomsetpoint"];
            snprintf(buffer, sizeof(buffer), "Room setpoint: %.1f", rsp);
            oled_display.println(buffer);
          }
          if ( !obj["roomtemp"].isNull() ){
            float rsp = obj["roomtemp"];
            snprintf(buffer, sizeof(buffer), "Room temp:     %.1f", rsp);
            oled_display.println(buffer);
          }
        } else {
          oled_display.println("Not Found.");
        }
      } else if (pageno == 4 ){
        snprintf(buffer, sizeof(buffer), "Status");
        oled_display.println(buffer);
        oled_display.println("");

        String str = doc["fw_version"];
        snprintf(buffer, sizeof(buffer), "FW version: %s", str.c_str());
        oled_display.println(buffer);
        unsigned ut = doc["runtime"];
        snprintf(buffer, sizeof(buffer), "Runtime:    %u s", ut);
        oled_display.println(buffer);
        if ( !doc["mqtt"].isNull() ){
          snprintf(buffer, sizeof(buffer), "MQTT:       %s", doc["mqtt"]["connected"] == false? "disconnected":"connected");
          oled_display.println(buffer);
          if ( doc["mqtt"]["basetopic"].isNull() ){
            snprintf(buffer, sizeof(buffer), "MQTT topic: n/a");
          } else {
            String str = doc["mqtt"]["basetopic"];
            snprintf(buffer, sizeof(buffer), "MQTT topic: %s", str.c_str());
          }
          oled_display.println(buffer);
        }

      }
      devstatus.unlock();
      } else {
          oled_display.println(F("Loading data..."));
      } 
    }
    // --- Memory Status ---
    // oled_display.setCursor(0, 20);
    // %u is for unsigned int, %lu is for long unsigned (safer for 32-bit heap values)
    // snprintf(buffer, sizeof(buffer), "%lu bytes free", (unsigned long)ESP.getFreeHeap());
    // oled_display.println(buffer);
    
    oled_display.display();
  }
}

// Helper to switch interfaces based on physical link
void manageNetwork() {
    unsigned long now = millis();
    if (now - lastNetworkCheck < netCheckInterval) return;
    lastNetworkCheck = now;

    bool hasLink = (Ethernet.linkStatus() == LinkON);

    // If Ethernet cable just plugged in, but we are currently on WiFi
    if (hasLink && !WIRED_ETHERNET_PRESENT) {
        Serial.println("Ethernet link detected. Switching to Wired...");
        if (!pageno) pageno=1; // wake display
        if (Ethernet.begin(ethMac, 2000)) { 
            WIRED_ETHERNET_PRESENT = true;
            WiFi.disconnect(); // Turn off WiFi to avoid routing conflicts
            Serial.print("Wired IP: ");
            Serial.println(Ethernet.localIP());
            
            String hn = devconfig.getHostname();
            MDNS.begin(hn.c_str());
        }
    } 
    // If Ethernet cable unplugged, but we were using Wired
    else if (!hasLink && WIRED_ETHERNET_PRESENT) {
        if (!pageno) pageno=1; // wake display
        Serial.println("Ethernet link lost. Switching to WiFi...");
        WIRED_ETHERNET_PRESENT = false;
        WiFi.begin(); 
    }
}
#endif // NODO

void setup() {
    Serial.begin();
    Serial.setTxTimeoutMs(100);
    pinMode(GPIO_STATUS_LED, OUTPUT);
    pinMode(GPIO_CONFIG_BUTTON, INPUT);
    
    setLedStatus(false);
#ifdef NODO  // Initialize I2C display and wired ethernet
    const unsigned char flame_bitmap[] PROGMEM = {
        0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x03, 0x80, 0x07, 0x80, 0x0f, 0xc0, 0x1f, 0xe0, 0x3f, 0xf0, 
        0x3f, 0xf0, 0x7f, 0xf8, 0x77, 0xf8, 0x73, 0xf8, 0xff, 0xf8, 0xff, 0xf8, 0x3f, 0xf0, 0x1f, 0xe0
    };
    Wire.begin(GPIO_I2C_SDA, GPIO_I2C_SCL);  // SDA=21, SCL=22 (default ESP32 pins)

    // Initialize OLED
    Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) { // device responded
    if (oled_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      // Clear buffer
      oled_display.clearDisplay();

      // Set text properties
      oled_display.setTextSize(1);             // Text size multiplier
      oled_display.setTextColor(SSD1306_WHITE);
      oled_display.setCursor(0, 0);          // Position on screen

      // Print message
      oled_display.println("Nodo OTGW32 V1.0.0");
      oled_display.setCursor(0, 20);          // Position on screen
      oled_display.println("Press Boot for more");
      oled_display.setCursor(0, 30);          // Position on screen
      oled_display.println("Hold Reset for config");

      // pretty flame
      oled_display.drawBitmap(56, 44, flame_bitmap, 16, 16, WHITE);
      // Push to display
      oled_display.display();
      OLED_PRESENT = true;
      pageno = 1;
    }
  }
  button_press_queue = xQueueCreate(10, sizeof(int64_t));
  gpio_set_direction((gpio_num_t)GPIO_BOOT_BUTTON, GPIO_MODE_INPUT);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_set_intr_type((gpio_num_t)GPIO_BOOT_BUTTON, GPIO_INTR_NEGEDGE);
  gpio_intr_enable((gpio_num_t)GPIO_BOOT_BUTTON);
  gpio_isr_handler_add((gpio_num_t)GPIO_BOOT_BUTTON, nodo_boot_button_interrupt,
                       NULL);
  // Initialize Ethernet
  SPI1.begin(GPIO_SPI_SCK, GPIO_SPI_MISO, GPIO_SPI_MOSI);
  driver.setSPI(SPI1);
  driver.setSpiFreq(10);
  driver.setPhyAddress(0);
  Ethernet.init(driver);
  {
    // Get ESP32 MAC
    uint64_t mac64 = ESP.getEfuseMac();
    for (int i = 0; i < 6; i++) {
      ethMac[5 - i] = (mac64 >> (8 * i)) & 0xFF;
    }
    // Mark as locally administered (set bit 1 of first byte)
    ethMac[0] |= 0x02;

    Serial.printf("Derived W5500 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  ethMac[0], ethMac[1], ethMac[2], ethMac[3], ethMac[4],
                  ethMac[5]);

    // Fast check at boot
    if (Ethernet.begin(ethMac, 1000)) {
      if (Ethernet.linkStatus() == LinkON) {
        WIRED_ETHERNET_PRESENT = true;
      }
    }
  }
#endif
  otcontrol.begin();

  statusLedTicker.attach(0.2, statusLedLoop);

    configMode = digitalRead(GPIO_CONFIG_BUTTON) == 0;
    if (configMode)
        statusLedData = 0xA000;
#ifdef NODO
    if (configMode) {
        WiFi.mode(WIFI_OFF); 
        delay(50);
        WiFi.mode(WIFI_AP); 
    }
#else
    Serial.begin();
    Serial.setTxTimeoutMs(100);
#endif

#ifdef DEBUG
    //auto bleSrv = NimBLEDevice::createServer();
    //auto bleSrvc = bleSrv->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    /*bleSerialTx = bleSrvc->createCharacteristic(
        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
        NIMBLE_PROPERTY::NOTIFY
    );
    auto bleSerialRx = bleSrvc->createCharacteristic(
        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
        NIMBLE_PROPERTY::WRITE
    );
    bleSrvc->start();
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(bleSrvc->getUUID());
    //adv->setScanResponse(true);
    adv->start();*/
#endif
#ifdef NODO
    if (!configMode) {
#endif
        WiFi.onEvent(wifiEvent);
        WiFi.setSleep(false);
        WiFi.begin();
        AddressableSensor::begin();
        OneWireNode::begin();
        BLESensor::begin();
        haDisc.begin();
        mqtt.begin();
#ifdef NODO
    } 
#endif

    // This is now outside the block, fixing the crash in Config Mode
    devconfig.begin(); 
    configTime(devconfig.getTimezone(), 3600, PSTR("pool.ntp.org"));
#ifdef NODO
    devstatus.lock();
    delay(50); // Hold it long enough for the RTOS to register the allocation
    devstatus.unlock();
#endif
    portal.begin(configMode);

#ifdef NODO
    delay(500);
    if (!configMode)
#endif
        command.begin();
#ifdef NODO
    if (!configMode) { // Only enable the watchdog in Normal Mode
#endif
        esp_task_wdt_config_t wdt_config = {
#ifdef NODO
            .timeout_ms = 10000,
            .idle_core_mask = (1 << 0) | (1 << 1), // check both cores
#else
            .timeout_ms = 6000,
            .idle_core_mask = (1 << 0), // check both cores
#endif
            .trigger_panic = true
        };
        esp_task_wdt_init(&wdt_config);
        esp_task_wdt_add(NULL);
#ifdef NODO
    }
#endif
#ifdef DEBUG
    ArduinoOTA.begin();
#endif
}

void loop() {
    unsigned long now = millis();
#ifdef NODO
    yield(); // keep WDT happy
#endif

    static unsigned long btnDown = 0;
    if (digitalRead(GPIO_CONFIG_BUTTON) == 0) {
        if ((now - btnDown) > 10000) {
            statusLedTicker.detach();
            setLedStatus(true);
            devconfig.remove();
            WiFi.persistent(true);
            WiFi.disconnect(true, true);
            while (digitalRead(GPIO_CONFIG_BUTTON) == 0)
                yield();
            ESP.restart();
        }
    }
    else
        btnDown = now;

#ifdef DEBUG
    ArduinoOTA.handle();
#endif
    
#ifdef NODO
    // Check for physical link status and swap if needed
    if (!configMode) 
        manageNetwork();
    else
        delay(1); // keep WDT happy

    static unsigned long lastDisplayUpdate = 0;
    if (now - lastDisplayUpdate > 1000 || uxQueueMessagesWaiting(button_press_queue)) { 
        displayNetworkStatus(now); 
        lastDisplayUpdate = now;
    }

    if (WIRED_ETHERNET_PRESENT) 
        Ethernet.maintain(); /* keep DHCP lease etc */
#endif 

    portal.loop();
    mqtt.loop();
    otcontrol.loop();
    Sensor::loopAll();
    devconfig.loop();
    OneWireNode::loop();

    esp_task_wdt_reset();
}