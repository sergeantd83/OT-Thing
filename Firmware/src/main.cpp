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
//#include <NimBLEDevice.h>

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

void statusLedLoop() {
    static uint16_t mask = 0x8000;

    setLedStatus((statusLedData & mask) != 0);
    mask >>= 1;
    if (!mask)
        mask = 0x8000;
}

void wifiEvent(WiFiEvent_t event) {
    switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: {
        String hn = devconfig.getHostname();
        WiFi.setHostname(hn.c_str());
        MDNS.begin(hn.c_str());
        MDNS.addService("http", "tcp", 80);
        break;
    }

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        devstatus.numWifiDiscon++;
        WiFi.reconnect();
        break;

    default:
        break;
    }
}
/*
class scanCallbacks : public NimBLEScanCallbacks {
    void onDiscovered(const NimBLEAdvertisedDevice* advertisedDevice) override {
        Serial.println("Discovered Device: %s\n", advertisedDevice->toString().c_str());
    }

    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        Serial.println("Device result: %s\n", advertisedDevice->toString().c_str());
    }

    void onScanEnd(const NimBLEScanResults& results, int reason) override {
        Serial.println("Scan ended reason = %d; restarting scan\n", reason);
        NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }
} scanCallbacks;
*/

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

  static unsigned long on_time = 20;

  int64_t c = 0;
  while (xQueueReceive(button_press_queue, &c, 0)) {

    // Skip this press.
    if ( (c - last_c ) < 300000 ){
      Serial.println("Button press ignored\n");
      last_c = c;
      continue;
    }
    last_c = c;
    Serial.println("Button press detected\n");
    if (!pageno) { // display off so turn on
      oled_display.ssd1306_command(SSD1306_DISPLAYON);
      oled_display.setTextSize(1);
      oled_display.setTextColor(SSD1306_WHITE);
      on_time = now;
      pageno = 1;
    } else { // request next page
      on_time = now; // reset timeout 
      pageno++;
      if ( pageno >= number_of_pages ) {
        pageno = 1;
      }
      Serial.printf("Page num: %u\n", pageno);
    }
  } 
  if ( c == 0 ){ // button not pressed
    if ( on_time > 10 && (now-on_time) > 30000 ) { // display timeout
      oled_display.ssd1306_command(SSD1306_DISPLAYOFF);
      on_time = 0;
      pageno = 0;
    }
  }

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
        oled_display.println("Press Boot button for more");
      } // config mode
    } else if ( pageno > 1 ) { 
      devstatus.lock(); 
      // Make a copy so we can unlock devstatus.
      JsonDocument &doc= devstatus.buildDoc();
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
    }
    // --- Memory Status ---
    // oled_display.setCursor(0, 20);
    // %u is for unsigned int, %lu is for long unsigned (safer for 32-bit heap values)
    // snprintf(buffer, sizeof(buffer), "%lu bytes free", (unsigned long)ESP.getFreeHeap());
    // oled_display.println(buffer);
    
    oled_display.display();
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
      oled_display.println("Hold Boot for screen");
      // pretty flame
      oled_display.drawBitmap(56, 34, flame_bitmap, 16, 16, WHITE);
      // Push to display
      oled_display.display();
      OLED_PRESENT = true;
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

    Serial.println("Waiting 10s to get dhcp lease");
    if (Ethernet.begin(ethMac, 10000)) {
      if (Ethernet.linkStatus() == LinkON) { // belt and braces
        Serial.print("DHCP successful! IP address: ");
        Serial.println(Ethernet.localIP());
        WIRED_ETHERNET_PRESENT = true;
      } else {
        Serial.println("Physical link is DOWN. Check cable connection.");
      }
    } else if ( Ethernet.hardwareStatus() == EthernetNoHardware ) {
      Serial.println("Failed to obtain DHCP, no hardware");
    } else {
      Serial.println("Failed to obtain DHCP");
    }
  }
#endif
  otcontrol.begin();

  statusLedTicker.attach(0.2, statusLedLoop);

  // Wifi needs to be initialized.
  WiFi.mode(WIFI_STA);
  // Read out the config from NVS
  wifi_config_t conf;
  esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, &conf);
  // If read is successfull and an ssid is specified we assume its stored
  bool noStoredWifi = (err != ESP_OK || conf.sta.ssid[0] == '\0');

  configMode = (digitalRead(GPIO_CONFIG_BUTTON) == 0) || noStoredWifi;
  if (configMode) {
    statusLedData = 0xA000;
  }

#ifdef NODO
  if (WIRED_ETHERNET_PRESENT == false) {
#endif
    WiFi.onEvent(wifiEvent);
    WiFi.setSleep(false);
    WiFi.begin();
#ifdef NODO
  }
#endif
  OneWireNode::begin();
  haDisc.begin();
  mqtt.begin();
  devconfig.begin();
  configTime(devconfig.getTimezone(), 3600, PSTR("pool.ntp.org"));

  portal.begin(configMode);

  command.begin();
/*
    NimBLEDevice::init("");                         // Initialize the device, you can specify a device name if you want.
    NimBLEScan* pBLEScan = NimBLEDevice::getScan(); // Create the scan object.
    pBLEScan->setScanCallbacks(&scanCallbacks, false); // Set the callback for when devices are discovered, no duplicates.
    pBLEScan->setActiveScan(true);          // Set active scanning, this will get more data from the advertiser.
    pBLEScan->setMaxResults(0);             // Do not store the scan results, use callback only.
    pBLEScan->start(30000UL, false, true); // duration, not a continuation of last scan, restart to get all devices again.
  */  
#ifdef DEBUG
    ArduinoOTA.begin();
#endif
}

void loop() {
    unsigned long now = millis();

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
        return;
    }
    else
        btnDown = now;

#ifdef DEBUG
    ArduinoOTA.handle();
#endif
    portal.loop();
    mqtt.loop();
    otcontrol.loop();
    Sensor::loopAll();
    devconfig.loop();
    OneWireNode::loop();
#ifdef NODO
    static unsigned long lastDisplayUpdate = 0;
    if (now - lastDisplayUpdate > 1000 || uxQueueMessagesWaiting(button_press_queue)) { 
        displayNetworkStatus(now); 
        lastDisplayUpdate = now;
    }
    if (WIRED_ETHERNET_PRESENT) {
        Ethernet.maintain(); /* keep DHCP lease etc */
    }
#endif   
}
