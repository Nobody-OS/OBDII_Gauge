/*
  CYD_OBD2_Full_withMenu_Final.ino
  - Full integrated sketch:
    * CYD (ESP32-2432S028) / ILI9341 2.8" + XPT2046 Touch
    * BLE (ELM327-style) via Nordic-UART (NUS) UUIDs
    * BLE fallback (works with name-less adapters)
    * Poll OBD PIDs, show live data
    * Webserver AP with /data JSON
    * Touch top buttons switch modes
    * SPORT-Modus: Backlight behavior:
         >=5500 rpm: full on
         >=6500 rpm: blinking
    * Menu (three dots top-right) -> Settings, Web Info, GitHub, Buy me a Coffee
    * Start logo "Nobody OBDII" with shadow + Fade-In
*/

// If your TFT BL pin is not defined in TFT_eSPI User_Setup, uncomment and set it:
// #define TFT_BL 21

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <FS.h>

// Extra colors
#ifndef TFT_GREY
#define TFT_GREY 0x5AEB
#endif
#ifndef TFT_DARKGREY
#define TFT_DARKGREY 0x39E7
#endif

// Touch pins (CYD)
#define TOUCH_CS 33
#define TOUCH_IRQ 36
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_CLK 25

// TFT + touch
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

// BLE (NUS)
static BLEUUID nusServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID nusTXCharUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // write
static BLEUUID nusRXCharUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // notify

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pTXChar = nullptr;
BLERemoteCharacteristic* pRXChar = nullptr;
bool bleConnected = false;
BLEAdvertisedDevice* foundDevice = nullptr;
String bleDeviceName = ""; // optional: set name to target a specific device

// Webserver AP
const char* AP_SSID = "CarDash-ESP32";
const char* AP_PASS = "cardash123";
WebServer server(80);

// Vehicle data
volatile float rpmVal = 0.0;
volatile float speedKph = 0.0;
volatile float coolantTemp = 0.0;
volatile float maf = -1.0;
volatile float fuelRate = -1.0;
String dtcList = "";
int estGear = 0;

// Modes & UI timing
enum Mode { SPORT = 0, ECO = 1, DATA = 2, ERROR = 3 };
Mode currentMode = SPORT;
unsigned long lastUIUpdate = 0;
const unsigned long UI_INTERVAL = 120;

// OBD polling interval
unsigned long lastOBDReq = 0;
const unsigned long OBD_INTERVAL_MS = 900;

// touch calibration
int touch_x_min = 200, touch_x_max = 3800;
int touch_y_min = 200, touch_y_max = 3800;

// Backlight control variables
int blinkState = 0;
unsigned long lastBlink = 0;
const unsigned long BLINK_INTERVAL = 200;

// Menu variables
bool menuActive = false;
int menuSelection = -1;
bool darkMode = true;            // default dark
int displayBrightness = 7;       // 1..10 default 7 (software-only brightness)
const int MENU_ICON_SIZE = 34;   // touch area for three-dot icon

// Forward declarations
void drawTopButtons();
void drawScreen();
void drawSport();
void drawEco();
void drawData();
void drawError();
void checkTouch();
void showLeftStatus(const char* msg, uint32_t bg, uint32_t fg);
void connectWorkflow();
void sendOBDCommand(const char* cmd);
void onBLENotify(BLERemoteCharacteristic* chr, uint8_t* data, size_t length, bool isNotify);
void setupWebServer();
void handleRoot();
void handleJSON();
String parseDTCs(String chunk);
void showAsciiLogo();
void updateBacklight();
uint16_t colorScaled(uint8_t r, uint8_t g, uint8_t b);

// Menu sub-functions
void drawSettings();
void handleSettingsTouch(int x, int y);
void drawWebInfo();
void drawGitHub();
void drawCoffee();

// BLE scan callback with fallback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    String n = advertisedDevice.haveName() ? advertisedDevice.getName().c_str() : "";
    Serial.print("Advertised: "); Serial.println(n.length()? n : "no name");
    if (bleDeviceName.length() == 0 && foundDevice == nullptr) {
      foundDevice = new BLEAdvertisedDevice(advertisedDevice); // first found
      Serial.print("Fallback candidate: "); Serial.println(advertisedDevice.getAddress().toString().c_str());
    } else if (n == bleDeviceName) {
      foundDevice = new BLEAdvertisedDevice(advertisedDevice);
      Serial.print("Matched requested name: "); Serial.println(n);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(120);

  tft.init();
  tft.setRotation(1);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  analogWrite(TFT_BL, 255); // start full
#endif

  ts.begin();
  ts.setRotation(1);

  showAsciiLogo();
  delay(800);

  showLeftStatus("Searching for OBDII device...", TFT_DARKGREY, TFT_WHITE);

  BLEDevice::init("");

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  setupWebServer();

  delay(150);
}

void loop() {
  if (!bleConnected) {
    connectWorkflow();
    return;
  }

  if (bleConnected && millis() - lastOBDReq >= OBD_INTERVAL_MS) {
    lastOBDReq = millis();
    sendOBDCommand("010C"); delay(80); // RPM
    sendOBDCommand("010D"); delay(80); // Speed
    sendOBDCommand("0105"); delay(80); // Coolant
    sendOBDCommand("0110"); delay(80); // MAF
    sendOBDCommand("015E"); delay(80); // Fuel rate
    static int dtcCounter = 0;
    if (++dtcCounter >= 6) { dtcCounter = 0; sendOBDCommand("03"); } // DTC occasionally
  }

  checkTouch();

  if (millis() - lastUIUpdate >= UI_INTERVAL) {
    drawScreen();
    if (!menuActive) updateBacklight();
    lastUIUpdate = millis();
  }

  server.handleClient();

  if (pClient && !pClient->isConnected()) {
    Serial.println("BLE disconnected");
    bleConnected = false;
    if (pClient) { pClient->disconnect(); delete pClient; pClient = nullptr; }
  }

  delay(10);
}

/* ========== ASCII Logo mit Schatten + Fade-In ========== */
void showAsciiLogo() {
  tft.fillScreen(TFT_DARKGREY);
  tft.setTextSize(3);
  int16_t x = 60, y = 100;
  for (int brightness = 0; brightness <= 10; brightness++) {
    tft.fillScreen(TFT_DARKGREY);
    tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
    tft.drawString("Nobody", x + 3, y + 3);
    tft.drawString("OBDII",  x + 3, y + 33);
    uint8_t g = brightness * 25; if (g > 255) g = 255;
    tft.setTextColor(tft.color565(0, g, 0), TFT_DARKGREY);
    tft.drawString("Nobody", x, y);
    tft.drawString("OBDII",  x, y + 30);
    delay(50);
  }
}

/* ========== UI helper: color scaling by displayBrightness (1..10) ========== */
uint16_t colorScaled(uint8_t r, uint8_t g, uint8_t b) {
  float scale = constrain(displayBrightness, 1, 10) / 10.0f;
  uint8_t rn = (uint8_t)min(255, (int)(r * scale));
  uint8_t gn = (uint8_t)min(255, (int)(g * scale));
  uint8_t bn = (uint8_t)min(255, (int)(b * scale));
  return tft.color565(rn, gn, bn);
}

/* ========== draw top menu icon (three dots) ========== */
void drawMenuIcon() {
  int x0 = tft.width() - MENU_ICON_SIZE - 6;
  int y0 = 2;
  uint16_t bg = darkMode ? TFT_DARKGREY : TFT_GREY;
  uint16_t fg = darkMode ? TFT_WHITE : TFT_BLACK;
  tft.fillRect(x0, y0, MENU_ICON_SIZE, 34, bg);
  int cx = x0 + MENU_ICON_SIZE/2;
  int cy = y0 + 10;
  tft.fillCircle(cx - 8, cy, 3, fg);
  tft.fillCircle(cx, cy, 3, fg);
  tft.fillCircle(cx + 8, cy, 3, fg);
}

/* ========== UI ========== */
void showLeftStatus(const char* msg, uint32_t bg, uint32_t fg) {
  tft.fillScreen(bg);
  tft.setTextSize(1);
  tft.setTextColor(fg, bg);
  tft.setCursor(6, 6);
  tft.print(msg);
}

void drawTopButtons() {
  int w = tft.width() / 4;
  const char* labels[] = { "SPORT", "ECO", "DATA", "ERROR" };
  uint16_t selCol = darkMode ? TFT_GREY : TFT_DARKGREY;
  uint16_t offCol = darkMode ? TFT_DARKGREY : TFT_GREY;
  for (int i = 0; i < 4; i++) {
    uint32_t col = (i == (int)currentMode) ? selCol : offCol;
    tft.fillRect(i * w, 0, w - 2, 38, col);
    tft.setCursor(i * w + 6, 6);
    tft.setTextSize(1);
    uint16_t txt = darkMode ? TFT_WHITE : TFT_BLACK;
    tft.setTextColor(txt, col);
    tft.print(labels[i]);
  }
  drawMenuIcon();
}

void drawScreen() {
  uint16_t bgColor = darkMode ? TFT_BLACK : TFT_WHITE;
  tft.fillRect(0, 38, tft.width(), tft.height() - 38, bgColor);
  drawTopButtons();
  if (menuActive) {
    uint16_t panelBg = darkMode ? colorScaled(20,20,20) : colorScaled(230,230,230);
    tft.fillRect(30, 48, tft.width() - 60, tft.height() - 96, panelBg);
    int bx = 40;
    int by = 60;
    int bw = tft.width() - 80;
    int bh = 34;
    uint16_t btnBg = darkMode ? colorScaled(40,40,40) : colorScaled(200,200,200);
    uint16_t btnFg = darkMode ? colorScaled(220,220,220) : colorScaled(20,20,20);

    tft.fillRoundRect(bx, by, bw, bh, 6, btnBg);
    tft.setCursor(bx + 8, by + 8);
    tft.setTextSize(2);
    tft.setTextColor(btnFg, btnBg);
    tft.print("Settings");

    by += (bh + 12);
    tft.fillRoundRect(bx, by, bw, bh, 6, btnBg);
    tft.setCursor(bx + 8, by + 8);
    tft.print("Web Server Info");

    by += (bh + 12);
    tft.fillRoundRect(bx, by, bw, bh, 6, btnBg);
    tft.setCursor(bx + 8, by + 8);
    tft.print("GitHub");

    by += (bh + 12);
    tft.fillRoundRect(bx, by, bw, bh, 6, btnBg);
    tft.setCursor(bx + 8, by + 8);
    tft.print("Buy me a Coffee");

    by += (bh + 12);
    tft.fillRoundRect(bx, by, bw, bh, 6, btnBg);
    tft.setCursor(bx + 8, by + 8);
    tft.print("Close");
  } else {
    switch (currentMode) {
      case SPORT: drawSport(); break;
      case ECO: drawEco(); break;
      case DATA: drawData(); break;
      case ERROR: drawError(); break;
    }
  }
}

/* ========== Touch handling (menu + top buttons) ========== */
void checkTouch() {
  if (!ts.touched()) return;
  TS_Point p = ts.getPoint();
  int x = map(p.x, touch_x_min, touch_x_max, 0, tft.width());
  int y = map(p.y, touch_y_min, touch_y_max, 0, tft.height());

  Serial.print("Touch: "); Serial.print(x); Serial.print(","); Serial.println(y); // debug

  static unsigned long lastTouch = 0;
  if (millis() - lastTouch < 120) return;
  lastTouch = millis();

  if (y < 38) {
    int menuX = tft.width() - MENU_ICON_SIZE - 6;
    if (x >= menuX && x <= menuX + MENU_ICON_SIZE) {
      menuActive = !menuActive;
      menuSelection = -1;
      drawScreen();
      Serial.println("Menu toggled");
      return;
    }
    int idx = x / (tft.width() / 4);
    idx = constrain(idx, 0, 3);
    currentMode = (Mode)idx;
    menuActive = false;
    menuSelection = -1;
    drawScreen();
    delay(150);
    return;
  }

  if (menuActive) {
    int bx = 40;
    int by = 60;
    int bw = tft.width() - 80;
    int bh = 34;
    // Settings
    if (x >= bx && x <= bx + bw && y >= by && y <= by + bh) {
      menuSelection = 0;
      drawSettings();
      return;
    }
    by += (bh + 12);
    // Web Info
    if (x >= bx && x <= bx + bw && y >= by && y <= by + bh) {
      menuSelection = 1;
      drawWebInfo();
      return;
    }
    by += (bh + 12);
    // GitHub
    if (x >= bx && x <= bx + bw && y >= by && y <= by + bh) {
      menuSelection = 2;
      drawGitHub();
      return;
    }
    by += (bh + 12);
    // Buy coffee
    if (x >= bx && x <= bx + bw && y >= by && y <= by + bh) {
      menuSelection = 3;
      drawCoffee();
      return;
    }
    by += (bh + 12);
    // Close
    if (x >= bx && x <= bx + bw && y >= by && y <= by + bh) {
      menuActive = false;
      menuSelection = -1;
      drawScreen();
      return;
    }

    // submenu handling
    if (menuSelection != -1) {
      if (menuSelection == 0) { // settings
        handleSettingsTouch(x, y);
        return;
      } else {
        // info pages: any tap returns to menu
        menuSelection = -1;
        drawScreen();
        return;
      }
    }
    return;
  }

  // not in menu: you could add other interactions here
}

/* ========== Menu sub-draw and handlers ==========
   Settings, Web Info, GitHub, Coffee
*/

void drawSettings() {
  tft.fillScreen(darkMode ? TFT_BLACK : TFT_WHITE);
  uint16_t panelBg = darkMode ? colorScaled(20,20,20) : colorScaled(240,240,240);
  tft.fillRect(20, 48, tft.width() - 40, tft.height() - 96, panelBg);

  int bx = 30;
  int by = 60;
  int bw = tft.width() - 60;
  int bh = 38;
  uint16_t btnBg = darkMode ? colorScaled(40,40,40) : colorScaled(210,210,210);
  uint16_t txt = darkMode ? colorScaled(220,220,220) : colorScaled(20,20,20);

  tft.setTextSize(2);
  tft.setCursor(bx, by - 10);
  tft.setTextColor(txt, panelBg);
  tft.print("Settings");

  // Dark/Light toggle
  tft.fillRoundRect(bx, by + 18, bw, bh, 6, btnBg);
  tft.setCursor(bx + 8, by + 26);
  tft.setTextSize(2);
  tft.setTextColor(txt, btnBg);
  tft.print(darkMode ? "Dark Mode (ON)" : "Light Mode (ON)");

  // Brightness slider
  int sliderY = by + 18 + bh + 16;
  tft.setCursor(bx, sliderY - 10);
  tft.setTextSize(1);
  tft.setTextColor(txt, panelBg);
  tft.print("Display Brightness");

  int sx = bx;
  int sw = (bw - 18) / 10;
  for (int i = 0; i < 10; i++) {
    uint16_t boxCol = (i < displayBrightness) ? colorScaled(0,200,0) : colorScaled(120,120,120);
    tft.fillRect(sx + i*(sw + 2), sliderY + 8, sw, 16, boxCol);
  }

  tft.setCursor(bx, sliderY + 40);
  tft.setTextSize(1);
  tft.setTextColor(txt, panelBg);
  tft.print("Tap dark/light or boxes. Tap outside to return.");
}

void handleSettingsTouch(int x, int y) {
  int bx = 30;
  int by = 60;
  int bw = tft.width() - 60;
  int bh = 38;
  int toggleY = by + 18;

  // Toggle dark/light
  if (x >= bx && x <= bx + bw && y >= toggleY && y <= toggleY + bh) {
    darkMode = !darkMode;
    drawSettings();
    return;
  }

  // Brightness selection
  int sliderY = by + 18 + bh + 16;
  int sw = (bw - 18) / 10;
  int sx = bx;
  for (int i = 0; i < 10; i++) {
    int rx = sx + i*(sw + 2);
    if (x >= rx && x <= rx + sw && y >= sliderY + 8 && y <= sliderY + 8 + 16) {
      displayBrightness = i + 1;
      drawSettings();
      return;
    }
  }

  // tap outside -> back to menu
  menuSelection = -1;
  drawScreen();
}

void drawWebInfo() {
  tft.fillScreen(darkMode ? TFT_BLACK : TFT_WHITE);
  uint16_t panelBg = darkMode ? colorScaled(10,10,10) : colorScaled(250,250,250);
  tft.fillRect(20, 48, tft.width() - 40, tft.height() - 96, panelBg);
  uint16_t txt = darkMode ? colorScaled(220,220,220) : colorScaled(20,20,20);
  tft.setTextSize(2);
  tft.setCursor(36, 64);
  tft.setTextColor(txt, panelBg);
  tft.print("Web Server Info");

  tft.setTextSize(1);
  tft.setCursor(36, 100);
  tft.print("AP SSID:");
  tft.setCursor(140, 100);
  tft.print(AP_SSID);

  tft.setCursor(36, 120);
  tft.print("AP PASS:");
  tft.setCursor(140, 120);
  tft.print(AP_PASS);

  tft.setCursor(36, 140);
  tft.print("AP IP:");
  IPAddress ip = WiFi.softAPIP();
  tft.setCursor(140, 140);
  tft.print(ip.toString());

  tft.setCursor(36, 170);
  tft.print("Tap anywhere to go back.");
}

void drawGitHub() {
  tft.fillScreen(darkMode ? TFT_BLACK : TFT_WHITE);
  uint16_t panelBg = darkMode ? colorScaled(10,10,10) : colorScaled(250,250,250);
  tft.fillRect(20, 48, tft.width() - 40, tft.height() - 96, panelBg);
  uint16_t txt = darkMode ? colorScaled(220,220,220) : colorScaled(20,20,20);
  tft.setTextSize(2);
  tft.setCursor(36, 64);
  tft.setTextColor(txt, panelBg);
  tft.print("GitHub");
  tft.setTextSize(1);
  tft.setCursor(36, 100);
  tft.print("https://github.com/Nobody-OS/");
  tft.setCursor(36, 130);
  tft.print("Tap anywhere to go back.");
}

void drawCoffee() {
  tft.fillScreen(darkMode ? TFT_BLACK : TFT_WHITE);
  uint16_t panelBg = darkMode ? colorScaled(10,10,10) : colorScaled(250,250,250);
  tft.fillRect(20, 48, tft.width() - 40, tft.height() - 96, panelBg);
  uint16_t txt = darkMode ? colorScaled(220,220,220) : colorScaled(20,20,20);
  tft.setTextSize(2);
  tft.setCursor(36, 64);
  tft.setTextColor(txt, panelBg);
  tft.print("Buy me a Coffee");
  tft.setTextSize(1);
  tft.setCursor(36, 100);
  tft.print("Bitcoin wallet:");
  tft.setCursor(36, 120);
  tft.print("bc1qw67c33tt9ychvwat789cjvda9a9cnygdyr2wu0");
  tft.setCursor(36, 150);
  tft.print("Tap anywhere to go back.");
}

/* ========== Mode Drawing ========== */
void drawRPMBar(float rpmv) {
  tft.setTextSize(2);
  tft.setCursor(6, 46);
  uint16_t fg = darkMode ? colorScaled(255,255,255) : colorScaled(0,0,0);
  uint16_t bg = darkMode ? colorScaled(0,0,0) : colorScaled(255,255,255);
  tft.setTextColor(fg, bg);
  char buf[32];
  snprintf(buf, sizeof(buf), "RPM: %.0f", rpmv);
  tft.print(buf);
  int barW = map(min((int)rpmv, 9000), 0, 9000, 0, 250);
  uint16_t rail = darkMode ? colorScaled(40,40,40) : colorScaled(200,200,200);
  tft.fillRect(6, 86, 250, 12, rail);
  uint16_t fillCol = (rpmv > 6000) ? colorScaled(200,0,0) : colorScaled(0,200,0);
  tft.fillRect(6, 86, barW, 12, fillCol);
  tft.drawRect(6, 86, 250, 12, darkMode ? TFT_WHITE : TFT_BLACK);
}

void drawGear(int g) {
  tft.setTextSize(2);
  tft.setCursor(tft.width() - 72, 46);
  uint16_t fg = colorScaled(200,200,0);
  uint16_t bg = darkMode ? colorScaled(0,0,0) : colorScaled(255,255,255);
  tft.setTextColor(fg, bg);
  char buf[12];
  snprintf(buf, sizeof(buf), "G:%d", g);
  tft.print(buf);
}

void drawShiftIndicator(float rpmv) {
  uint16_t color = colorScaled(0,0,0);
  if (currentMode == SPORT) {
    if (rpmv > 6000) color = colorScaled(200,0,0);
  } else if (currentMode == ECO) {
    if (rpmv > 2500) color = colorScaled(200,0,0);
    else if (rpmv < 1500 && speedKph > 5) color = colorScaled(0,0,200);
  }
  tft.fillCircle(tft.width() - 26, 170, 10, color);
}

void drawTempBar(float temp) {
  int x = tft.width() - 34;
  int y = 46;
  int h = 120;
  tft.drawRect(x, y, 20, h, darkMode ? TFT_WHITE : TFT_BLACK);
  int fill = constrain(map((int)temp, 20, 120, 0, h), 0, h);
  int tval = constrain(map((int)temp, 20, 120, 0, 255), 0, 255);
  uint16_t color = colorScaled(tval, 0, 255 - tval);
  if (fill > 2) tft.fillRect(x + 1, y + h - fill + 1, 18, fill - 1, color);
  tft.setCursor(tft.width() - 78, y + h + 6);
  tft.setTextSize(1);
  tft.setTextColor(darkMode ? colorScaled(255,255,255) : colorScaled(0,0,0), darkMode ? colorScaled(0,0,0) : colorScaled(255,255,255));
  char buf[16];
  snprintf(buf, sizeof(buf), "T:%.0f", temp);
  tft.print(buf);
}

void drawSport() {
  drawRPMBar(rpmVal);
  drawGear(estGear);
  drawShiftIndicator(rpmVal);
  drawTempBar(coolantTemp);
}

void drawEco() {
  drawRPMBar(rpmVal);
  drawGear(estGear);
  drawShiftIndicator(rpmVal);
  drawTempBar(coolantTemp);
}

void drawData() {
  tft.setTextSize(1);
  tft.setCursor(6, 46);
  tft.setTextColor(darkMode ? colorScaled(255,255,255) : colorScaled(0,0,0), darkMode ? colorScaled(0,0,0) : colorScaled(255,255,255));
  char buf[64];
  snprintf(buf, sizeof(buf), "RPM: %.0f  SPD: %.0fkm/h  MAF: %.1f  Fuel: %.1f", rpmVal, speedKph, maf, fuelRate);
  tft.print(buf);
  tft.setCursor(6, 66);
  tft.print("DTCs: "); tft.print(dtcList);
}

void drawError() {
  tft.setTextColor(colorScaled(200,0,0), darkMode ? colorScaled(0,0,0) : colorScaled(255,255,255));
  tft.setTextSize(2);
  tft.setCursor(6, 46);
  tft.print("ERROR: No connection");
}

/* ========== BLE / OBD ========== */
void connectWorkflow() {
  // Clear previous foundDevice to rescan fresh
  if (foundDevice) { delete foundDevice; foundDevice = nullptr; }

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setActiveScan(true);
  Serial.println("Starting BLE scan...");
  scan->start(5, false); // 5 seconds scan
  Serial.println("Scan complete");

  if (foundDevice) {
    if (pClient) { pClient->disconnect(); delete pClient; pClient = nullptr; }
    pClient = BLEDevice::createClient();
    Serial.println("Connecting...");
    bool ok = false;
    for (int attempt = 0; attempt < 2; attempt++) {
      if (pClient->connect(foundDevice)) { ok = true; break; }
      delay(200);
    }
    if (!ok) {
      Serial.println("BLE connection failed, will retry scan");
      if (pClient) { pClient->disconnect(); delete pClient; pClient = nullptr; }
      foundDevice = nullptr;
      bleConnected = false;
      return;
    }
    BLERemoteService* pSvc = pClient->getService(nusServiceUUID);
    if (pSvc) {
      pTXChar = pSvc->getCharacteristic(nusTXCharUUID);
      pRXChar = pSvc->getCharacteristic(nusRXCharUUID);
      if (pRXChar && pRXChar->canNotify()) pRXChar->registerForNotify(onBLENotify);
      bleConnected = true;
      Serial.println("BLE Connected!");
      showLeftStatus("BLE Connected!", TFT_BLACK, TFT_GREEN);
    } else {
      Serial.println("Required NUS service not found on device.");
      pClient->disconnect();
      delete pClient;
      pClient = nullptr;
      foundDevice = nullptr;
      bleConnected = false;
    }
  } else {
    Serial.println("No BLE device found, rescanning...");
    delay(500);
  }
}

void sendOBDCommand(const char* cmd) {
  if (pTXChar && bleConnected) {
    std::string c = std::string(cmd) + "\r";
    pTXChar->writeValue(c);
  }
}

void onBLENotify(BLERemoteCharacteristic* chr, uint8_t* data, size_t length, bool isNotify) {
  String resp;
  for (size_t i = 0; i < length; i++) resp += (char)data[i];
  resp.trim();
  if (resp.startsWith("41 0C")) {
    if (resp.length() >= 11) {
      int A = strtol(resp.substring(6,8).c_str(), nullptr, 16);
      int B = strtol(resp.substring(9,11).c_str(), nullptr, 16);
      rpmVal = ((A*256)+B)/4.0f;
    }
  } else if (resp.startsWith("41 0D")) {
    if (resp.length() >= 9) {
      int v = strtol(resp.substring(6,8).c_str(), nullptr, 16);
      speedKph = v;
    }
  } else if (resp.startsWith("41 05")) {
    if (resp.length() >= 9) {
      int v = strtol(resp.substring(6,8).c_str(), nullptr, 16);
      coolantTemp = v - 40;
    }
  } else if (resp.startsWith("41 10")) {
    if (resp.length() >= 11) {
      int A = strtol(resp.substring(6,8).c_str(), nullptr, 16);
      int B = strtol(resp.substring(9,11).c_str(), nullptr, 16);
      maf = ((A*256)+B)/100.0f;
    }
  } else if (resp.startsWith("03")) {
    dtcList = parseDTCs(resp);
  }

  float ratio = rpmVal / ((speedKph > 1.0f) ? speedKph : 1.0f);
  estGear = constrain(round(ratio / 10.0f), 1, 6);
}

String parseDTCs(String chunk) {
  chunk.replace(" ", "");
  chunk.replace("\r", "");
  String dtcStr = "";
  for (int i = 2; i < chunk.length(); i+=4) {
    dtcStr += chunk.substring(i,i+4);
    if (i+4 < chunk.length()) dtcStr += " ";
  }
  return dtcStr;
}

/* ========== Webserver ========== */
void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/data", handleJSON);
  server.begin();
}

void handleRoot() {
  server.send(200, "text/plain", "ESP32 OBD2 Dashboard\nUse /data for JSON");
}

void handleJSON() {
  DynamicJsonDocument doc(512);
  doc["rpm"] = rpmVal;
  doc["speed"] = speedKph;
  doc["coolant"] = coolantTemp;
  doc["maf"] = maf;
  doc["fuel"] = fuelRate;
  doc["gear"] = estGear;
  doc["dtc"] = dtcList;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

/* ========== Backlightsteuerung SPORT-Modus ==========
   Uses TFT_BL if defined in TFT_eSPI or manually set above.
*/
void updateBacklight() {
#ifdef TFT_BL
  if (currentMode == SPORT) {
    if (rpmVal >= 6500) {
      if (millis() - lastBlink > BLINK_INTERVAL) {
        blinkState = !blinkState;
        lastBlink = millis();
      }
      analogWrite(TFT_BL, blinkState ? 255 : 0);
    } else if (rpmVal >= 5500) {
      analogWrite(TFT_BL, 255);
    } else {
      analogWrite(TFT_BL, 150);
    }
  } else {
    analogWrite(TFT_BL, 150);
  }
#endif
}

