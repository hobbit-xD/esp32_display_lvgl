#include "BLEDevice.h"
#include <TFT_eSPI.h>
#include <CST816S.h>
#include <lvgl.h>
#include "lv_conf.h"
#include "fonts/droid_50.h"
#include "fonts/font_awesome_icons_small.h"

#define DEBUG 0

#if DEBUG
#define prt(x) Serial.print(x);
#define prtn(x) Serial.println(x);
#else
#define prt(x)
#define prtn(x)
#endif

#define DEFAULT_PRESSURE 101  // Default atmospheric pressure is 101kPa
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define OIL_SYMBOL "\xEF\x98\x93"
#define WATER_SYMBOL "\xEF\x98\x94"

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("0000fff2-0000-1000-8000-00805f9b34fb");  //char write
static BLEUUID txUUID("0000fff1-0000-1000-8000-00805f9b34fb");    // char read notify

/*Change to your screen resolution*/
static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
//static lv_color_t buf1[screenWidth * screenHeight / 10];
static lv_color_t* buf1 = (lv_color_t*)heap_caps_aligned_alloc(32, (screenWidth * screenHeight * 2) / 6, MALLOC_CAP_DMA);
static lv_color_t* buf2 = (lv_color_t*)heap_caps_aligned_alloc(32, (screenWidth * screenHeight * 2) / 6, MALLOC_CAP_DMA);

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);  // Create object "tft"
CST816S touch(6, 7, 13, 5);                          // sda, scl, rst, irq/int


static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pReadCharacteristic;
static BLEAdvertisedDevice* myDevice;

String imapRequestCommand = "010B\r";     //Intake manifold absolute pressure pid command return value in kPa
String bpRequestCommand = "0133\r";       //Absolute Barometric Pressure return value in kPa
String oilTempRequestCommand = "015C\r";  //Engine oil temperature

unsigned int oilTemperature=0;
unsigned int barometricPressure = DEFAULT_PRESSURE;
unsigned int bp = 0;
unsigned int imap = DEFAULT_PRESSURE;


int intervalloScansione_pressione = 2700000;
unsigned long tempo_trascorso_pressione;

int intervalloScansione_temperatura = 60000;
unsigned long tempo_trascorso_temperatura;

int intervalloScansione = 50;
unsigned long tempo_trascorso_imap;

//LVGL Part
hw_timer_t* timer = nullptr;


lv_obj_t* startup_scr;
lv_obj_t* splash_scr;
lv_obj_t* level_scr;  // Water and oil screen
lv_obj_t* turbo_scr;  //Turbo meter screen

lv_obj_t* oil_arc;    // Oil arc
lv_obj_t* oil_label;  // Oil value display

lv_obj_t* water_arc;    //Water arc
lv_obj_t* water_label;  // Water value display

lv_obj_t* turbo_meter;
lv_meter_indicator_t* turbo_arc_indicator;
lv_meter_indicator_t* turbo_indicator;

bool startup_complete = false;
bool startup_ready = false;
bool complete = false;  // flag for screen changes to prevent recurssion
bool is_levels_mode = false;

// Global styles
static lv_style_t style_unit_text;
static lv_style_t style_track_value_text;
static lv_style_t style_icon;

//Color palette
lv_color_t palette_black = LV_COLOR_MAKE(0, 0, 0);
lv_color_t palette_white = LV_COLOR_MAKE(255, 255, 255);
lv_color_t palette_blue = LV_COLOR_MAKE(48, 190, 252);
lv_color_t palette_grey = LV_COLOR_MAKE(109, 104, 108);
lv_color_t palette_bg = LV_COLOR_MAKE(21, 22, 24);
lv_color_t palette_arc = LV_COLOR_MAKE(15, 16, 18);
lv_color_t palette_red = LV_COLOR_MAKE(255, 0, 0);


float psi_older[5];  // store a few older readings for smoother readout

unsigned int hexToDec(String hexString) {
  prtn("hexToDec function");
  int ArrayLength = hexString.length() + 1;  //The +1 is for the 0x00h Terminator
  char CharArray[ArrayLength];
  hexString.toCharArray(CharArray, ArrayLength);

  return strtoul(CharArray, NULL, 16);
}
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  // Process the received data to extract the speed
  if (length > 0) {
    String response = "";

    for (size_t i = 0; i < length; i++) {
      response += (char)pData[i];
    }
    prtn("Received response: " + response);
    if (response.substring(2, 4) == "5C") {
      //Calculating oil Temperature
      oilTemperature = hexToDec(response.substring(4)) - 40;

    } else if (response.substring(2, 4) == "0B") {
      //Calculating Imap
      imap = hexToDec(response.substring(4));

    } else if (response.substring(2, 4) == "33") {
      //Calculating BarometriPressure
      bp = hexToDec(response.substring(4));
    }
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    prtn("onDisconnect");
  }
};

bool connectToServer() {
  prt("Forming a connection to ");
  prtn(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  prtn(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  prtn(" - Connected to server");
  pClient->setMTU(517);  // set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    prt("Failed to find our service UUID: ");
    prtn(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  prtn(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    prt("Failed to find our characteristic UUID: ");
    prtn(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  prtn(" - Found our write characteristic");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pReadCharacteristic = pRemoteService->getCharacteristic(txUUID);
  if (pReadCharacteristic == nullptr) {
    prt("Failed to find our characteristic UUID: ");
    prtn(txUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  prtn(" - Found our read/notify characteristic");


  // Read the value of the characteristic.
  if (pReadCharacteristic->canRead()) {
    prtn("The characteristic can read");
    //String value = pReadCharacteristic->readValue();
    //prt("The characteristic value was: ");
    //prtn(value.c_str());
  }

  if (pReadCharacteristic->canNotify()) {
    prtn("The characteristic can notify, registering callback: ");
    pReadCharacteristic->registerForNotify(notifyCallback);
  }

  prtn("We are now connected to the BLE Server.");
  prtn("ELM327 initializing..");

  // Initialize ELM327 after successful connection
  if (!initializeELM327()) {
    prtn("Failed to initialize ELM327");
    connected = false;
    return false;
  }
  connected = true;
  prtn("ELM327 initialized successfully");
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    prt("BLE Advertised Device found: ");
    prtn(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      prtn("Device found");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      startup_ready=true;
      doScan = true;
    }
  }
};

bool initializeELM327() {
  // Commands to initialize ELM327
  const char* initCommands[] = {
    "ATD\r",    // set all to default
    "ATZ\r",    // Reset all
    "ATE0\r",   // Echo off
    "ATL0\r",   // Linefeeds off
    "ATS0\r",   // Spaces off
    "ATH0\r",   // Headers off
    "ATSP0\r",  // Auto select protocol
    "ATDP\r"    // Display current protocol
  };

  for (const char* cmd : initCommands) {
    prt("Sending command: ");
    prtn(cmd);
    pRemoteCharacteristic->writeValue(cmd, strlen(cmd));
    delay(1000);  // Wait for response
                  // Might want to read the response here and check for success
    // No response is given for some reason.
  }

  return true;  // Return true if initialization is successful
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp_drv);
}
/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t* indev_drv, lv_indev_data_t* data) {

  //choose display page
  if (touch.available()) {
    if (touch.gesture() == "DOUBLE CLICK") {
      is_levels_mode = !is_levels_mode;
      update_mode();
    }
  }
}

// Global styles
void make_styles(void) {
  lv_style_init(&style_unit_text);
  lv_style_set_text_font(&style_unit_text, &lv_font_montserrat_24);
  lv_style_set_text_color(&style_unit_text, palette_white);

  lv_style_init(&style_track_value_text);
  lv_style_set_text_font(&style_track_value_text, &droid_50);  //ubuntu_100
  lv_style_set_text_color(&style_track_value_text, palette_white);

  lv_style_init(&style_icon);
  lv_style_set_text_font(&style_icon, &font_awesome_icons_small);
  lv_style_set_text_color(&style_icon, palette_white);
}


// Switch between screens
void update_mode(void) {
  if (is_levels_mode) {
    lv_scr_load_anim(level_scr, LV_SCR_LOAD_ANIM_MOVE_TOP, 1000, 0, false);
  } else {
    lv_scr_load_anim(turbo_scr, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 1000, 0, false);
  }
}

void set_turbo_value(void* indic, int32_t v) {
  if (v < 0) {
    lv_meter_set_indicator_start_value(turbo_meter, turbo_arc_indicator, v);  //valore variabile
    lv_meter_set_indicator_end_value(turbo_meter, turbo_arc_indicator, 0);
  } else {
    lv_meter_set_indicator_start_value(turbo_meter, turbo_arc_indicator, 0);  //valore variabile
    lv_meter_set_indicator_end_value(turbo_meter, turbo_arc_indicator, v);
  }
  lv_meter_set_indicator_value(turbo_meter, (lv_meter_indicator_t*)indic, v);
}

void update_oil(int new_oil) {
  char string_oil[4];
  itoa(new_oil, string_oil, 10);
  lv_label_set_text(oil_label, string_oil);
  lv_arc_set_value(oil_arc, new_oil);

  if (new_oil > 100) {
    lv_obj_set_style_arc_color(oil_arc, palette_red, LV_PART_INDICATOR);
  }
}

void update_water(int new_water) {
  char string_water[4];
  itoa(new_water, string_water, 10);
  lv_label_set_text(water_label, string_water);
  lv_arc_set_value(water_arc, new_water);

  if (new_water > 100) {
    lv_obj_set_style_arc_color(water_arc, palette_red, LV_PART_INDICATOR);
  }
}

void make_turbo_meter() {
  turbo_meter = lv_meter_create(turbo_scr);
  lv_obj_set_style_bg_color(turbo_meter, palette_bg, 0);
  lv_obj_set_size(turbo_meter, 240, 240);
  lv_obj_set_style_border_width(turbo_meter, 0, 0);
  lv_obj_center(turbo_meter);
  lv_obj_remove_style(turbo_meter, NULL, LV_PART_INDICATOR);


  lv_meter_scale_t* scale = lv_meter_add_scale(turbo_meter);
  lv_meter_set_scale_ticks(turbo_meter, scale, 46, 2, 5, palette_grey);
  lv_meter_set_scale_major_ticks(turbo_meter, scale, 10, 2, 15, palette_white, 20);
  lv_meter_set_scale_range(turbo_meter, scale, 0, 45, 180, 180);


  lv_meter_scale_t* scale2 = lv_meter_add_scale(turbo_meter);
  lv_meter_set_scale_ticks(turbo_meter, scale2, 16, 0, 0, palette_white);
  lv_meter_set_scale_major_ticks(turbo_meter, scale2, 5, 2, 15, palette_white, 20);
  lv_meter_set_scale_range(turbo_meter, scale2, -15, 0, 60, 120);

  lv_meter_scale_t* main_scale = lv_meter_add_scale(turbo_meter);
  lv_meter_set_scale_ticks(turbo_meter, main_scale, 61, 0, 0, palette_white);
  lv_meter_set_scale_major_ticks(turbo_meter, main_scale, 5, 2, 15, palette_white, -140);
  lv_meter_set_scale_range(turbo_meter, main_scale, -15, 45, 240, 120);


  lv_meter_indicator_t* outline = lv_meter_add_arc(turbo_meter, main_scale, 2, palette_blue, 3);
  lv_meter_set_indicator_start_value(turbo_meter, outline, -17);  //valore variabile
  lv_meter_set_indicator_end_value(turbo_meter, outline, 47);

  turbo_arc_indicator = lv_meter_add_arc(turbo_meter, main_scale, 15, palette_blue, 0);
  //lv_meter_set_indicator_start_value(turbo_meter, arc, 0);  //valore variabile
  //lv_meter_set_indicator_end_value(turbo_meter, arc, 0);

  lv_obj_set_style_text_color(turbo_meter, palette_white, LV_PART_TICKS);

  turbo_indicator = lv_meter_add_needle_line(turbo_meter, main_scale, 6, palette_blue, -15);
  //lv_meter_set_indicator_value(turbo_meter, turbo_indicator, 0);

  lv_obj_t* inner_circle = lv_obj_create(turbo_scr);
  lv_obj_set_size(inner_circle, 50, 50);
  lv_obj_center(inner_circle);
  lv_obj_set_style_bg_color(inner_circle, palette_black, 0);
  lv_obj_set_style_bg_opa(inner_circle, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(inner_circle, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(inner_circle, 1, 0);
  lv_obj_set_style_border_color(inner_circle, palette_blue, 0);
}

void make_oil_arc() {

  oil_arc = lv_arc_create(level_scr);
  lv_obj_set_size(oil_arc, 230, 230);
  lv_arc_set_rotation(oil_arc, 300);
  lv_arc_set_range(oil_arc, 0, 120);
  lv_arc_set_bg_angles(oil_arc, 0, 120);
  lv_arc_set_mode(oil_arc, LV_ARC_MODE_REVERSE);
  lv_obj_center(oil_arc);

  lv_obj_set_style_arc_color(oil_arc, palette_grey, LV_PART_MAIN);
  lv_obj_set_style_arc_color(oil_arc, palette_white, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(oil_arc, 15, LV_PART_MAIN);
  lv_obj_set_style_arc_width(oil_arc, 15, LV_PART_INDICATOR);
  lv_obj_remove_style(oil_arc, NULL, LV_PART_KNOB);

  oil_label = lv_label_create(level_scr);
  lv_label_set_text(oil_label, "0");
  lv_obj_add_style(oil_label, &style_track_value_text, 0);
  lv_obj_align(oil_label, LV_ALIGN_CENTER, 50, 2);

  lv_obj_t* oil_icon = lv_label_create(level_scr);
  lv_label_set_text(oil_icon, OIL_SYMBOL);
  lv_obj_add_style(oil_icon, &style_icon, 0);
  lv_obj_set_style_text_color(oil_icon, palette_white, 0);
  lv_obj_align(oil_icon, LV_ALIGN_CENTER, 50, -40);

  lv_obj_t* oil_unit = lv_label_create(level_scr);
  lv_label_set_text(oil_unit, "°C");
  lv_obj_add_style(oil_unit, &style_unit_text, 0);
  lv_obj_set_style_text_color(oil_unit, palette_white, 0);
  lv_obj_align(oil_unit, LV_ALIGN_CENTER, 50, 40);
}

void make_water_arc() {

  water_arc = lv_arc_create(level_scr);
  lv_obj_set_size(water_arc, 230, 230);
  lv_arc_set_rotation(water_arc, 120);
  lv_arc_set_range(water_arc, 0, 120);
  lv_arc_set_bg_angles(water_arc, 0, 120);
  //lv_arc_set_mode(water_arc, LV_ARC_MODE_REVERSE);
  lv_obj_center(water_arc);

  lv_obj_set_style_arc_color(water_arc, palette_grey, LV_PART_MAIN);
  lv_obj_set_style_arc_color(water_arc, palette_white, LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(water_arc, 15, LV_PART_MAIN);
  lv_obj_set_style_arc_width(water_arc, 15, LV_PART_INDICATOR);
  lv_obj_remove_style(water_arc, NULL, LV_PART_KNOB);

  water_label = lv_label_create(level_scr);
  lv_label_set_text(water_label, "0");
  lv_obj_add_style(water_label, &style_track_value_text, 0);
  lv_obj_align(water_label, LV_ALIGN_CENTER, -50, 2);

  lv_obj_t* water_icon = lv_label_create(level_scr);
  lv_label_set_text(water_icon, WATER_SYMBOL);
  lv_obj_add_style(water_icon, &style_icon, 0);
  lv_obj_set_style_text_color(water_icon, palette_white, 0);
  lv_obj_align(water_icon, LV_ALIGN_CENTER, -50, -40);

  lv_obj_t* water_unit = lv_label_create(level_scr);
  lv_label_set_text(water_unit, "°C");
  lv_obj_add_style(water_unit, &style_unit_text, 0);
  lv_obj_set_style_text_color(water_unit, palette_white, 0);
  lv_obj_align(water_unit, LV_ALIGN_CENTER, -50, 40);
}















void example_increase_lvgl_tick(void* arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}
void make_splash_screen(void) {
  splash_scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(splash_scr, palette_red, 0);
}
void make_turbo_screen() {
  turbo_scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(turbo_scr, palette_black, 0);

  make_turbo_meter();
}

void make_levels_screen() {
  level_scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(level_scr, palette_black, 0);
  make_oil_arc();
  make_water_arc();
}

void make_ui() {
  make_styles();
  make_splash_screen();
  make_turbo_screen();
  make_levels_screen();
}

// if no startup message received, start anyway
void force_splash(lv_timer_t* timer) {
  // avoid refire if complete
  if (!startup_complete) {
    start_splash();
    startup_complete = true;
  }
}


// load the splash screen
void start_splash() {
  lv_scr_load_anim(splash_scr, LV_SCR_LOAD_ANIM_FADE_IN, 1000, 0, false);

  lv_timer_t* exit_timer = lv_timer_create(change_loading_scr, 3500, startup_scr);  // back to blank
  lv_timer_set_repeat_count(exit_timer, 1);
}

void change_loading_scr(lv_timer_t* timer) {
  lv_obj_t* next_scr = (lv_obj_t*)timer->user_data;
  lv_scr_load_anim(next_scr, LV_SCR_LOAD_ANIM_FADE_IN, 1000, 0, true);  // delete startup on exit

  if (!complete) {
    lv_timer_t* exit_timer = lv_timer_create(change_loading_scr, 2000, (is_levels_mode) ? level_scr : turbo_scr);  // back to blank
    lv_timer_set_repeat_count(exit_timer, 1);
    complete = true;
  }
}

void setup() {
  Serial.begin(115200);
  prtn("Starting Arduino BLE Client application...");

  lv_init();

  touch.begin();
  touch.enable_double_click();  // Enable double-click detection
  tft.begin();                  // initialize the display
  tft.setRotation(0);           // set the display rotation, in this rotation, the USB port is on the bottom

  /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, (screenWidth * screenHeight) / 6);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  startup_scr = lv_scr_act();  // Make startup screen active
  lv_obj_set_style_bg_color(startup_scr, palette_black, 0);
  lv_obj_set_style_bg_opa(startup_scr, LV_OPA_COVER, 0);

  // Setup the timer
  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  make_ui();

  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  tempo_trascorso_pressione = lv_tick_get();
  tempo_trascorso_temperatura = lv_tick_get();
  tempo_trascorso_imap = lv_tick_get();

}

void loop() {
  lv_timer_handler();

  // if (startup_ready && !startup_complete) {
  //   //delay(120);
  //   start_splash();
  //   startup_ready = false;
  // }

  if (doConnect == true) {
    if (startup_ready && !startup_complete) {
      start_splash();
      startup_ready = false;
    }

    if (connectToServer()) {
      prtn("We are now connected to the BLE Server and ELM327 is initialized.");
    } else {
      prtn("We have failed to connect to the server or initialize ELM327; there is nothing more we will do.");
    }
    doConnect = false;
    delay(500);
  }

  if (connected) {

    if (!is_levels_mode) {
      // Sending the command to request intake manifold pressure
      if (lv_tick_get() - tempo_trascorso_imap >= intervalloScansione) {
        prtn("Sending Imap request command: " + imapRequestCommand);
        pRemoteCharacteristic->writeValue(imapRequestCommand.c_str(), imapRequestCommand.length());
        tempo_trascorso_imap = lv_tick_get();
      }
      // Sending the command to request barometric pressure
      if ((lv_tick_get() - tempo_trascorso_pressione >= intervalloScansione_pressione) || bp == 0) {

        prtn("Sending Bp request command: " + bpRequestCommand);
        pRemoteCharacteristic->writeValue(bpRequestCommand.c_str(), bpRequestCommand.length());
        tempo_trascorso_pressione = lv_tick_get();
      }

      if (bp != 0) {
        barometricPressure = bp;
      }

      int boost_Kpa = imap - barometricPressure;
      //float boost_bar = boost_Kpa / 100.0;
      float boost_psi = boost_Kpa / 6.895;  //boost_bar * 14.504;

      // store a few older measurements
      for (int i = 0; i < 4; i++) {
        psi_older[i] = psi_older[i + 1];
      }
      psi_older[4] = boost_psi;

      float psi_combined = 0;
      for (int i = 0; i < 5; i++) {
        psi_combined = psi_combined + psi_older[i];
      }
      psi_combined = psi_combined / 5;  // final pressure value is the average of last 5 readings

      //int image_pressure = map(round(psi_combined), -15, 45, 0, 60);
      //image_pressure = constrain(image_pressure, 0, 60);  // restrict the images to 0-60 in case the PSI pressure is outside the range of -15..45

      //tft.pushImage(0, 0, 240, 240, boost_gauges_allArray[image_pressure]);  // draw the fullscreen boost gauge image

      int pressure = constrain(psi_combined, -15, 45);
      set_turbo_value(turbo_indicator, pressure);
    } else {
      if ((lv_tick_get() - tempo_trascorso_temperatura >= intervalloScansione_temperatura) || oilTemperature == 0) {
        prtn("Sending Oil temperature request command: " + oilTempRequestCommand);
        pRemoteCharacteristic->writeValue(oilTempRequestCommand.c_str(), oilTempRequestCommand.length());
        tempo_trascorso_temperatura = lv_tick_get();

        update_water(barometricPressure);
        update_oil(oilTemperature);
      }
    }
  } else if (doScan) {
    BLEDevice::getScan()->start(0);
  }
}
