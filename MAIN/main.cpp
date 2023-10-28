/**
 * @file      factory.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xinyuan Electronic Technology Co., Ltd
 * @date      2023-04-05
 * @note      Arduino Setting
 *            Tools ->
 *                  Board:"ESP32S3 Dev Module"
 *                  USB CDC On Boot:"Enable"
 *                  USB DFU On Boot:"Disable"
 *                  Flash Size : "16MB(128Mb)"
 *                  Flash Mode"QIO 80MHz
 *                  Partition Scheme:"16M Flash(3M APP/9.9MB FATFS)"
 *                  PSRAM:"OPI PSRAM"
 *                  Upload Mode:"UART0/Hardware CDC"
 *                  USB Mode:"Hardware CDC and JTAG"
 */
#define ENABLE_PLAYER
#define ENABLE_IR_SENDER
#include <Arduino.h>
#include <LilyGoLib.h>
#include <LV_Helper.h>
//#include <WiFi.h>
#ifdef ENABLE_IR_SENDER
#include <IRsend.h>
IRsend irsend(BOARD_IR_PIN);
#endif
#include <driver/i2s.h>
#include <esp_vad.h>

#include <math.h>
#include <cmath>

#ifdef ENABLE_PLAYER
#include <AudioFileSourcePROGMEM.h>
#include <AudioFileSourceID3.h>
#include <AudioGeneratorMP3.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <AudioFileSourceSPIFFS.h>

//ESP-NOW//
#include <esp_now.h>  
#include <esp_wifi.h> // if hibernating
#include <WiFi.h>  

//Autopilot UI//
//#include "app/autopilot/ui.h"
#include "app/autopilot/ui_helpers.h"
//#include "app/autopilot/ui_events.h"


AudioGeneratorMP3       *mp3;
AudioFileSourcePROGMEM  *file;
AudioOutputI2S          *out;
AudioFileSourceID3      *id3;
AudioGeneratorWAV       *wav;
AudioFileSourceSPIFFS   *file_fs;

extern const uint8_t boot_music[4365];
#endif

#define LVGL_MESSAGE_PROGRESS_CHANGED_ID        (88)
#define DEFAULT_RECORD_FILENAME                 "/rec.wav"
#define AUDIO_DATA                              boot_music
#define RADIO_TRANSMIT_PAGE_ID                  9
#define WIFI_SCAN_PAGE_ID                       8
#define MIC_IR_PAGE_ID                          11

#define DEFAULT_SCREEN_TIMEOUT                  15*1000
#define DEFAULT_COLOR                           (lv_color_make(252, 218, 72))
#define VAD_FRAME_LENGTH_MS                     30
#define VAD_BUFFER_LENGTH                       (VAD_FRAME_LENGTH_MS * MIC_I2S_SAMPLE_RATE / 1000)



// Set your new MAC Address for THIS DEVICE
//uint8_t newMACAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x64};

// REPLACE WITH THE MAC Address of your receiver (ESP_NOW)

//BRUTE FORCE MAC ADDRESS TEST
//uint8_t broadcastAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x66};
//uint8_t peerAddress[] = {0xC0, 0x49, 0xEF, 0x65, 0x2D, 0xE4};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


// Define variables to store incoming readings
//float incomingHeading;
float incomingHTS;  // make into an int to remove decimal??
float APState;
//boolean requestHTS = 0; // need to setup on pilot code

// Define command payload to be sent to the autopilot 
uint8_t APbutton; 

  //  char heading[10];
char heading_to_steer[10];

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int doit;
    float HTS;
    float STATE;
} struct_message;

// Create a struct_message called APCommand to hold command to send
struct_message APCommand;
// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;


LV_IMG_DECLARE(clock_face);
LV_IMG_DECLARE(clock_hour_hand);
LV_IMG_DECLARE(clock_minute_hand);
LV_IMG_DECLARE(clock_second_hand);

LV_IMG_DECLARE(radiiwatchface_dial);
LV_IMG_DECLARE(radiiwatchface_sec);
LV_IMG_DECLARE(battery);
LV_IMG_DECLARE(battery2);
LV_IMG_DECLARE(raddiblank);
LV_IMG_DECLARE(mindot);
LV_IMG_DECLARE(mindot2);
LV_IMG_DECLARE(daydot);
LV_IMG_DECLARE(daydotc);
LV_IMG_DECLARE(centerdial);


LV_IMG_DECLARE(lcd_watchface_dial);

LV_IMG_DECLARE(watch_if);
LV_IMG_DECLARE(watch_bg);
LV_IMG_DECLARE(watch_if_hour);
LV_IMG_DECLARE(watch_if_min);
LV_IMG_DECLARE(watch_if_sec);

LV_IMG_DECLARE(watch_if_bg2);
LV_IMG_DECLARE(watch_if_hour2);
LV_IMG_DECLARE(watch_if_min2);
LV_IMG_DECLARE(watch_if_sec2);

LV_FONT_DECLARE(font_siegra);
LV_FONT_DECLARE(font_sandbox);
LV_FONT_DECLARE(font_jetBrainsMono);
LV_FONT_DECLARE(font_firacode_60);
LV_FONT_DECLARE(font_ubuntu_18);    // jeff corrected from "ununtu"
LV_FONT_DECLARE(lv_font_montserrat_28);
LV_FONT_DECLARE(lv_font_montserrat_16);
LV_FONT_DECLARE(lv_font_montserrat_22);
LV_FONT_DECLARE(lv_font_montserrat_48);
LV_FONT_DECLARE(lv_font_montserrat_20);
LV_FONT_DECLARE(lv_font_montserrat_36);
LV_FONT_DECLARE(LCD_12px);
LV_FONT_DECLARE(LCD_16px);
LV_FONT_DECLARE(LCD_32px);
LV_FONT_DECLARE(LCD_48px);
LV_FONT_DECLARE(LCD_72px);

LV_IMG_DECLARE(img_usb_plug);

LV_IMG_DECLARE(charge_done_battery);

LV_IMG_DECLARE(watch_if_5);
LV_IMG_DECLARE(watch_if_6);
LV_IMG_DECLARE(watch_if_8);



static lv_obj_t *battery_percent;
static lv_obj_t *battery_percent2;
static lv_obj_t *battery_percent3;
static lv_obj_t *weather_celsius;
static lv_obj_t *step_counter;
static lv_obj_t *radstep_counter;
static lv_obj_t *lcdstep_counter;

static lv_obj_t *hour_img;
static lv_obj_t *min_img;
static lv_obj_t *sec_img;
static lv_obj_t *batt_img;
static lv_obj_t *batt2_img;

static lv_obj_t *tileview;
static lv_obj_t *radio_ta;
static lv_obj_t *wifi_table_list;
static lv_obj_t *label_datetime;

static lv_obj_t *label_date_m;
static lv_obj_t *label_date_h;
static lv_obj_t *label_date_s;
static lv_obj_t *label_date_a;
static lv_obj_t *label_date_d;

static lv_obj_t *charge_cont;

static lv_timer_t *transmitTask;
static lv_timer_t *clockTimer;
static TaskHandle_t playerTaskHandler;
static TaskHandle_t vadTaskHandler;

static lv_style_t button_default_style;
static lv_style_t button_press_style;
// Save the ID of the current page
static uint8_t pageId = 0;
// Flag used to indicate whether to use light sleep, currently unavailable
static bool lightSleep = false;
// Flag used for acceleration interrupt status
static bool sportsIrq = false;
// Flag used to indicate whether recording is enabled
static bool recordFlag = false;
// Flag used for PMU interrupt trigger status
static bool pmuIrq = false;
// Flag used to select whether to turn off the screen
static bool canScreenOff = true;
// Flag used to detect USB insertion status
static bool usbPlugIn = false;
// Flag to indicate that a packet was sent or received
static bool radioTransmitFlag = false;
// Save transmission states between loops
static int transmissionState = RADIOLIB_ERR_NONE;
// Flag to indicate transmission or reception state
static bool transmitFlag = false;
// Save pedometer steps
static uint32_t stepCounter = 0;
// Save Radio Transmit Interval
static uint32_t configTransmitInterval = 0;
// Save brightness value
static RTC_DATA_ATTR int brightnessLevel = 0;
// Vad detecte values
static int16_t *vad_buff = NULL;
static vad_handle_t vad_inst = NULL;
static bool vad_detect = false;
const size_t vad_buffer_size = VAD_BUFFER_LENGTH * sizeof(short);
static lv_obj_t *vad_btn_label;
static lv_obj_t *vad_btn;
static uint32_t vad_detected_counter = 0;

typedef  struct _lv_datetime {
    lv_obj_t *obj;
    const char *name;
    uint16_t minVal;
    uint16_t maxVal;
    uint16_t defaultVal;
    uint8_t digitFormat;
} lv_datetime_t;

static lv_datetime_t lv_datetime [] = {
    {NULL, "Year", 2023, 2099, 2023, 4},
    {NULL, "Mon", 1, 12, 4, 2},
    {NULL, "Day", 1, 30, 12, 2},
    {NULL, "Hour", 0, 24, 22, 2},
    {NULL, "Min", 0, 59, 30, 2},
    {NULL, "Sec", 0, 59, 0, 2}
};

typedef bool (*player_cb_t)(void);
static player_cb_t player_task_cb = NULL;
static bool playWAV();
static bool playMP3();

void factory_ui();

void productPinmap(lv_obj_t *parent);
void analogclock(lv_obj_t *parent);
void analogclock2(lv_obj_t *parent);
void analogclock3(lv_obj_t *parent);
void radiiclock(lv_obj_t *parent);
void lcarsclock(lv_obj_t *parent);
void lcdclock(lv_obj_t *parent);
void digitalClock(lv_obj_t *parent);
void digitalClock2(lv_obj_t *parent);


void devicesInformation(lv_obj_t *parent);
void wifiscan(lv_obj_t *parent);
void musicPlay(lv_obj_t *parent);
void radioPingPong(lv_obj_t *parent);
void irRemoteVeiw(lv_obj_t *parent);
void datetimeVeiw(lv_obj_t *parent);
void lilygo_qrcode(lv_obj_t *parent);

//ESP-NOW Elements
void espnowinit();
void OnDataSent(const uint8_t *, esp_now_send_status_t);
void OnDataRecv(const uint8_t *, const uint8_t *, int);


//Autopilot elements
void autopilot(lv_obj_t *parent);
void callautopilot();
void autopilot_main_update_label();

void ToggleON(lv_event_t * e);
void ToggleOFF(lv_event_t * e);
void down10button(lv_event_t * e);
void down1button(lv_event_t * e);
void up1button(lv_event_t * e);
void up10button(lv_event_t * e);
void connectbutton(lv_event_t * e);


void ui_init(void);
void ui_APFace_screen_init(void);
lv_obj_t *ui_APFace;
void ui_event_toggle( lv_event_t * e);
static lv_obj_t *ui_toggle;
static lv_obj_t *ui_Togglelabel;
static lv_obj_t *ui_CTSlabel;
static lv_obj_t *ui_CTSfield;
void ui_event_down10( lv_event_t * e);
static lv_obj_t *ui_down10;
static lv_obj_t *ui_Label1;
void ui_event_down1( lv_event_t * e);
static lv_obj_t *ui_down1;
static lv_obj_t *ui_Label2;
void ui_event_up1( lv_event_t * e);
static lv_obj_t *ui_up1;
static lv_obj_t *ui_Label3;
void ui_event_up10( lv_event_t * e);
static lv_obj_t *ui_up10;
static lv_obj_t *ui_Label4;
static lv_obj_t *ui____initial_actions0;
void ui_event_connect(lv_event_t *e);
static lv_obj_t *ui_connect;


void settingPMU();
void settingSensor();
void settingRadio();
void settingPlayer();
void settingIRRemote();
void settingButtonStyle();
void PMUHandler();
void lowPowerEnergyHandler();
void destoryChargeUI();

void radioTask(lv_timer_t *parent);
static void PDM_Record(const char *song_name, uint32_t duration);
static bool CreateWAV(const char *song_name, uint32_t duration, uint16_t num_channels, const uint32_t sampling_rate, uint16_t bits_per_sample);

void WiFiScanDone(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("WiFiScanDone");
    if (pageId != WIFI_SCAN_PAGE_ID) {
        canScreenOff = true;
        WiFi.removeEvent(WiFiEvent_t::ARDUINO_EVENT_WIFI_SCAN_DONE);
        WiFi.mode(WIFI_OFF);
        return;
    }

    int16_t counter =  WiFi.scanComplete();

    lv_table_set_row_cnt(wifi_table_list, counter);

    for (int i = 0; i < counter; ++i) {

        lv_table_set_cell_value_fmt(wifi_table_list, i, 0,
                                    LV_SYMBOL_WIFI"[%4d]"" %-10.10s",
                                    WiFi.RSSI(i),
                                    WiFi.SSID(i).c_str()
                                   );

        // Print SSID and RSSI for each network found
        Serial.printf("%2d", i + 1);
        Serial.print(" | ");
        Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
        Serial.print(" | ");
        Serial.printf("%4d", WiFi.RSSI(i));
        Serial.print(" | ");
        Serial.printf("%2d", WiFi.channel(i));
        Serial.print(" | ");
        switch (WiFi.encryptionType(i)) {
        case WIFI_AUTH_OPEN:
            Serial.print("open");
            break;
        case WIFI_AUTH_WEP:
            Serial.print("WEP");
            break;
        case WIFI_AUTH_WPA_PSK:
            Serial.print("WPA");
            break;
        case WIFI_AUTH_WPA2_PSK:
            Serial.print("WPA2");
            break;
        case WIFI_AUTH_WPA_WPA2_PSK:
            Serial.print("WPA+WPA2");
            break;
        case WIFI_AUTH_WPA2_ENTERPRISE:
            Serial.print("WPA2-EAP");
            break;
        case WIFI_AUTH_WPA3_PSK:
            Serial.print("WPA3");
            break;
        case WIFI_AUTH_WPA2_WPA3_PSK:
            Serial.print("WPA2+WPA3");
            break;
        case WIFI_AUTH_WAPI_PSK:
            Serial.print("WAPI");
            break;
        default:
            Serial.print("unknown");
        }
        Serial.println();
    }
    //Keep scan
    WiFi.scanNetworks(true);
    canScreenOff = false;
}

void setup()
{
      // ESP32 Board add-on after version > 1.0.5
  //esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
    
   //      WiFi.mode(WIFI_STA);

     //Stop wifi
   // WiFi.mode(WIFI_OFF);

    btStop();

  //  setCpuFrequencyMhz(160);

 setCpuFrequencyMhz(80);

    Serial.begin(115200);

    watch.begin();

    watch.initMicrophone();

    settingPMU();

    settingSensor();

    settingRadio();

    settingPlayer();

    settingIRRemote();

    beginLvglHelper(false);

    settingButtonStyle();

    factory_ui();

    usbPlugIn =  watch.isVbusIn();

    espnowinit();
   // WiFi.mode(WIFI_OFF);
}

static void charge_anim_cb(void *obj, int32_t v)
{
    lv_obj_t *arc = (lv_obj_t *)obj;
    static uint32_t last_check_inteval;
    static int battery_percent;
    if (last_check_inteval < millis()) {
        battery_percent =  watch.getBatteryPercent();
        lv_obj_t *label_percent =  (lv_obj_t *)lv_obj_get_user_data(arc);
        lv_label_set_text_fmt(label_percent, "%d%%", battery_percent);
        if (battery_percent == 100) {
            lv_obj_t *img =  (lv_obj_t *)lv_obj_get_user_data(label_percent);
            lv_anim_del(arc, charge_anim_cb);
            lv_arc_set_value(arc, 100);
            lv_img_set_src(img, &charge_done_battery);
        }
        last_check_inteval = millis() + 2000;
    }
    if (v >= battery_percent) {
        return;
    }
    lv_arc_set_value(arc, v);
}


void createChargeUI()
{
    if (charge_cont) {
        return;
    }

    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_100);
    lv_style_set_bg_color(&cont_style, lv_color_black());
    lv_style_set_radius(&cont_style, 0);
    lv_style_set_border_width(&cont_style, 0);

    charge_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(charge_cont, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_add_style(charge_cont, &cont_style, LV_PART_MAIN);
    lv_obj_center(charge_cont);

    lv_obj_add_event_cb(charge_cont, [](lv_event_t *e) {
        destoryChargeUI();
    }, LV_EVENT_PRESSED, NULL);

    int battery_percent =  watch.getBatteryPercent();
    static int last_battery_percent = 0;

    lv_obj_t *arc = lv_arc_create(charge_cont);
    lv_obj_set_size(arc, LV_PCT(90), LV_PCT(90));
    lv_arc_set_rotation(arc, 0);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_obj_set_style_arc_color(arc, lv_color_make(19, 161, 14), LV_PART_INDICATOR);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    lv_obj_center(arc);

    lv_obj_t *img_chg = lv_img_create(charge_cont);
    lv_obj_set_style_bg_opa(img_chg, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_img_recolor(img_chg, lv_color_make(19, 161, 14), LV_PART_ANY);

    lv_obj_t *label_percent = lv_label_create(charge_cont);
    lv_obj_set_style_text_font(label_percent, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_percent, lv_color_white(), LV_PART_MAIN);
    lv_label_set_text_fmt(label_percent, "%d%%", battery_percent);

    //set user data
    lv_obj_set_user_data(arc, label_percent);
    lv_obj_set_user_data(label_percent, img_chg);

    lv_img_set_src(img_chg, &charge_done_battery);

    if (battery_percent == 100) {
        lv_arc_set_value(arc, 100);
        lv_img_set_src(img_chg, &charge_done_battery);
    } else {
        lv_img_set_src(img_chg, &img_usb_plug);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, arc);
        lv_anim_set_start_cb(&a, [](lv_anim_t *a) {
            lv_obj_t *arc = (lv_obj_t *)a->var;
            lv_arc_set_value(arc, 0);
        });

        lv_anim_set_exec_cb(&a, charge_anim_cb);
        lv_anim_set_time(&a, 1000);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_repeat_delay(&a, 500);
        lv_anim_set_values(&a, 0, 100);
        lv_anim_start(&a);
    }
    lv_obj_center(img_chg);
    lv_obj_align_to(label_percent, img_chg, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);


    lv_task_handler();
}


void destoryChargeUI()
{
    if (!charge_cont) {
        return;
    }
    lv_obj_del(charge_cont);
    charge_cont = NULL;
}


void PMUHandler()
{
    if (pmuIrq) {
        pmuIrq = false;
        watch.readPMU();
        if (watch.isVbusInsertIrq()) {
            Serial.println("isVbusInsert");
            createChargeUI();
            watch.incrementalBrightness(brightnessLevel);
            usbPlugIn = true;
        }
        if (watch.isVbusRemoveIrq()) {
            Serial.println("isVbusRemove");
            destoryChargeUI();
            watch.incrementalBrightness(brightnessLevel);
            usbPlugIn = false;
        }
        if (watch.isBatChagerDoneIrq()) {
            Serial.println("isBatChagerDone");
        }
        if (watch.isBatChagerStartIrq()) {
            Serial.println("isBatChagerStart");
        }
        // Clear watch Interrupt Status Register
        watch.clearPMU();
    }
}

void SensorHandler()
{
    if (sportsIrq) {
        sportsIrq = false;
        // The interrupt status must be read after an interrupt is detected
        uint16_t status = watch.readBMA();
        Serial.printf("Accelerometer interrupt mask : 0x%x\n", status);

        if (watch.isPedometer()) {
            stepCounter = watch.getPedometerCounter();
            Serial.printf("Step count interrupt,step Counter:%u\n", stepCounter);
        }
        if (watch.isActivity()) {
            Serial.println("Activity interrupt");
        }
        if (watch.isTilt()) {
            Serial.println("Tilt interrupt");
        }
        if (watch.isDoubleTap()) {
            Serial.println("DoubleTap interrupt");
        }
        if (watch.isAnyNoMotion()) {
            Serial.println("Any motion / no motion interrupt");
        }
    }
}


void lowPowerEnergyHandler()
{
    Serial.println("Enter light sleep mode!");
    brightnessLevel = watch.getBrightness();
    watch.decrementBrightness(0);

    watch.clearPMU();

    watch.configreFeatureInterrupt(
        SensorBMA423::INT_STEP_CNTR |   // Pedometer interrupt
        SensorBMA423::INT_ACTIVITY |    // Activity interruption
        SensorBMA423::INT_TILT |        // Tilt interrupt
        // SensorBMA423::INT_WAKEUP |      // DoubleTap interrupt
        SensorBMA423::INT_ANY_NO_MOTION,// Any  motion / no motion interrupt
        false);

    sportsIrq = false;
    pmuIrq = false;
    lv_timer_pause(transmitTask);
    //TODO: Low power consumption not debugged
    if (lightSleep) {
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        // esp_sleep_enable_ext1_wakeup(1ULL << BOARD_BMA423_INT1, ESP_EXT1_WAKEUP_ANY_HIGH);
        // esp_sleep_enable_ext1_wakeup(1ULL << BOARD_PMU_INT, ESP_EXT1_WAKEUP_ALL_LOW);

        gpio_wakeup_enable ((gpio_num_t)BOARD_PMU_INT, GPIO_INTR_LOW_LEVEL);
        gpio_wakeup_enable ((gpio_num_t)BOARD_BMA423_INT1, GPIO_INTR_HIGH_LEVEL);
        esp_sleep_enable_gpio_wakeup ();
        esp_light_sleep_start();
    } else {

//experiment
//esp_wifi_stop();
WiFi.mode(WIFI_OFF);

        setCpuFrequencyMhz(10);
        // setCpuFrequencyMhz(80);
        while (!pmuIrq && !sportsIrq && !watch.getTouched()) {
            delay(300);
            // gpio_wakeup_enable ((gpio_num_t)BOARD_TOUCH_INT, GPIO_INTR_LOW_LEVEL);
            // esp_sleep_enable_timer_wakeup(3 * 1000);
            // esp_light_sleep_start();
        }

        //setCpuFrequencyMhz(240);  
                setCpuFrequencyMhz(80); //experiment
    }

    // Clear Interrupts in Loop
    // watch.readBMA();
    // watch.clearPMU();

    watch.configreFeatureInterrupt(
        SensorBMA423::INT_STEP_CNTR |   // Pedometer interrupt
        SensorBMA423::INT_ACTIVITY |    // Activity interruption
        SensorBMA423::INT_TILT |        // Tilt interrupt
        // SensorBMA423::INT_WAKEUP |      // DoubleTap interrupt
        SensorBMA423::INT_ANY_NO_MOTION,// Any  motion / no motion interrupt
        true);


    lv_timer_resume(transmitTask);

    lv_disp_trig_activity(NULL);
    // Run once
    lv_task_handler();

    watch.incrementalBrightness(brightnessLevel);
    
    //experiment
  // if (pageId ==7) 
  WiFi.mode(WIFI_STA);

}

void loop()
{
    SensorHandler();

    PMUHandler();

    if (recordFlag) {
        recordFlag = false;
        PDM_Record(DEFAULT_RECORD_FILENAME, 8);
        canScreenOff = true;
        lv_disp_trig_activity(NULL);
    }

    bool screenTimeout = lv_disp_get_inactive_time(NULL) < DEFAULT_SCREEN_TIMEOUT;
    if (screenTimeout ||
            !canScreenOff ||
            usbPlugIn) {
        if (!screenTimeout) {
            if (usbPlugIn &&
                    (pageId != WIFI_SCAN_PAGE_ID &&
                     pageId != RADIO_TRANSMIT_PAGE_ID)
               ) {
                createChargeUI();
            }
            lv_disp_trig_activity(NULL);
        }
        lv_task_handler();
        delay(2);
    } else {
        lowPowerEnergyHandler();
    }

}

void tileview_change_cb(lv_event_t *e)
{
    static uint16_t lastPageID = 0;
    lv_obj_t *tileview = lv_event_get_target(e);
    pageId = lv_obj_get_index(lv_tileview_get_tile_act(tileview));
    lv_event_code_t c = lv_event_get_code(e);
    Serial.print("Code : ");
    Serial.print(c);
    uint32_t count =  lv_obj_get_child_cnt(tileview);
    Serial.print(" Count:");
    Serial.print(count);
    Serial.print(" pageId:");
    Serial.println(pageId);
/*/
    switch (pageId) {
    case WIFI_SCAN_PAGE_ID:
        if ((WiFi.getStatusBits() & WIFI_SCANNING_BIT) != WIFI_SCANNING_BIT ) {
            Serial.println("scanNetworks");
            WiFi.onEvent(WiFiScanDone, WiFiEvent_t::ARDUINO_EVENT_WIFI_SCAN_DONE);
            WiFi.mode(WIFI_STA);
            WiFi.scanNetworks(true);
            canScreenOff = false;
        }
        break;
    case RADIO_TRANSMIT_PAGE_ID:
        lv_timer_resume(transmitTask);
        canScreenOff = false;
        break;
    default:
        if (lastPageID == MIC_IR_PAGE_ID) {
            lv_label_set_text(vad_btn_label, "VAD detect");
            lv_obj_clear_state(vad_btn, 3);
            vTaskSuspend(vadTaskHandler);
        }
        if (WiFi.getMode() != WIFI_OFF ) {
            WiFi.removeEvent(WiFiEvent_t::ARDUINO_EVENT_WIFI_SCAN_DONE);
            WiFi.mode(WIFI_OFF);
            Serial.println("WiFi.mode(WIFI_OFF);");
        }
        if (!transmitTask->paused) {
            lv_timer_pause(transmitTask);
            Serial.println("lv_timer_pause transmitTask");
        }
        canScreenOff = true;
        break;
    }
    */
    lastPageID = pageId;
}

void factory_ui()
{

    static lv_style_t bgStyle;
    lv_style_init(&bgStyle);
    lv_style_set_bg_color(&bgStyle, lv_color_black());

    tileview = lv_tileview_create(lv_scr_act());
    lv_obj_add_style(tileview, &bgStyle, LV_PART_MAIN);
    lv_obj_set_size(tileview, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_add_event_cb(tileview, tileview_change_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *t1 = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t1_0 = lv_tileview_add_tile(tileview, 0, 1, LV_DIR_TOP | LV_DIR_BOTTOM);
    lv_obj_t *t1_1 = lv_tileview_add_tile(tileview, 0, 2, LV_DIR_TOP | LV_DIR_BOTTOM);

    lv_obj_t *t2 = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
   
   //experiment
    lv_obj_set_size(t2, 240, 240);
    lv_obj_set_scrollbar_mode(t2, LV_SCROLLBAR_MODE_OFF);  //experimental for scrolling off
    lv_obj_clear_flag(t2, LV_OBJ_FLAG_SCROLLABLE);
   
    lv_obj_t *t2_1 = lv_tileview_add_tile(tileview, 1, 1, LV_DIR_TOP | LV_DIR_BOTTOM);
    lv_obj_t *t2_2 = lv_tileview_add_tile(tileview, 1, 2, LV_DIR_TOP | LV_DIR_BOTTOM);
    lv_obj_t *t2_3 = lv_tileview_add_tile(tileview, 1, 3, LV_DIR_TOP | LV_DIR_BOTTOM);

    lv_obj_t *t3 = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t3_1 = lv_tileview_add_tile(tileview, 2, 1, LV_DIR_TOP | LV_DIR_BOTTOM);


    lv_obj_t *t4 = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_HOR);
    lv_obj_t *t5 = lv_tileview_add_tile(tileview, 4, 0, LV_DIR_HOR);
    lv_obj_t *t6 = lv_tileview_add_tile(tileview, 5, 0, LV_DIR_HOR);
    lv_obj_t *t7 = lv_tileview_add_tile(tileview, 6, 0, LV_DIR_HOR);


    //productPinmap(t1);
   // lilygo_qrcode(t1_0, "https://www.lilygo.cc/", "Official website");
    //lilygo_qrcode(t1_1, "https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library", "Github website");

    autopilot(t3);
   
    //analogclock2(t2_1);
    //analogclock3(t2_2);
    //digitalClock(t2_3);

    radiiclock(t2);
   // analogclock(t2_1);
   // lcdclock(t2_2);
   // lcarsclock(t2_3);

    devicesInformation(t1);
   // wifiscan(t3_1);

    radioPingPong(t4);
    musicPlay(t5);
   // irRemoteVeiw(t6);
    datetimeVeiw(t6);

    transmitTask =  lv_timer_create(radioTask, 200, NULL);

    lv_disp_trig_activity(NULL);

    lv_obj_set_tile(tileview, t2, LV_ANIM_OFF);
}

void radioTask(lv_timer_t *parent)
{
    char buf[256];
    // check if the previous operation finished
    if (radioTransmitFlag) {
        // reset flag
        radioTransmitFlag = false;

        if (transmitFlag) {
            //TX
            // the previous operation was transmission, listen for response
            // print the result
            if (transmissionState == RADIOLIB_ERR_NONE) {
                // packet was successfully sent
                Serial.println(F("transmission finished!"));
            } else {
                Serial.print(F("failed, code "));
                Serial.println(transmissionState);
            }

            lv_snprintf(buf, 256, "[%u]:Tx %s", lv_tick_get() / 1000, transmissionState == RADIOLIB_ERR_NONE ? "Successed" : "Failed");
            lv_textarea_set_text(radio_ta, buf);

            transmissionState = watch.startTransmit("Hello World!");

        } else {
            // RX
            // the previous operation was reception
            // print data and send another packet
            String str;
            int state = watch.readData(str);

            if (state == RADIOLIB_ERR_NONE) {
                // packet was successfully received
                Serial.println(F("[SX1262] Received packet!"));

                // print data of the packet
                Serial.print(F("[SX1262] Data:\t\t"));
                Serial.println(str);

                // print RSSI (Received Signal Strength Indicator)
                Serial.print(F("[SX1262] RSSI:\t\t"));
                Serial.print(watch.getRSSI());
                Serial.println(F(" dBm"));

                // print SNR (Signal-to-Noise Ratio)
                Serial.print(F("[SX1262] SNR:\t\t"));
                Serial.print(watch.getSNR());
                Serial.println(F(" dB"));


                lv_snprintf(buf, 256, "[%u]:Rx %s \nRSSI:%.2f", lv_tick_get() / 1000, str.c_str(), watch.getRSSI());
                lv_textarea_set_text(radio_ta, buf);
            }

            watch.startReceive();
        }
    }
}

static void draw_part_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    /*If the cells are drawn...*/
    if (dsc->part == LV_PART_ITEMS) {
        uint32_t row = dsc->id / lv_table_get_col_cnt(obj);
        uint32_t col = dsc->id - row * lv_table_get_col_cnt(obj);

        /*Make the texts in the first cell center aligned*/
        if (row == 0) {
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
            dsc->rect_dsc->bg_color = lv_color_mix(lv_palette_main(LV_PALETTE_BLUE), dsc->rect_dsc->bg_color, LV_OPA_20);
            dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        }
        /*In the first column align the texts to the right*/
        else if (col == 0) {
            dsc->label_dsc->align = LV_TEXT_ALIGN_RIGHT;
        }

        /*MAke every 2nd row grayish*/
        if ((row != 0 && row % 2) == 0) {
            dsc->rect_dsc->bg_color = lv_color_mix(lv_palette_main(LV_PALETTE_GREY), dsc->rect_dsc->bg_color, LV_OPA_10);
            dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        }
    }
}


void productPinmap(lv_obj_t *parent)
{

    struct  board_struct {
        const char *func_name;
        const uint8_t pin;
    } __lilygo[] = {
        {"TFT MOSI", BOARD_TFT_MOSI},
        {"TFT SCK", BOARD_TFT_SCLK},
        {"TFT CS", BOARD_TFT_CS},
        {"TFT DC", BOARD_TFT_DC},
        {"TFT BL", BOARD_TFT_BL},
        {"TOUCH SDA", BOARD_TOUCH_SDA},
        {"TOUCH SCL", BOARD_TOUCH_SCL},
        {"TOUCH INT", BOARD_TOUCH_INT},
        {"SENSOR SDA", BOARD_I2C_SDA},
        {"SENSOR SCL", BOARD_I2C_SCL},
        {"RTC INT", BOARD_RTC_INT_PIN},
        {"PMU INT", BOARD_PMU_INT},
        {"SENSOR INT", BOARD_BMA423_INT1},
        {"IR Remote", BOARD_IR_PIN},
        {"DAC BCK", BOARD_DAC_IIS_BCK},
        {"DAC WS", BOARD_DAC_IIS_WS},
        {"DAC DOUT", BOARD_DAC_IIS_DOUT},
        {"RADIO SCK", BOARD_RADIO_SCK},
        {"RADIO MISO", BOARD_RADIO_MISO},
        {"RADIO MOSI", BOARD_RADIO_MOSI},
        {"RADIO CS", BOARD_RADIO_SS},
        {"RADIO DIO1", BOARD_RADIO_DI01},
        {"RADIO RST", BOARD_RADIO_RST},
        {"RADIO BUSY", BOARD_RADIO_BUSY},
        {"MIC DOUT", BOARD_MIC_DATA},
        {"MIC SCK", BOARD_MIC_CLOCK}
    };

    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_100);
    lv_style_set_bg_img_opa(&cont_style, LV_OPA_100);
    lv_style_set_bg_color(&cont_style, lv_color_black());
    // lv_style_set_line_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_border_width(&cont_style, 0);
    lv_style_set_text_color(&cont_style, DEFAULT_COLOR);


    lv_obj_t *table = lv_table_create(parent);
    lv_obj_set_scrollbar_mode(table, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(table, LV_DIR_VER);
    lv_obj_add_style(table, &cont_style, LV_PART_SCROLLBAR);
    lv_obj_add_style(table, &cont_style, LV_PART_ITEMS);
    lv_obj_add_style(table, &cont_style, LV_PART_MAIN);

    lv_table_set_cell_value(table, 0, 0, "Name");
    lv_table_set_cell_value(table, 0, 1, "GPIO");
    for (int i = 1; i < sizeof(__lilygo) / sizeof(__lilygo[0]); ++i) {
        lv_table_set_cell_value_fmt(table, i, 0, "%s", __lilygo[i].func_name);
        lv_table_set_cell_value_fmt(table, i, 1, "%d", __lilygo[i].pin);
    }

    lv_obj_add_event_cb(table, draw_part_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);

}

void lilygo_qrcode(lv_obj_t *parent, const char *url, const char *str)
{
    lv_color_t bg_color = lv_palette_lighten(LV_PALETTE_YELLOW, 5);
    lv_color_t fg_color = lv_palette_darken(LV_PALETTE_YELLOW, 4);
    lv_coord_t size = 150;
    lv_obj_t *qr = lv_qrcode_create(parent, size, fg_color, bg_color);

    lv_qrcode_update(qr, url, strlen(url));
    lv_obj_center(qr);
    /*Add a border with bg_color*/
    lv_obj_set_style_border_color(qr, bg_color, 0);
    lv_obj_set_style_border_width(qr, 5, 0);

    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, str );
    lv_obj_set_style_text_color(label, DEFAULT_COLOR, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &font_sandbox, LV_PART_MAIN);
    lv_obj_align_to(label, qr, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}


lv_obj_t *watch_radii_hh_img;
lv_obj_t *watch_radii_mm_img;
lv_obj_t *watch_radii_ss_img;
lv_obj_t *watch_radii_batt_img;
lv_obj_t *watch_radii_batt2_img;
lv_obj_t *watch_radii_mindot2_img;
lv_obj_t *watch_radii_daydot_img;
lv_obj_t *watch_radii_centerdial_img;
lv_obj_t *radiiface;

void radiiclock(lv_obj_t *parent)
{

 bool antialias = true;
    lv_img_header_t header;

    const void *clock_filename = &raddiblank;
    const void *sec_filename = &radiiwatchface_sec;
    const void *batt_filename = &battery;
    const void *batt2_filename = &battery2;
    const void *mindot_filename = &mindot;
    const void *centerdial_filename = &centerdial;
    const void *daydot_filename = &daydotc;
                    
//images
 lv_obj_t *clock_bg =  lv_img_create(parent);
    lv_img_set_src(clock_bg, clock_filename);
    lv_obj_set_size(clock_bg, 240, 240);
    lv_obj_center(clock_bg);
lv_obj_clear_flag( clock_bg, LV_OBJ_FLAG_SCROLLABLE );

        lv_img_decoder_get_info(batt_filename, &header);
    watch_radii_batt_img = lv_img_create(parent);
    lv_img_set_src(watch_radii_batt_img,  batt_filename);
    //lv_obj_center(watch_radii_batt_img);
        lv_obj_align(watch_radii_batt_img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_pivot(watch_radii_batt_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_radii_batt_img, antialias);
        lv_obj_set_size(watch_radii_batt_img, 240, 240);
        lv_obj_clear_flag( watch_radii_batt_img, LV_OBJ_FLAG_SCROLLABLE );

    lv_img_decoder_get_info(batt2_filename, &header);
    watch_radii_batt2_img = lv_img_create(parent);
    lv_img_set_src(watch_radii_batt2_img,  batt2_filename);
    lv_obj_center(watch_radii_batt2_img);
    lv_img_set_pivot(watch_radii_batt2_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_radii_batt2_img, antialias);
        lv_obj_set_size(watch_radii_batt2_img, 240, 240);
        lv_obj_clear_flag( watch_radii_batt2_img, LV_OBJ_FLAG_SCROLLABLE );

        lv_img_decoder_get_info(watch_radii_daydot_img, &header);
    watch_radii_daydot_img = lv_img_create(parent);
    lv_img_set_src(watch_radii_daydot_img,  daydot_filename);
    lv_obj_align(watch_radii_daydot_img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_pivot(watch_radii_daydot_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_radii_daydot_img, antialias);
            lv_obj_set_size(watch_radii_daydot_img, 240, 240);
        lv_obj_clear_flag( watch_radii_daydot_img, LV_OBJ_FLAG_SCROLLABLE );

    lv_img_decoder_get_info(sec_filename, &header);
    watch_radii_ss_img = lv_img_create(parent);
    lv_img_set_src(watch_radii_ss_img,  sec_filename);
    lv_obj_center(watch_radii_ss_img);
    lv_img_set_pivot(watch_radii_ss_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_radii_ss_img, antialias);

 lv_obj_t *watch_radii_centerdial_img =  lv_img_create(parent);
    lv_img_set_src(watch_radii_centerdial_img, centerdial_filename);
    lv_obj_set_size(watch_radii_centerdial_img, 240, 240);
    lv_obj_center(watch_radii_centerdial_img);
        lv_img_set_antialias(watch_radii_centerdial_img, antialias);

    lv_img_decoder_get_info(watch_radii_mindot2_img, &header);
    watch_radii_mindot2_img = lv_img_create(parent);
    lv_img_set_src(watch_radii_mindot2_img, mindot_filename);
    lv_obj_align(watch_radii_mindot2_img, LV_ALIGN_CENTER, 0, 0);
    lv_img_set_antialias(watch_radii_mindot2_img, antialias);

//labels
    static lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_color(&label_style, lv_color_white());

//hour
     label_date_h = lv_label_create(parent);
    lv_label_set_text(label_date_h, "%d");
    lv_obj_set_style_text_font(label_date_h, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_date_h, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(label_date_h, LV_ALIGN_CENTER, 0, 0);

//minute
     label_date_m = lv_label_create(parent);
    lv_label_set_text(label_date_m, "%d");
    lv_obj_set_style_text_font(label_date_m, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_date_m, lv_color_black(), LV_PART_MAIN);
    lv_obj_align(label_date_m, LV_ALIGN_CENTER, 75, 0);
 
//day of week
/*
 label_date_a = lv_label_create(parent);
    lv_label_set_text(label_date_a, "%c");
    lv_obj_set_style_text_font(label_date_a, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_date_a, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(label_date_a, LV_ALIGN_CENTER, -80, -30);
*/

//day of month
     label_date_d = lv_label_create(parent);
    lv_label_set_text(label_date_d, "%d");
    lv_obj_set_style_text_font(label_date_d, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_date_d, lv_color_black(), LV_PART_MAIN);
    lv_obj_align(label_date_d, LV_ALIGN_CENTER, -105, 0);
     
//battery level
    battery_percent2 = lv_label_create(parent);
    lv_label_set_text(battery_percent2, "%d");
    lv_obj_align(battery_percent2, LV_ALIGN_RIGHT_MID, -15, 20);
    lv_obj_add_style(battery_percent2, &label_style, LV_PART_MAIN);

//step counter
    radstep_counter = lv_label_create(parent);
    lv_label_set_text(radstep_counter, "%d");
    lv_obj_align(radstep_counter, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_add_style(radstep_counter, &label_style, LV_PART_MAIN);

//read the time
    clockTimer =   lv_timer_create([](lv_timer_t *timer) {
        time_t now;
        struct tm  timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

//minute bubble rotation
int bubblerot = (timeinfo.tm_min * 60) -900;
if (bubblerot < 0) bubblerot = 360 - abs(bubblerot);
        lv_img_set_angle( watch_radii_mindot2_img, bubblerot);

//not sure if needed     
lv_img_set_angle(
     watch_radii_ss_img, (-(timeinfo.tm_sec) * 60 +10));

//24h to 12h clock
        if (timeinfo.tm_hour >= 13) timeinfo.tm_hour = timeinfo.tm_hour - 12;

 //update time labels
    lv_label_set_text_fmt(label_date_m, "%d", timeinfo.tm_min);
    lv_label_set_text_fmt(label_date_h, "%d", timeinfo.tm_hour);
    lv_label_set_text_fmt(label_date_d, "%d", lv_datetime[2].defaultVal);


//update minute label locations based on time
    // Convert from degrees to radians via multiplication by PI/180      
    int angleInDegrees = timeinfo.tm_min * 6 - 90;  
    if (angleInDegrees < 0) angleInDegrees = 360 - abs(angleInDegrees);
    float xminlabel = (65 * cos(angleInDegrees * (3.1415926 / 180)));
    float yminlabel = (65 * sin(angleInDegrees * (3.1415926 / 180)));

// move minute label based on time
    lv_obj_align(label_date_m, LV_ALIGN_CENTER, xminlabel, yminlabel);
    lv_obj_align(watch_radii_mindot2_img, LV_ALIGN_CENTER, xminlabel, yminlabel);

//update day of week dot

        int weekday = trunc(timeinfo.tm_wday);
        int wkdyang;
        if (weekday == 0) {wkdyang = 225;}
        else if (weekday == 1){wkdyang = 210;}
        else if (weekday == 2){wkdyang = 195;}
        else if (weekday == 3){wkdyang = 180;}
        else if (weekday == 4){wkdyang = 165;}
        else if (weekday == 5){wkdyang = 150;}
        else if (weekday == 6){wkdyang = 135;}
        else wkdyang = 0;

 //lv_label_set_text_fmt(label_date_a, "%c", wkdy);
            float xwkdlabel = (105 * cos(wkdyang * (3.1415926 / 180)));
            float ywkdlabel = (105 * sin(wkdyang * (3.1415926 / 180)));
    lv_obj_align(label_date_d, LV_ALIGN_CENTER, xwkdlabel, ywkdlabel);
    lv_obj_align(watch_radii_daydot_img, LV_ALIGN_CENTER, xwkdlabel, ywkdlabel);
    

// Update battery percent
        int percent = watch.getBatteryPercent();
        lv_label_set_text_fmt(battery_percent2, "%d", percent == -1 ? 0 : percent);
        lv_img_set_angle(
            watch_radii_batt_img, ((100-percent) *10)-50 );   // removed * 100 after (percent)


//second animation
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, watch_radii_ss_img);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_angle);
        lv_anim_set_values(&a, (timeinfo.tm_sec * 60) % 3600,
                           (timeinfo.tm_sec + 1) * 60);
        lv_anim_set_time(&a, 1000);
        lv_anim_start(&a);

 // Update step counter
        lv_label_set_text_fmt(radstep_counter, "%d", stepCounter);

    },
    1000, NULL); 

}

void lcarsclock(lv_obj_t *parent)
{

}


void lcdclock(lv_obj_t *parent)
{
 bool antialias = true;
    lv_img_header_t header;
    
    const void *clock_filename = &lcd_watchface_dial;
    lv_obj_t *lcdclock_bg =  lv_img_create(parent);
    lv_img_set_src(lcdclock_bg, clock_filename);
    lv_obj_set_size(lcdclock_bg, 240, 240);
    lv_obj_center(lcdclock_bg);

    static lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_color(&label_style, lv_color_hex(0xff0000));


    label_datetime = lv_label_create(parent);
    lv_label_set_text(label_datetime, "%d:%d");
    lv_obj_set_style_text_font(label_datetime, &font_firacode_60, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_datetime, lv_color_hex(0xff0000), LV_PART_MAIN);
    lv_obj_align(label_datetime, LV_ALIGN_CENTER, 0, 0);

    battery_percent3 = lv_label_create(parent);
    lv_label_set_text(battery_percent3, "%d%");
    lv_obj_align(battery_percent3, LV_ALIGN_CENTER, 0, -100);
     lv_obj_set_style_text_font(battery_percent3, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(battery_percent3, lv_color_hex(0xFF0000), LV_PART_MAIN);

    lcdstep_counter = lv_label_create(parent);
    lv_label_set_text(lcdstep_counter,  "%d");
    lv_obj_align(lcdstep_counter, LV_ALIGN_BOTTOM_MID, 0, -35);
         lv_obj_set_style_text_font(lcdstep_counter, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lcdstep_counter, lv_color_hex(0xFF0000), LV_PART_MAIN);


    lv_timer_create([](lv_timer_t *timer) {
        time_t now;
        struct tm  timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_hour >= 13) timeinfo.tm_hour = timeinfo.tm_hour - 12;
        
        static  bool rever = false;
        lv_label_set_text_fmt(label_datetime, rever ? "%02d:%02d" : "%02d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        rever = !rever;

// Update battery percent
        int percent = watch.getBatteryPercent();
       lv_label_set_text_fmt(battery_percent3, "%d%", percent == -1 ? 0 : percent);

        // Update step counter
        lv_label_set_text_fmt(lcdstep_counter, "Steps: %d", stepCounter);
        
    },
    1000, NULL);

}


void analogclock(lv_obj_t *parent)
{
    bool antialias = true;
    lv_img_header_t header;

    const void *clock_filename = &clock_face;
    const void *hour_filename = &clock_hour_hand;
    const void *min_filename = &clock_minute_hand;
    const void *sec_filename = &clock_second_hand;

    lv_obj_t *clock_bg =  lv_img_create(parent);
    lv_img_set_src(clock_bg, clock_filename);
    lv_obj_set_size(clock_bg, 240, 240);
    lv_obj_center(clock_bg);


    hour_img = lv_img_create(parent);
    lv_img_decoder_get_info(hour_filename, &header);
    lv_img_set_src(hour_img, hour_filename);
    lv_obj_center(hour_img);
    lv_img_set_pivot(hour_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(hour_img, antialias);

    lv_img_decoder_get_info(min_filename, &header);
    min_img = lv_img_create(parent);
    lv_img_set_src(min_img,  min_filename);
    lv_obj_center(min_img);
    lv_img_set_pivot(min_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(min_img, antialias);

    lv_img_decoder_get_info(sec_filename, &header);
    sec_img = lv_img_create(parent);
    lv_img_set_src(sec_img,  sec_filename);
    lv_obj_center(sec_img);
    lv_img_set_pivot(sec_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(sec_img, antialias);

    static lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_color(&label_style, lv_color_white());

    battery_percent = lv_label_create(parent);
    lv_label_set_text(battery_percent, "%d");
    lv_obj_align(battery_percent, LV_ALIGN_LEFT_MID, 68, -10);
    lv_obj_add_style(battery_percent, &label_style, LV_PART_MAIN);

    weather_celsius = lv_label_create(parent);
    lv_label_set_text(weather_celsius, "23°C");
    lv_obj_align(weather_celsius, LV_ALIGN_RIGHT_MID, -62, -2);
    lv_obj_add_style(weather_celsius, &label_style, LV_PART_MAIN);

    step_counter = lv_label_create(parent);
    lv_label_set_text(step_counter, "%d");
    lv_obj_align(step_counter, LV_ALIGN_BOTTOM_MID, 0, -55);
    lv_obj_add_style(step_counter, &label_style, LV_PART_MAIN);

    clockTimer =   lv_timer_create([](lv_timer_t *timer) {

        time_t now;
        struct tm  timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        lv_img_set_angle(
            hour_img, ((timeinfo.tm_hour) * 300 + ((timeinfo.tm_min) * 5)) % 3600);
        lv_img_set_angle(min_img, (timeinfo.tm_min) * 60);

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, sec_img);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_angle);
        lv_anim_set_values(&a, (timeinfo.tm_sec * 60) % 3600,
                           (timeinfo.tm_sec + 1) * 60);
        lv_anim_set_time(&a, 1000);
        lv_anim_start(&a);

        // Update step counter
        lv_label_set_text_fmt(step_counter, "%d", stepCounter);

        // Update battery percent
        int percent = watch.getBatteryPercent();
        lv_label_set_text_fmt(battery_percent, "%d", percent == -1 ? 0 : percent);

        // float  temp = watch.readBmaTemp();
        // Serial.print(temp);
        // Serial.println("*C");
        // lv_label_set_text_fmt(weather_celsius, "%d°C", (int)temp);

    },
    1000, NULL);
}


lv_obj_t *watch_if_hh_img;
lv_obj_t *watch_if_mm_img;
lv_obj_t *watch_if_ss_img;

void analogclock2(lv_obj_t *parent)
{
    bool antialias = true;
    lv_img_header_t header;

    const void *clock_filename = &watch_if;
    const void *hour_filename = &watch_if_hour;
    const void *min_filename = &watch_if_min;
    const void *sec_filename = &watch_if_sec;


    lv_obj_t *clock_bg =  lv_img_create(parent);
    lv_img_set_src(clock_bg, &watch_bg);
    lv_obj_set_size(clock_bg, 240, 240);
    lv_obj_center(clock_bg);

    lv_obj_t *clock_if =  lv_img_create(parent);
    lv_img_set_src(clock_if, clock_filename);
    lv_obj_set_size(clock_if, 240, 240);
    lv_obj_center(clock_if);


    watch_if_hh_img = lv_img_create(parent);
    lv_img_decoder_get_info(hour_filename, &header);
    lv_img_set_src(watch_if_hh_img, hour_filename);
    lv_obj_center(watch_if_hh_img);
    lv_img_set_pivot(watch_if_hh_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_if_hh_img, antialias);

    lv_img_decoder_get_info(min_filename, &header);
    watch_if_mm_img = lv_img_create(parent);
    lv_img_set_src(watch_if_mm_img,  min_filename);
    lv_obj_center(watch_if_mm_img);
    lv_img_set_pivot(watch_if_mm_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_if_mm_img, antialias);

    lv_img_decoder_get_info(sec_filename, &header);
    watch_if_ss_img = lv_img_create(parent);
    lv_img_set_src(watch_if_ss_img,  sec_filename);
    lv_obj_center(watch_if_ss_img);
    lv_img_set_pivot(watch_if_ss_img, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_if_ss_img, antialias);

    lv_timer_create([](lv_timer_t *timer) {

        time_t now;
        struct tm  timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        lv_img_set_angle(watch_if_hh_img, ((timeinfo.tm_hour) * 300 + ((timeinfo.tm_min) * 5)) % 3600);
        lv_img_set_angle(watch_if_mm_img, (timeinfo.tm_min) * 60);

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, watch_if_ss_img);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_angle);
        lv_anim_set_values(&a, (timeinfo.tm_sec * 60) % 3600,
                           (timeinfo.tm_sec + 1) * 60);
        lv_anim_set_time(&a, 1000);
        lv_anim_start(&a);
    },
    1000, NULL);
}

lv_obj_t *watch_if_hh_img3;
lv_obj_t *watch_if_mm_img3;
lv_obj_t *watch_if_ss_img3;

void analogclock3(lv_obj_t *parent)
{
    bool antialias = true;
    lv_img_header_t header;

    const void *clock_filename = &watch_if_bg2;
    const void *hour_filename = &watch_if_hour2;
    const void *min_filename = &watch_if_min2;
    const void *sec_filename = &watch_if_sec2;


    lv_obj_t *clock_if =  lv_img_create(parent);
    lv_img_set_src(clock_if, clock_filename);
    lv_obj_set_size(clock_if, 240, 240);
    lv_obj_center(clock_if);


    watch_if_hh_img3 = lv_img_create(parent);
    lv_img_decoder_get_info(hour_filename, &header);
    lv_img_set_src(watch_if_hh_img3, hour_filename);
    lv_obj_center(watch_if_hh_img3);
    lv_img_set_pivot(watch_if_hh_img3, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_if_hh_img3, antialias);

    lv_img_decoder_get_info(min_filename, &header);
    watch_if_mm_img3 = lv_img_create(parent);
    lv_img_set_src(watch_if_mm_img3,  min_filename);
    lv_obj_center(watch_if_mm_img3);
    lv_img_set_pivot(watch_if_mm_img3, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_if_mm_img3, antialias);

    lv_img_decoder_get_info(sec_filename, &header);
    watch_if_ss_img3 = lv_img_create(parent);
    lv_img_set_src(watch_if_ss_img3,  sec_filename);
    lv_obj_center(watch_if_ss_img3);
    lv_img_set_pivot(watch_if_ss_img3, header.w / 2, header.h / 2);
    lv_img_set_antialias(watch_if_ss_img3, antialias);

    lv_timer_create([](lv_timer_t *timer) {

        time_t now;
        struct tm  timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        lv_img_set_angle(watch_if_hh_img3, ((timeinfo.tm_hour) * 300 + ((timeinfo.tm_min) * 5)) % 3600);
        lv_img_set_angle(watch_if_mm_img3, (timeinfo.tm_min) * 60);

        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, watch_if_ss_img3);
        lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_angle);
        lv_anim_set_values(&a, (timeinfo.tm_sec * 60) % 3600,
                           (timeinfo.tm_sec + 1) * 60);
        lv_anim_set_time(&a, 1000);
        lv_anim_start(&a);
    },
    1000, NULL);

}




void digitalClock(lv_obj_t *parent)
{
    const void *clock_filename = &watch_if_5;
    lv_obj_t *clock_if =  lv_img_create(parent);
    lv_img_set_src(clock_if, clock_filename);
    lv_obj_set_size(clock_if, 240, 240);
    lv_obj_center(clock_if);

    label_datetime = lv_label_create(parent);
    lv_label_set_text(label_datetime, "00:00");
    lv_obj_set_style_text_font(label_datetime, &font_firacode_60, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_datetime, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(label_datetime, LV_ALIGN_CENTER, 0, 50);

    lv_timer_create([](lv_timer_t *timer) {
        time_t now;
        struct tm  timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        static  bool rever = false;
        lv_label_set_text_fmt(label_datetime, rever ? "%02d:%02d" : "%02d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        rever = !rever;
    },
    1000, NULL);
}

static void slider_event_cb(lv_event_t *e)
{
    lv_obj_t *slider = lv_event_get_target(e);
    lv_obj_t *slider_label =  (lv_obj_t *)lv_event_get_user_data(e);
    uint8_t level = (uint8_t)lv_slider_get_value(slider);
    int percentage = map(level, 0, 255, 0, 100);
    lv_label_set_text_fmt(slider_label, "%u%%", percentage);
    lv_obj_align_to(slider_label, slider, LV_ALIGN_CENTER, 0, 0);
    watch.setBrightness(level);
}

static void light_sw_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        Serial.printf("State: %s\n", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
        lightSleep = lv_obj_has_state(obj, LV_STATE_CHECKED);
    }
}

void devicesInformation(lv_obj_t *parent)
{
    lv_obj_t *label = lv_label_create(parent);
    String text;
    text = "Chip:";
    text += ESP.getChipModel();
    text += " / Rev:";
    text += ESP.getChipRevision();
    text += "\n";
    text += "Flash Size:";
    text += ESP.getFlashChipSize() / 1024 / 1024;
    text += "MB Type:OPI\nSpeed:";
    text += ESP.getFlashChipSpeed() / 1000 / 1000;
    text += "MHz\n";
    text += "Psram Size:";
    text += (ESP.getPsramSize() / 1024);
    text += "/";
    text += (ESP.getFreePsram() / 1024);
    text += "kb\n";
    text += "IDF Version: ";
    text += ESP.getSdkVersion();
    text += "\n";
    text += "Arduino Version: v";
    text += String(ESP_ARDUINO_VERSION_MAJOR);
    text += ".";
    text += String(ESP_ARDUINO_VERSION_MINOR);
    text += ".";
    text += String(ESP_ARDUINO_VERSION_PATCH);
    text += "\n";
    text += "lvgl Version: v";
    text += String(lv_version_major());
    text += ".";
    text += String(lv_version_minor());
    text += ".";
    text += String(lv_version_patch());
    text += "\n";
    text += "TFT_eSPI Version: v";
    text += TFT_ESPI_VERSION;
    text += "\n";

    /*Create a transition*/
    static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, LV_STYLE_PROP_INV};
    static lv_style_transition_dsc_t transition_dsc;
    lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 300, 0, NULL);

    static lv_style_t style_indicator;
    static lv_style_t style_knob;


    lv_style_init(&style_indicator);
    lv_style_set_bg_opa(&style_indicator, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indicator, DEFAULT_COLOR);
    lv_style_set_radius(&style_indicator, LV_RADIUS_CIRCLE);
    lv_style_set_transition(&style_indicator, &transition_dsc);

    lv_style_init(&style_knob);
    lv_style_set_bg_opa(&style_knob, LV_OPA_COVER);
    lv_style_set_bg_color(&style_knob, DEFAULT_COLOR);
    lv_style_set_border_color(&style_knob, lv_palette_darken(LV_PALETTE_YELLOW, 2));
    lv_style_set_border_width(&style_knob, 2);
    lv_style_set_radius(&style_knob, LV_RADIUS_CIRCLE);
    lv_style_set_pad_all(&style_knob, 6); /*Makes the knob larger*/
    lv_style_set_transition(&style_knob, &transition_dsc);


    static lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_color(&label_style, lv_color_white());
    lv_obj_add_style(label, &label_style, LV_PART_MAIN);
    lv_label_set_text(label, text.c_str());
    lv_obj_set_style_text_font(label, &font_jetBrainsMono, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, DEFAULT_COLOR, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);


    /*Create a slider and add the style*/
    lv_obj_t *slider = lv_slider_create(parent);
    lv_obj_set_size(slider, 200, 30);
    lv_slider_set_range(slider, 5, 255);
    lv_obj_add_style(slider, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(slider, &style_knob, LV_PART_KNOB);
    lv_obj_align_to(slider, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_slider_set_value(slider, watch.getBrightness(), LV_ANIM_OFF);

    /*Create a label below the slider*/
    lv_obj_t *slider_label = lv_label_create(parent);
    lv_label_set_text_fmt(slider_label, "%u%%", watch.getBrightness());
    lv_obj_set_style_text_color(slider_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, slider_label);
    lv_obj_align_to(slider_label, slider, LV_ALIGN_CENTER, 0, 0);

    // label = lv_label_create(parent);
    // lv_label_set_text(label, "LightSleep:");
    // lv_obj_align(label, LV_ALIGN_BOTTOM_LEFT, 20, -40);
    // lv_obj_add_style(label, &label_style, LV_PART_MAIN);

    // lv_obj_t *sw = lv_switch_create(parent);
    // lv_obj_align(sw, LV_ALIGN_BOTTOM_RIGHT, -20, -40);
    // lv_obj_add_event_cb(sw, light_sw_event_cb, LV_EVENT_VALUE_CHANGED, sw);
}

void wifiscan(lv_obj_t *parent)
{
    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_line_opa(&cont_style, LV_OPA_50);
    lv_style_set_border_width(&cont_style, 5);
    lv_style_set_border_color(&cont_style, DEFAULT_COLOR);
    lv_style_set_text_color(&cont_style, DEFAULT_COLOR);
    lv_style_set_pad_right(&cont_style, 0);
    lv_style_set_pad_left(&cont_style, 0);

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_disp_get_hor_res(NULL), LV_PCT(100));
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_add_style(cont, &cont_style, LV_PART_MAIN);


    lv_obj_t *label = lv_label_create(cont);
    lv_label_set_text(label, "Scanning...");
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
    lv_obj_align_to(label, NULL, LV_ALIGN_TOP_MID, 0, 2);


    static lv_style_t table_style;
    lv_style_init(&table_style);
    lv_style_set_bg_opa(&table_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&table_style, LV_OPA_TRANSP);
    lv_style_set_line_opa(&table_style, LV_OPA_50);
    lv_style_set_border_width(&table_style, 1);
    lv_style_set_border_color(&table_style, DEFAULT_COLOR);
    lv_style_set_text_color(&table_style, DEFAULT_COLOR);
    lv_style_set_text_font(&table_style, &lv_font_montserrat_16);


    wifi_table_list = lv_table_create(cont);
    lv_obj_set_scroll_dir(wifi_table_list, LV_DIR_VER);
    lv_obj_set_size(wifi_table_list, LV_PCT(100), LV_PCT(100));

    lv_table_set_col_width(wifi_table_list, 0, lv_disp_get_ver_res(NULL) - 20);
    lv_table_set_row_cnt(wifi_table_list, 1);
    lv_table_set_col_cnt(wifi_table_list, 1);

    lv_obj_add_style(wifi_table_list, &table_style, LV_PART_ITEMS);
    lv_obj_add_style(wifi_table_list, &table_style, LV_PART_MAIN);

    lv_obj_align_to(wifi_table_list, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
}

static void radio_rxtx_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t id = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, id);
    switch (id) {
    case 0:
        lv_timer_resume(transmitTask);
        // TX
        // send the first packet on this node
        Serial.print(F("[Radio] Sending first packet ... "));
        transmissionState = watch.startTransmit("Hello World!");
        transmitFlag = true;

        break;
    case 1:
        lv_timer_resume(transmitTask);
        // RX
        Serial.print(F("[Radio] Starting to listen ... "));
        if (watch.startReceive() == RADIOLIB_ERR_NONE) {
            Serial.println(F("success!"));
        } else {
            Serial.println(F("failed "));
        }
        transmitFlag = false;
        lv_textarea_set_text(radio_ta, "[RX]:Listening.");

        break;
    case 2:
        if (!transmitTask->paused) {
            lv_textarea_set_text(radio_ta, "Radio has disable.");
            lv_timer_pause(transmitTask);
            watch.standby();
            // watch.sleep();
        }
        break;
    default:
        break;
    }
}

static void radio_bandwidth_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t id = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, id);

    // set carrier bandwidth
    const float bw[] = {125.0, 250.0, 500.0};
    if (id > sizeof(bw) / sizeof(bw[0])) {
        Serial.println("invalid bandwidth params!");
        return;
    }

    bool isRunning = !transmitTask->paused;
    if (isRunning) {
        lv_timer_pause(transmitTask);
        watch.standby();
    }

    // set bandwidth
    if (watch.setBandwidth(bw[id]) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
    }

    if (transmitFlag) {
        watch.startTransmit("");
    } else {
        watch.startReceive();
    }

    if (isRunning) {
        lv_timer_resume(transmitTask);
    }
}

static void radio_freq_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t id = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, id);

    // set carrier frequency
    const float freq[] = {433.0, 470.0, 868.0, 915.0, 923.0};
    if (id > sizeof(freq) / sizeof(freq[0])) {
        Serial.println("invalid params!");
        return;
    }

    bool isRunning = !transmitTask->paused;
    if (isRunning) {
        lv_timer_pause(transmitTask);
    }

    if (watch.setFrequency(freq[id]) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
    }


    if (transmitFlag) {
        watch.startTransmit("");
    } else {
        watch.startReceive();
    }

    if (isRunning) {
        lv_timer_resume(transmitTask);
    }

}

static void radio_power_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t id = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, id);


    bool isRunning = !transmitTask->paused;
    if (isRunning) {
        lv_timer_pause(transmitTask);
        watch.standby();
    }

    uint8_t dBm[] = {
        2, 5, 10, 12, 17, 20, 22
    };
    if (id > sizeof(dBm) / sizeof(dBm[0])) {
        Serial.println("invalid dBm params!");
        return;
    }
    // "2dBm\n"
    // "5dBm\n"
    // "10dBm\n"
    // "12dBm\n"
    // "17dBm\n"
    // "20dBm\n"
    // "22dBmn"

    // set output power (accepted range is - 17 - 22 dBm)
    if (watch.setOutputPower(dBm[id]) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        Serial.println(F("Selected output power is invalid for this module!"));
    }

    if (transmitFlag) {
        watch.startTransmit("");
    } else {
        watch.startReceive();
    }

    if (isRunning) {
        lv_timer_resume(transmitTask);
    }
}

static void radio_tx_interval_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    uint32_t id = lv_dropdown_get_selected(obj);
    Serial.printf("Option: %s id:%u\n", buf, id);

    // set carrier bandwidth
    uint16_t interval[] = {100, 200, 500, 1000, 2000, 3000};
    if (id > sizeof(interval) / sizeof(interval[0])) {
        Serial.println("invalid  tx interval params!");
        return;
    }
    // Save the configured transmission interval
    configTransmitInterval = interval[id];
    lv_timer_set_period(transmitTask, interval[id]);
}

void radioPingPong(lv_obj_t *parent)
{
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_black());
    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_border_width(&style, 5);
    lv_style_set_border_color(&style, DEFAULT_COLOR);
    lv_style_set_outline_color(&style, DEFAULT_COLOR);
    lv_style_set_bg_opa(&style, LV_OPA_50);

    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_line_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_border_width(&cont_style, 0);
    lv_style_set_text_color(&cont_style, DEFAULT_COLOR);
    // lv_style_set_text_color(&cont_style, lv_color_white());

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_disp_get_hor_res(NULL), 400);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_add_style(cont, &cont_style, LV_PART_MAIN);

    radio_ta = lv_textarea_create(cont);
    lv_obj_set_size(radio_ta, 210, 80);
    lv_obj_align(radio_ta, LV_ALIGN_TOP_MID, 0, 20);
    lv_textarea_set_text(radio_ta, "Radio Test");
    lv_textarea_set_max_length(radio_ta, 256);
    lv_textarea_set_cursor_click_pos(radio_ta, false);
    lv_textarea_set_text_selection(radio_ta, false);
    lv_obj_add_style(radio_ta, &style, LV_PART_MAIN);
    // lv_textarea_set_one_line(radio_ta, true);

    /////////////////////////////!!!!!!!!!!!!!!!!!!!

    static lv_style_t cont1_style;
    lv_style_init(&cont1_style);
    lv_style_set_bg_opa(&cont1_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&cont1_style, LV_OPA_TRANSP);
    lv_style_set_line_opa(&cont1_style, LV_OPA_TRANSP);
    lv_style_set_text_color(&cont1_style, DEFAULT_COLOR);
    lv_style_set_text_color(&cont1_style, lv_color_white());
    lv_style_set_border_width(&cont1_style, 5);
    lv_style_set_border_color(&cont1_style, DEFAULT_COLOR);
    lv_style_set_outline_color(&cont1_style, DEFAULT_COLOR);


    //! cont1
    lv_obj_t *cont1 = lv_obj_create(cont);
    lv_obj_set_scrollbar_mode(cont1, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(cont1, LV_FLEX_FLOW_ROW_WRAP);
    // lv_obj_set_scroll_dir(cont1, LV_DIR_HOR);
    lv_obj_set_size(cont1, 210, 300);
    lv_obj_add_style(cont1, &cont1_style, LV_PART_MAIN);


    lv_obj_t *dd ;

    dd = lv_dropdown_create(cont1);
    lv_dropdown_set_options(dd, "TX\n"
                            "RX\n"
                            "Disable"
                           );
    lv_dropdown_set_selected(dd, 2);
    lv_obj_add_flag(dd, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(dd, 170, 50);
    lv_obj_add_event_cb(dd, radio_rxtx_cb,
                        LV_EVENT_VALUE_CHANGED
                        , NULL);

    dd = lv_dropdown_create(cont1);
    lv_dropdown_set_options(dd, "433M\n"
                            "470M\n"
                            "868M\n"
                            "915M\n"
                            "923M"
                           );
    lv_dropdown_set_selected(dd, 2);
    lv_obj_add_flag(dd, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(dd, 170, 50);
    lv_obj_add_event_cb(dd, radio_freq_cb,
                        LV_EVENT_VALUE_CHANGED
                        , NULL);


    dd = lv_dropdown_create(cont1);
    lv_dropdown_set_options(dd, "125KHz\n"
                            "250KHz\n"
                            "500KHz"
                           );
    lv_obj_add_flag(dd, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(dd, 170, 50);
    lv_dropdown_set_selected(dd, 1);
    lv_obj_add_event_cb(dd, radio_bandwidth_cb,
                        LV_EVENT_VALUE_CHANGED
                        , NULL);


    dd = lv_dropdown_create(cont1);
    lv_dropdown_set_options(dd, "2dBm\n"
                            "5dBm\n"
                            "10dBm\n"
                            "12dBm\n"
                            "17dBm\n"
                            "20dBm\n"
                            "22dBmn"
                           );
    lv_obj_add_flag(dd, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(dd, 170, 50);
    lv_dropdown_set_selected(dd, 6);
    lv_obj_add_event_cb(dd, radio_power_cb,
                        LV_EVENT_VALUE_CHANGED
                        , NULL);


    dd = lv_dropdown_create(cont1);
    lv_dropdown_set_options(dd, "100ms\n"
                            "200ms\n"
                            "500ms\n"
                            "1000ms\n"
                            "2000ms\n"
                            "3000ms"
                           );
    lv_dropdown_set_selected(dd, 1);
    lv_obj_add_flag(dd, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_size(dd, 170, 50);
    lv_obj_add_event_cb(dd, radio_tx_interval_cb,
                        LV_EVENT_VALUE_CHANGED
                        , NULL);

}

void lv_player_event_cb(lv_event_t *e)
{
#ifdef ENABLE_PLAYER
    Serial.println("Running mp3 player!");
    out->SetGain(0.3);
    file->open(AUDIO_DATA, sizeof(AUDIO_DATA));
    mp3->begin(id3, out);
    player_task_cb = playMP3;
    vTaskResume(playerTaskHandler);
#endif
}

void lv_play_record_event_cb(lv_event_t *e)
{
#ifdef ENABLE_PLAYER
    Serial.println("Running wav player!");
    out->SetGain(1);
    file_fs->open(DEFAULT_RECORD_FILENAME);
    wav->begin(file_fs, out);
    player_task_cb = playWAV;
    vTaskResume(playerTaskHandler);
#endif
}

static void progressBarSubscriberCB(lv_event_t *e)
{
    lv_obj_t *arc = lv_event_get_target(e);
    lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
    lv_msg_t *msg = lv_event_get_msg(e);

    if (msg->id == LVGL_MESSAGE_PROGRESS_CHANGED_ID ) {
        int  *percentage = (int *)lv_msg_get_payload(msg);
        if (percentage) {
            lv_arc_set_value(arc, *percentage);
            lv_label_set_text_fmt(label, "%d%%", *percentage);
            if (*percentage == 100) {
                lv_msg_unsubscribe_obj(LVGL_MESSAGE_PROGRESS_CHANGED_ID, arc);
                lv_obj_del(lv_obj_get_parent(arc));
                lv_disp_trig_activity(NULL);
            }
        }
    }
}

void createProgressBar(lv_obj_t *parent)
{

    /*Create a transition*/
    static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, LV_STYLE_PROP_INV};
    static lv_style_transition_dsc_t transition_dsc;
    lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 300, 0, NULL);


    static lv_style_t style_indicator;
    static lv_style_t style_knob;


    lv_style_init(&style_indicator);
    lv_style_set_bg_opa(&style_indicator, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indicator, DEFAULT_COLOR);
    lv_style_set_radius(&style_indicator, LV_RADIUS_CIRCLE);
    lv_style_set_transition(&style_indicator, &transition_dsc);

    lv_style_init(&style_knob);
    lv_style_set_bg_opa(&style_knob, LV_OPA_COVER);
    lv_style_set_bg_color(&style_knob, DEFAULT_COLOR);
    lv_style_set_border_color(&style_knob, lv_palette_darken(LV_PALETTE_YELLOW, 2));
    lv_style_set_border_width(&style_knob, 2);
    lv_style_set_radius(&style_knob, LV_RADIUS_CIRCLE);
    lv_style_set_pad_all(&style_knob, 6); /*Makes the knob larger*/
    lv_style_set_transition(&style_knob, &transition_dsc);

    ////////////////////
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(cont, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_100, LV_PART_MAIN);
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);

    lv_obj_t *arc = lv_arc_create(cont);
    lv_obj_set_style_arc_color(arc, DEFAULT_COLOR, LV_PART_MAIN);
    // lv_obj_set_style_arc_color(arc, DEFAULT_COLOR, LV_PART_INDICATOR);

    // lv_obj_add_style(arc, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(arc, &style_knob, LV_PART_KNOB);
    lv_obj_set_size(arc, 150, 150);
    lv_arc_set_rotation(arc, 180);
    lv_arc_set_bg_angles(arc, 0, 360);
    lv_arc_set_value(arc, 0);
    lv_obj_center(arc);

    lv_obj_t *label = lv_label_create(cont);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(cont, DEFAULT_COLOR, LV_PART_MAIN);
    lv_obj_center(label);

    lv_obj_add_event_cb(arc, progressBarSubscriberCB, LV_EVENT_MSG_RECEIVED, label);
    lv_msg_subsribe_obj(LVGL_MESSAGE_PROGRESS_CHANGED_ID, arc, NULL);
}

void lv_record_event_cb(lv_event_t *e)
{
    Serial.println("Started Recording...");
    recordFlag = true;
    createProgressBar(lv_scr_act());
    lv_disp_trig_activity(NULL);
}

void createButton(lv_obj_t *parent, const char *txt, lv_event_cb_t event_cb)
{
    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_text_color(&cont_style, DEFAULT_COLOR);

    lv_obj_t *label_cont = lv_obj_create(parent);
    lv_obj_set_size(label_cont, 210, 90);
    lv_obj_set_scrollbar_mode(label_cont, LV_SCROLLBAR_MODE_OFF);
    // lv_obj_set_flex_flow(label_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(label_cont, LV_DIR_NONE);
    lv_obj_set_style_pad_top(label_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(label_cont, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(label_cont, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(label_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_border_color(label_cont, DEFAULT_COLOR, LV_PART_MAIN);


    lv_obj_t *label = lv_label_create(label_cont);
    lv_label_set_text(label, txt);
    lv_obj_set_style_text_font(label, &font_siegra, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, DEFAULT_COLOR, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 5, 0);

    lv_obj_t *btn1 = lv_btn_create(label_cont);
    lv_obj_remove_style_all(btn1);                          /*Remove the button_default_style coming from the theme*/
    lv_obj_add_style(btn1, &button_default_style, LV_PART_MAIN);
    lv_obj_add_style(btn1, &button_press_style, LV_STATE_PRESSED);
    lv_obj_set_size(btn1, 160, 30);
    lv_obj_align_to(btn1, label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 3);

    label = lv_label_create(btn1);
    lv_label_set_text(label, txt);
    lv_obj_align_to(label, btn1, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn1, event_cb, LV_EVENT_CLICKED, NULL);
}

void musicPlay(lv_obj_t *parent)
{
    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_line_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_border_width(&cont_style, 0);
    lv_style_set_text_color(&cont_style, DEFAULT_COLOR);
    // lv_style_set_text_color(&cont_style, lv_color_white());

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_add_style(cont, &cont_style, LV_PART_MAIN);

    createButton(cont, "Play", lv_player_event_cb);
    createButton(cont, "Record", lv_record_event_cb);
    createButton(cont, "Play record", lv_play_record_event_cb);
}

void lv_ir_sender_cb(lv_event_t *e)
{
#ifdef ENABLE_IR_SENDER
    Serial.println("NEC");
    irsend.sendNEC(0x12345678);
#endif
}

void irRemoteVeiw(lv_obj_t *parent)
{
    /*Properties to transition*/
    static lv_style_prop_t props[] = {
        LV_STYLE_TRANSFORM_WIDTH,
        LV_STYLE_TRANSFORM_HEIGHT,
        LV_STYLE_TEXT_LETTER_SPACE,
        LV_STYLE_PROP_INV
    };

    /*Transition descriptor when going back to the default state.
     *Add some delay to be sure the press transition is visible even if the press was very short*/
    static lv_style_transition_dsc_t transition_dsc_def;
    lv_style_transition_dsc_init(&transition_dsc_def, props, lv_anim_path_overshoot, 250, 100, NULL);

    /*Transition descriptor when going to pressed state.
     *No delay, go to presses state immediately*/
    static lv_style_transition_dsc_t transition_dsc_pr;
    lv_style_transition_dsc_init(&transition_dsc_pr, props, lv_anim_path_ease_in_out, 250, 0, NULL);

    /*Add only the new transition to he default state*/
    static lv_style_t style_def;
    lv_style_init(&style_def);
    lv_style_set_transition(&style_def, &transition_dsc_def);
    lv_style_set_bg_color(&style_def, lv_palette_main(LV_PALETTE_YELLOW));
    lv_style_set_bg_grad_color(&style_def, lv_palette_darken(LV_PALETTE_YELLOW, 2));

    /*Add the transition and some transformation to the presses state.*/
    static lv_style_t style_pr;
    lv_style_init(&style_pr);
    lv_style_set_transform_width(&style_pr, 10);
    lv_style_set_transform_height(&style_pr, -10);
    lv_style_set_text_letter_space(&style_pr, 10);
    lv_style_set_transition(&style_pr, &transition_dsc_pr);
    lv_style_set_bg_color(&style_pr, lv_palette_darken(LV_PALETTE_YELLOW, 2));
    lv_style_set_bg_grad_color(&style_pr, lv_palette_darken(LV_PALETTE_YELLOW, 4));


    lv_obj_t *ir_btn = lv_btn_create(parent);
    lv_obj_set_width(ir_btn, LV_PCT(60));
    lv_obj_add_style(ir_btn, &style_pr, LV_STATE_PRESSED);
    lv_obj_add_style(ir_btn, &style_def, LV_PART_MAIN);
    lv_obj_t *label = lv_label_create(ir_btn);
    lv_label_set_text(label, "Send IR signal");
    lv_obj_add_event_cb(ir_btn, lv_ir_sender_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_center(label);
    lv_obj_align(ir_btn, LV_ALIGN_CENTER, 0, 0);

    vad_btn = lv_btn_create(parent);
    lv_obj_set_width(vad_btn, LV_PCT(60));
    lv_obj_add_style(vad_btn, &style_pr, LV_STATE_PRESSED);
    lv_obj_add_style(vad_btn, &style_def, LV_PART_MAIN);
    lv_obj_add_flag(vad_btn, LV_OBJ_FLAG_CHECKABLE);


    vad_btn_label = lv_label_create(vad_btn);
    lv_label_set_text(vad_btn_label, "VAD detect");

    lv_obj_add_event_cb(vad_btn, [](lv_event_t *e) {
        lv_event_code_t code = lv_event_get_code(e);
        lv_obj_t *vad_btn = lv_event_get_target(e);
        lv_obj_t *vad_btn_label = (lv_obj_t *)lv_event_get_user_data(e);
        if (code == LV_EVENT_CLICKED) {
            lv_state_t state =    lv_obj_get_state(vad_btn);
            if (state == 2) {
                vTaskSuspend(vadTaskHandler);
                lv_label_set_text(vad_btn_label, "VAD detect");
            } else {
                vad_detected_counter = 0;
                vTaskResume(vadTaskHandler);
            }
        }
    }, LV_EVENT_ALL, vad_btn_label);
    lv_obj_center(vad_btn_label);
    lv_obj_align_to(vad_btn, ir_btn, LV_ALIGN_OUT_TOP_MID, 0, -30);


    lv_obj_t *vibrate_btn = lv_btn_create(parent);
    lv_obj_set_width(vibrate_btn, LV_PCT(60));
    lv_obj_add_style(vibrate_btn, &style_pr, LV_STATE_PRESSED);
    lv_obj_add_style(vibrate_btn, &style_def, LV_PART_MAIN);
    label = lv_label_create(vibrate_btn);
    lv_label_set_text(label, "Vibrate");
    lv_obj_add_event_cb(vibrate_btn, [](lv_event_t *e) {
        watch.setWaveform(0, 78);
        watch.run();
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_center(label);
    lv_obj_align_to(vibrate_btn, ir_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);

    lv_align_t alignt_array[] = {LV_ALIGN_TOP_LEFT, LV_ALIGN_TOP_MID, LV_ALIGN_TOP_RIGHT,
                                 LV_ALIGN_LEFT_MID, LV_ALIGN_RIGHT_MID,
                                 LV_ALIGN_BOTTOM_LEFT, LV_ALIGN_BOTTOM_MID, LV_ALIGN_BOTTOM_RIGHT
                                };

    // Test touch boundaries
    for (int i = 0; i < 8; ++i) {
        lv_obj_t *btn1 = lv_btn_create(parent);
        lv_obj_add_event_cb(btn1,  [](lv_event_t *e) {
            watch.setWaveform(0, 78);
            watch.run();
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_add_style(btn1, &style_pr, LV_STATE_PRESSED);
        lv_obj_add_style(btn1, &style_def, LV_PART_MAIN);
        lv_obj_align(btn1, alignt_array[i], 0, 0);
        label = lv_label_create(btn1);
        lv_label_set_text_fmt(label, "%d", i);
        lv_obj_center(label);
    }



}

static void lv_spinbox_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        bool *inc =  (bool *)lv_event_get_user_data(e);
        lv_obj_t *target = lv_event_get_current_target(e);
        lv_datetime_t *datetime_obj =  (lv_datetime_t *)lv_obj_get_user_data(target);
        if (!datetime_obj) {
            Serial.println("datetime_obj is null");
            return;
        }
        Serial.print(datetime_obj->name);

        if (*inc) {
            lv_spinbox_increment(datetime_obj->obj);
        } else {
            lv_spinbox_decrement(datetime_obj->obj);
        }

    }
}

lv_obj_t *createAdjustButton(lv_obj_t *parent, const char *txt, lv_event_cb_t event_cb, void *user_data)
{
    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_text_color(&cont_style, DEFAULT_COLOR);

    lv_obj_t *label_cont = lv_obj_create(parent);
    lv_obj_set_size(label_cont, 210, 90);
    lv_obj_set_scrollbar_mode(label_cont, LV_SCROLLBAR_MODE_OFF);
    // lv_obj_set_flex_flow(label_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(label_cont, LV_DIR_NONE);
    lv_obj_set_style_pad_top(label_cont, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(label_cont, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(label_cont, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(label_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_border_color(label_cont, DEFAULT_COLOR, LV_PART_MAIN);

    lv_obj_t *label = lv_label_create(label_cont);
    lv_label_set_text(label, txt);
    lv_obj_set_style_text_font(label, &font_siegra, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, DEFAULT_COLOR, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 5, 0);


    lv_obj_t *cont = lv_obj_create(label_cont);
    lv_obj_set_size(cont, 185, 45);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_dir(cont, LV_DIR_NONE);
    lv_obj_align_to(cont, label, LV_ALIGN_OUT_BOTTOM_LEFT, -6, 5);

    lv_obj_set_style_pad_all(cont, 1, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);

    lv_coord_t w = 50;
    lv_coord_t h = 40;
    lv_obj_t *btn = lv_btn_create(cont);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_style(btn, &button_default_style, LV_PART_MAIN);
    lv_obj_add_style(btn, &button_press_style, LV_STATE_PRESSED);



    static bool increment = 1;
    static bool decrement = 0;
    lv_obj_set_user_data(btn, user_data);
    lv_obj_add_event_cb(btn, event_cb, LV_EVENT_ALL, &increment);

    lv_obj_t *spinbox = lv_spinbox_create(cont);
    lv_spinbox_set_step(spinbox, 1);
    lv_spinbox_set_rollover(spinbox, false);
    lv_spinbox_set_cursor_pos(spinbox, 0);

    if (user_data) {
        lv_datetime_t *datetime_obj = (lv_datetime_t *)user_data;
        lv_spinbox_set_digit_format(spinbox, datetime_obj->digitFormat, 0);
        lv_spinbox_set_range(spinbox, datetime_obj->minVal, datetime_obj->maxVal);
        lv_spinbox_set_value(spinbox, datetime_obj->defaultVal);
    }
    lv_obj_set_width(spinbox, 65);
    lv_obj_set_height(spinbox, h + 2);

    lv_obj_set_style_bg_opa(spinbox, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_text_color(spinbox, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_color(spinbox, DEFAULT_COLOR, LV_PART_MAIN);
    lv_obj_set_style_text_font(spinbox, &font_sandbox, LV_PART_MAIN);

    // lv_obj_set_style_bg_opa(spinbox, LV_OPA_TRANSP, LV_PART_SELECTED);
    // lv_obj_set_style_bg_opa(spinbox, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(spinbox, LV_OPA_TRANSP, LV_PART_CURSOR);


    btn = lv_btn_create(cont);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_style(btn, &button_default_style, LV_PART_MAIN);
    lv_obj_add_style(btn, &button_press_style, LV_STATE_PRESSED);
    lv_obj_set_user_data(btn, user_data);
    // lv_obj_add_event_cb(btn, lv_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(btn, event_cb, LV_EVENT_ALL, &decrement);

    return spinbox;
}

static void datetime_event_handler(lv_event_t *e)
{
    Serial.println("Save setting datetime.");
    int32_t year =  lv_spinbox_get_value(lv_datetime[0].obj);
    int32_t month =  lv_spinbox_get_value(lv_datetime[1].obj);
    int32_t day =  lv_spinbox_get_value(lv_datetime[2].obj);
    int32_t hour =  lv_spinbox_get_value(lv_datetime[3].obj);
    int32_t minute =  lv_spinbox_get_value(lv_datetime[4].obj);
    int32_t second =  lv_spinbox_get_value(lv_datetime[5].obj);

    Serial.printf("Y=%dM=%dD=%d H:%dM:%dS:%d\n", year, month, day,
                  hour, minute, second);

    watch.setDateTime(year, month, day, hour, minute, second);

    // Reading time synchronization from RTC to system time
    watch.hwClockRead();
}

void datetimeVeiw(lv_obj_t *parent)
{
    //set default datetime
    time_t now;
    struct tm  info;
    time(&now);
    localtime_r(&now, &info);
    lv_datetime[0].defaultVal = info.tm_year + 1900;
    lv_datetime[1].defaultVal = info.tm_mon + 1;
    lv_datetime[2].defaultVal = info.tm_mday;
    lv_datetime[3].defaultVal = info.tm_hour;
    lv_datetime[4].defaultVal = info.tm_min ;
    lv_datetime[5].defaultVal = info.tm_sec ;


    static lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_bg_img_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_line_opa(&cont_style, LV_OPA_TRANSP);
    lv_style_set_border_width(&cont_style, 0);
    lv_style_set_text_color(&cont_style, DEFAULT_COLOR);

    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, lv_disp_get_hor_res(NULL), 400);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_add_style(cont, &cont_style, LV_PART_MAIN);

    for (int i = 0; i < sizeof(lv_datetime) / sizeof(lv_datetime[0]); ++i) {
        lv_datetime[i].obj =  createAdjustButton(cont, lv_datetime[i].name, lv_spinbox_event_cb, &(lv_datetime[i]));
    }

    lv_obj_t *btn_cont = lv_obj_create(cont);
    lv_obj_set_size(btn_cont, 210, 60);
    lv_obj_set_scrollbar_mode(btn_cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(btn_cont, LV_DIR_NONE);
    lv_obj_set_style_pad_top(btn_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(btn_cont, 5, LV_PART_MAIN);
    lv_obj_set_style_border_opa(btn_cont, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn_cont, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *btn = lv_btn_create(btn_cont);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_SAVE, 0);
    lv_obj_set_size(btn, 180, 50);
    lv_obj_add_style(btn, &button_default_style, LV_PART_MAIN);
    lv_obj_add_style(btn, &button_press_style, LV_STATE_PRESSED);
    lv_obj_add_event_cb(btn, datetime_event_handler, LV_EVENT_CLICKED, NULL);
}

void settingButtonStyle()
{
    /*Init the button_default_style for the default state*/
    lv_style_init(&button_default_style);

    lv_style_set_radius(&button_default_style, 3);

    lv_style_set_bg_opa(&button_default_style, LV_OPA_100);
    lv_style_set_bg_color(&button_default_style, lv_palette_main(LV_PALETTE_YELLOW));
    lv_style_set_bg_grad_color(&button_default_style, lv_palette_darken(LV_PALETTE_YELLOW, 2));
    lv_style_set_bg_grad_dir(&button_default_style, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&button_default_style, LV_OPA_40);
    lv_style_set_border_width(&button_default_style, 2);
    lv_style_set_border_color(&button_default_style, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&button_default_style, 8);
    lv_style_set_shadow_color(&button_default_style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_ofs_y(&button_default_style, 8);

    lv_style_set_outline_opa(&button_default_style, LV_OPA_COVER);
    lv_style_set_outline_color(&button_default_style, lv_palette_main(LV_PALETTE_YELLOW));

    lv_style_set_text_color(&button_default_style, lv_color_white());
    lv_style_set_pad_all(&button_default_style, 10);

    /*Init the pressed button_default_style*/
    lv_style_init(&button_press_style);

    /*Add a large outline when pressed*/
    lv_style_set_outline_width(&button_press_style, 30);
    lv_style_set_outline_opa(&button_press_style, LV_OPA_TRANSP);

    lv_style_set_translate_y(&button_press_style, 5);
    lv_style_set_shadow_ofs_y(&button_press_style, 3);
    lv_style_set_bg_color(&button_press_style, lv_palette_darken(LV_PALETTE_YELLOW, 2));
    lv_style_set_bg_grad_color(&button_press_style, lv_palette_darken(LV_PALETTE_YELLOW, 4));

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props[] = {LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, LV_STYLE_PROP_INV};
    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);

    lv_style_set_transition(&button_press_style, &trans);

}
/*
 ************************************
 *      HARDWARE SETTING            *
 ************************************
*/
void setSportsFlag()
{
    sportsIrq = true;
}

void settingSensor()
{

    //Default 4G ,200HZ
    watch.configAccelerometer();

    watch.enableAccelerometer();

    watch.enablePedometer();

    watch.configInterrupt();

    watch.enableFeature(SensorBMA423::FEATURE_STEP_CNTR |
                        SensorBMA423::FEATURE_ANY_MOTION |
                        SensorBMA423::FEATURE_NO_MOTION |
                        SensorBMA423::FEATURE_ACTIVITY |
                        SensorBMA423::FEATURE_TILT |
                        SensorBMA423::FEATURE_WAKEUP,
                        true);


    watch.enablePedometerIRQ();
    watch.enableTiltIRQ();
    watch.enableWakeupIRQ();
    watch.enableAnyNoMotionIRQ();
    watch.enableActivityIRQ();


    watch.attachBMA(setSportsFlag);

}

void setPMUFlag()
{
    pmuIrq = true;
}

void settingPMU()
{
    watch.clearPMU();

    watch.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Enable the required interrupt function
    watch.enableIRQ(
        // XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |  //POWER KEY
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
        // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
    );
    watch.attachPMU(setPMUFlag);

}

void setRadioFlag(void)
{
    radioTransmitFlag = true;
}

void settingRadio()
{
#ifdef USING_TWATCH_S3
    // set carrier frequency to 868.0 MHz
    if (watch.setFrequency(868.0) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
    }

    // set bandwidth to 250 kHz
    if (watch.setBandwidth(250.0) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
    }

    // set spreading factor to 10
    if (watch.setSpreadingFactor(10) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        Serial.println(F("Selected spreading factor is invalid for this module!"));
    }

    // set coding rate to 6
    if (watch.setCodingRate(6) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        Serial.println(F("Selected coding rate is invalid for this module!"));
    }

    // set LoRa sync word to 0xAB
    if (watch.setSyncWord(0xAB) != RADIOLIB_ERR_NONE) {
        Serial.println(F("Unable to set sync word!"));
    }

    // set output power to 10 dBm (accepted range is -17 - 22 dBm)
    if (watch.setOutputPower(22) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        Serial.println(F("Selected output power is invalid for this module!"));
    }

    // set over current protection limit to 140 mA (accepted range is 45 - 140 mA)
    // NOTE: set value to 0 to disable overcurrent protection
    if (watch.setCurrentLimit(140) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        Serial.println(F("Selected current limit is invalid for this module!"));
    }

    // set LoRa preamble length to 15 symbols (accepted range is 0 - 65535)
    if (watch.setPreambleLength(15) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
        Serial.println(F("Selected preamble length is invalid for this module!"));
    }

    // disable CRC
    if (watch.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        Serial.println(F("Selected CRC is invalid for this module!"));
    }

    // set the function that will be called
    // when new packet is received
    watch.setDio1Action(setRadioFlag);
#endif

}

void playerTask(void *params)
{
#ifdef ENABLE_PLAYER
    while (1) {
        if (player_task_cb) {
            if (!player_task_cb()) {
                vTaskSuspend(NULL);
            }
        } else {
            vTaskSuspend(NULL);
        }
        delay(5);
    }
#endif
}

static bool playMP3()
{
#ifdef ENABLE_PLAYER
    if (mp3->isRunning()) {
        if (!mp3->loop()) {
            mp3->stop();
            return false;
        }
    }
    return true;
#endif
}

static bool playWAV()
{
#ifdef ENABLE_PLAYER
    if (wav->isRunning()) {
        if (!wav->loop()) {
            wav->stop();
            return false;
        }
    }
    return true;
#endif
}


void vadTask(void *params)
{
#ifdef ENABLE_PLAYER
    vTaskSuspend(NULL);
    while (1) {
        size_t read_len = 0;
        if (watch.readMicrophone((char *) vad_buff, vad_buffer_size, &read_len)) {
            // Feed samples to the VAD process and get the result
#if   ESP_IDF_VERSION_VAL(4,4,1) == ESP_IDF_VERSION
            vad_state_t vad_state = vad_process(vad_inst, vad_buff);
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,4,1)
            vad_state_t vad_state = vad_process(vad_inst, vad_buff, MIC_I2S_SAMPLE_RATE, VAD_FRAME_LENGTH_MS);
#else
#error "No support this version."
#endif
            if (vad_state == VAD_SPEECH) {
                Serial.print(millis());
                Serial.println(" Noise detected!!!");
                if (vad_btn_label) {
                    lv_label_set_text_fmt(vad_btn_label, "Noise detected %u", vad_detected_counter++);
                }
            }
        }
        delay(5);
    }
#endif
}

void settingPlayer()
{
#ifdef ENABLE_PLAYER
    file = new AudioFileSourcePROGMEM(AUDIO_DATA, sizeof(AUDIO_DATA));
    id3 = new AudioFileSourceID3(file);
    out = new AudioOutputI2S(1, AudioOutputI2S::EXTERNAL_I2S);
    out->SetPinout(BOARD_DAC_IIS_BCK, BOARD_DAC_IIS_WS, BOARD_DAC_IIS_DOUT);
    out->SetGain(0.2);
    mp3 = new AudioGeneratorMP3();
    mp3->begin(id3, out);

    file_fs = new AudioFileSourceSPIFFS();
    wav = new AudioGeneratorWAV();
    player_task_cb = playMP3;

    xTaskCreate(playerTask, "player", 8 * 1024, NULL, 12, &playerTaskHandler);
#endif

    // Initialize esp-sr vad detected
#if ESP_IDF_VERSION_VAL(4,4,1) == ESP_IDF_VERSION
    vad_inst = vad_create(VAD_MODE_0, MIC_I2S_SAMPLE_RATE, VAD_FRAME_LENGTH_MS);
#elif  ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4,4,1)
    vad_inst = vad_create(VAD_MODE_0);
#else
#error "No support this version."
#endif
    vad_buff = (int16_t *)ps_malloc(vad_buffer_size);
    if (vad_buff == NULL) {
        while (1) {
            Serial.println("Memory allocation failed!");
            delay(1000);
        }
    }
    xTaskCreate(vadTask, "vad", 8 * 1024, NULL, 12, &vadTaskHandler);


}

void settingIRRemote()
{
#ifdef ENABLE_IR_SENDER
    irsend.begin();
#endif
}

// Create a file and add wav header to it so we can play it from PC later
static bool CreateWAV(const char *song_name, uint32_t duration, uint16_t num_channels, const uint32_t sampling_rate, uint16_t bits_per_sample)
{
    // data size in bytes - > this amount of data should be recorded from microphone later
    uint32_t data_size = sampling_rate * num_channels * bits_per_sample * duration / 8;

    File new_audio_file = SPIFFS.open(song_name, FILE_WRITE);
    if (!new_audio_file) {
        Serial.println("Failed to create file");
        return false;
    }

    /* *************** ADD ".WAV" HEADER *************** */
    uint8_t CHUNK_ID[4] = {'R', 'I', 'F', 'F'};
    new_audio_file.write(CHUNK_ID, 4);

    uint32_t chunk_size = data_size + 36;
    uint8_t CHUNK_SIZE[4] = {
        (uint8_t)(chunk_size),
        (uint8_t)(chunk_size >> 8),
        (uint8_t)(chunk_size >> 16),
        (uint8_t)(chunk_size >> 24)
    };
    new_audio_file.write(CHUNK_SIZE, 4);

    uint8_t FORMAT[4] = {'W', 'A', 'V', 'E'};
    new_audio_file.write(FORMAT, 4);

    uint8_t SUBCHUNK_1_ID[4] = {'f', 'm', 't', ' '};
    new_audio_file.write(SUBCHUNK_1_ID, 4);

    uint8_t SUBCHUNK_1_SIZE[4] = {0x10, 0x00, 0x00, 0x00};
    new_audio_file.write(SUBCHUNK_1_SIZE, 4);

    uint8_t AUDIO_FORMAT[2] = {0x01, 0x00};
    new_audio_file.write(AUDIO_FORMAT, 2);

    uint8_t NUM_CHANNELS[2] = {
        (uint8_t)num_channels,
        (uint8_t)(num_channels >> 8)
    };
    new_audio_file.write(NUM_CHANNELS, 2);

    uint8_t SAMPLING_RATE[4] = {
        (uint8_t)(sampling_rate),
        (uint8_t)(sampling_rate >> 8),
        (uint8_t)(sampling_rate >> 16),
        (uint8_t)(sampling_rate >> 24)
    };
    new_audio_file.write(SAMPLING_RATE, 4);

    uint32_t byte_rate = num_channels * sampling_rate * bits_per_sample / 8;
    uint8_t BYTE_RATE[4] = {
        (uint8_t)byte_rate,
        (uint8_t)(byte_rate >> 8),
        (uint8_t)(byte_rate >> 16),
        (uint8_t)(byte_rate >> 24)
    };
    new_audio_file.write(BYTE_RATE, 4);

    uint16_t block_align = num_channels * bits_per_sample / 8;
    uint8_t BLOCK_ALIGN[2] = {
        (uint8_t)block_align,
        (uint8_t)(block_align >> 8)
    };
    new_audio_file.write(BLOCK_ALIGN, 2);

    uint8_t BITS_PER_SAMPLE[2] = {
        (uint8_t)bits_per_sample,
        (uint8_t)(bits_per_sample >> 8)
    };
    new_audio_file.write(BITS_PER_SAMPLE, 2);

    uint8_t SUBCHUNK_2_ID[4] = {'d', 'a', 't', 'a'};
    new_audio_file.write(SUBCHUNK_2_ID, 4);

    uint8_t SUBCHUNK_2_SIZE[4] = {
        (uint8_t)(data_size),
        (uint8_t)(data_size >> 8),
        (uint8_t)(data_size >> 16),
        (uint8_t)(data_size >> 24)
    };
    new_audio_file.write(SUBCHUNK_2_SIZE, 4);

    // Actual data should be appended after this point later

    new_audio_file.close();
    return true;
}

static void PDM_Record(const char *song_name, uint32_t duration)
{
    // Add wav header to the file so we can play it from PC later
    if (!CreateWAV(song_name, duration, 1, MIC_I2S_SAMPLE_RATE, MIC_I2S_BITS_PER_SAMPLE)) {
        Serial.println("Error during wav header creation");
        return;
    }

    Serial.println("Create wav file succssed!");

    // Buffer to receive data from microphone
    const size_t BUFFER_SIZE = 500;
    uint8_t *buf = (uint8_t *)malloc(BUFFER_SIZE);
    if (!buf) {
        Serial.println("Failed to alloc memory");
        return;
    }

    // Open created .wav file in append+binary mode to add PCM data
    File audio_file = SPIFFS.open(song_name, FILE_APPEND);
    if (!audio_file) {
        Serial.println("Failed to create file");
        return;
    }

    // data size in bytes - > this amount of data should be recorded from microphone
    uint32_t data_size = MIC_I2S_SAMPLE_RATE * MIC_I2S_BITS_PER_SAMPLE * duration / 8;

    // Record until "file_size" bytes have been read from mic.
    uint32_t counter = 0;
    uint32_t bytes_written;
    Serial.println("Recording started");
    int percentage = 0;

    while (counter != data_size) {

        percentage = ((float)counter / (float)data_size) * 100;
        Serial.print(percentage);
        Serial.println("%");

        lv_msg_send(LVGL_MESSAGE_PROGRESS_CHANGED_ID, &percentage);

        // Check for file size overflow
        if (counter > data_size) {
            Serial.println("File is corrupted. data_size must be multiple of BUFFER_SIZE. Please modify BUFFER_SIZE");
            break;
        }

        // Read data from microphone
        if (!watch.readMicrophone(buf, BUFFER_SIZE, &bytes_written)) {
            Serial.println("readMicrophone() error");
        }


        if (bytes_written != BUFFER_SIZE) {
            Serial.println("Bytes written error");
        }

        // Save data to SPIFFS
        audio_file.write( buf, BUFFER_SIZE);

        // Increment the counter
        counter += BUFFER_SIZE;

        lv_task_handler();
    }
    percentage = 100;
    lv_msg_send(LVGL_MESSAGE_PROGRESS_CHANGED_ID, &percentage);
    Serial.println("Recording finished");

    audio_file.close();
    free(buf);
}


// ESP-NOW setup
void espnowinit() 
{
     WiFi.mode(WIFI_STA);

     esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
       esp_wifi_set_ps(WIFI_PS_NONE);

// Automatically connect to the autopilot on the boat (do I need this? maybe not if it's mac address based)
//const char *ssid = "PlotDevice";
//const char *password = "thereyouare";
    //WiFi.begin(ssid, password);

//combine WIFI and ESP-NOW channels so they can run simultaneously
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

  if ( esp_now_init() != ESP_OK ) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

/**/
// ESP-NOW Callback when data is sent
void OnDataSent(const uint8_t *broadcastAddress, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail (shit)");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}


 //ESP-NOW Callback when data is received

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  //incomingHeading = incomingReadings.HDG;
  incomingHTS = incomingReadings.HTS;
  APState = incomingReadings.STATE;

autopilot_main_update_label(); 
  //String headingstring = String(incomingHeading, 0);
 // String HTSstring = String(incomingHTS, 0);
}


// ESP-NOW Wifi send-receive handler
static void autopilot_wifi_event(){

           esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &APCommand, sizeof(APCommand));
             //esp_err_t result = esp_now_send(broadcastAddress, reinterpret_cast<uint8_t*>(&APCommand), sizeof(APCommand));
             if (result == ESP_OK) {
               Serial.println("ap wifi event Sent with success");
             }
             else {
               Serial.println("Error sending ap wifi event");
             }


//WiFi.mode(WIFI_OFF);

 //update State and HTS in display
//autopilot_main_update_label();  // duplicative?

//esp_wifi_stop();

}



///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
//#if LV_COLOR_16_SWAP !=1
  //  #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
//#endif


void callautopilot()
{

 // send initial ESP-NOW handshake
        APbutton = 6;
        APCommand.doit = APbutton;

esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &APCommand, sizeof(APCommand));
              if (result == ESP_OK) {
               Serial.println("Sent callautopilot with success");
             }
             else {
               Serial.println("Error sending callautopilot");
             }

autopilot_main_update_label();

}
///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////

void ui_event_toggle( lv_event_t * e) 
{
  ToggleON( e );
}

void ui_event_down10( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      down10button( e );
}
}
void ui_event_down1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      down1button( e );
}
}
void ui_event_up1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      up1button( e );
}
}
void ui_event_up10( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      up10button( e );
}
}

void ui_event_connect( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      connectbutton( e );
}
}
///////////////////// SCREENS ////////////////////




void autopilot_main_update_label() //displays updated info in the watch face. Need to gather that data from the pilot via wifi.
{
  //  snprintf(heading, sizeof( heading ), "%.1f°", incomingHeading);
    snprintf(heading_to_steer, sizeof( heading_to_steer ), "%0.1f°", incomingHTS);
    //snprintf(incAPstate, sizeof (incAPstate), "%s", APState);     
     // lv_label_set_text_fmt(ui_Togglelabel, "%s", incAPstate);
      lv_label_set_text( ui_CTSfield, heading_to_steer );
     
     
      if (APState == 1) {
lv_label_set_text(ui_Togglelabel,"ON");}
else {
lv_label_set_text(ui_Togglelabel, "OFF");}

}


//void ui_APFace_screen_init(lv_obj_t *parent)
void autopilot(lv_obj_t *parent)
{
//experiment
WiFi.mode(WIFI_STA);

lv_disp_t *dispp = lv_disp_get_default();
lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
lv_disp_set_theme(dispp, theme);


lv_obj_t *ui_APFace = lv_obj_create(parent);
//lv_obj_clear_flag( ui_APFace, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM );    /// Flags
 lv_obj_set_size(ui_APFace, 240, 240);
lv_obj_set_style_bg_color(ui_APFace, lv_color_hex(0x2A0A03), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_APFace, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_APFace, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_APFace, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_APFace, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_APFace, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_APFace, 0, LV_PART_MAIN| LV_STATE_DEFAULT);


ui_toggle = lv_btn_create(ui_APFace);
lv_obj_set_width( ui_toggle, 120);
lv_obj_set_height( ui_toggle, 40);
lv_obj_set_x( ui_toggle, 0 );
lv_obj_set_y( ui_toggle, 25 );
lv_obj_set_align( ui_toggle, LV_ALIGN_CENTER );
//lv_obj_add_state( ui_toggle, LV_STATE_CHECKED );     /// States
//lv_obj_set_style_bg_color(ui_toggle, lv_color_hex(0x383838), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_toggle, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

//lv_obj_set_style_bg_color(ui_toggle, lv_color_hex(0x458A8A), LV_PART_INDICATOR | LV_STATE_CHECKED );
//lv_obj_set_style_bg_opa(ui_toggle, 255, LV_PART_INDICATOR| LV_STATE_CHECKED);

lv_obj_set_style_bg_color(ui_toggle, lv_color_hex(0x383838), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_down10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_toggle, lv_color_hex(0x8A8A8A), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_border_opa(ui_down10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_toggle, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_toggle, lv_color_hex(0x458A8A), LV_PART_MAIN | LV_STATE_PRESSED );
//lv_obj_set_style_bg_opa(ui_down10, 255, LV_PART_MAIN| LV_STATE_PRESSED);
//////////

ui_Togglelabel = lv_label_create(ui_toggle);
lv_obj_set_width( ui_Togglelabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Togglelabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Togglelabel, -2 );
lv_obj_set_y( ui_Togglelabel, -1 );
lv_obj_set_align( ui_Togglelabel, LV_ALIGN_CENTER );

if (APState == 1) {
lv_label_set_text(ui_Togglelabel,"ON");}
else {
lv_label_set_text(ui_Togglelabel, "OFF");}

lv_obj_set_style_text_color(ui_Togglelabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Togglelabel, &lv_font_montserrat_36, LV_PART_MAIN| LV_STATE_DEFAULT);

/////////

ui_CTSlabel = lv_label_create(ui_APFace);
lv_obj_set_width( ui_CTSlabel, 105);
lv_obj_set_height( ui_CTSlabel, LV_SIZE_CONTENT);   /// 70
lv_obj_set_x( ui_CTSlabel, 0 );
lv_obj_set_y( ui_CTSlabel, -48 );
lv_obj_set_align( ui_CTSlabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_CTSlabel,"Course");
lv_obj_clear_flag( ui_CTSlabel, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_CTSlabel, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_CTSlabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_CTSlabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_CTSlabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_CTSlabel, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);
//lv_obj_set_style_bg_color(ui_CTSlabel, lv_color_hex(0x2A0A03), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_CTSlabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_CTSlabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_CTSlabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_CTSlabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_CTSlabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_CTSlabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);


ui_CTSfield = lv_label_create(ui_APFace);
lv_obj_set_width( ui_CTSfield, 150);
lv_obj_set_height( ui_CTSfield, LV_SIZE_CONTENT);   /// 70
lv_obj_set_x( ui_CTSfield, 0 );
lv_obj_set_y( ui_CTSfield, -20 );
lv_obj_set_align( ui_CTSfield, LV_ALIGN_CENTER );
lv_label_set_text(ui_CTSfield, "---");
//lv_textarea_set_placeholder_text(ui_CTSfield,"---");
lv_obj_clear_flag( ui_CTSfield, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_CTSfield, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_text_color(ui_CTSfield, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_CTSfield, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_align(ui_CTSfield, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_CTSfield, &lv_font_montserrat_36, LV_PART_MAIN| LV_STATE_DEFAULT);
//lv_obj_set_style_bg_color(ui_CTSfield, lv_color_hex(0x2A0A03), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_CTSfield, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_CTSfield, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
//lv_obj_set_style_pad_left(ui_CTSfield, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
//lv_obj_set_style_pad_right(ui_CTSfield, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
//lv_obj_set_style_pad_top(ui_CTSfield, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
//lv_obj_set_style_pad_bottom(ui_CTSfield, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_down10 = lv_btn_create(ui_APFace);
lv_obj_set_width( ui_down10, 80);
lv_obj_set_height( ui_down10, 40);
lv_obj_set_x( ui_down10, -75 );
lv_obj_set_y( ui_down10, 94 );
lv_obj_set_align( ui_down10, LV_ALIGN_CENTER );
//lv_obj_add_flag( ui_down10, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
//lv_obj_clear_flag( ui_down10, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_down10, lv_color_hex(0x383838), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_down10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_down10, lv_color_hex(0x8A8A8A), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_border_opa(ui_down10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_down10, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_down10, lv_color_hex(0x458A8A), LV_PART_MAIN | LV_STATE_PRESSED );
//lv_obj_set_style_bg_opa(ui_down10, 255, LV_PART_MAIN| LV_STATE_PRESSED);

ui_Label1 = lv_label_create(ui_down10);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label1, -2 );
lv_obj_set_y( ui_Label1, -1 );
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"<<");
lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_down1 = lv_btn_create(ui_APFace);
lv_obj_set_width( ui_down1, 80);
lv_obj_set_height( ui_down1, 40);
lv_obj_set_x( ui_down1, -75 );
lv_obj_set_y( ui_down1, -94 );
lv_obj_set_align( ui_down1, LV_ALIGN_CENTER );
//lv_obj_add_flag( ui_down1, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_down1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_down1, lv_color_hex(0x383838), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_down1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_down1, lv_color_hex(0x8A8A8A), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_border_opa(ui_down1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_down1, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_down1, lv_color_hex(0x458A8A), LV_PART_MAIN | LV_STATE_PRESSED );
//lv_obj_set_style_bg_opa(ui_down1, 255, LV_PART_MAIN| LV_STATE_PRESSED);

ui_Label2 = lv_label_create(ui_down1);
lv_obj_set_width( ui_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label2, -2 );
lv_obj_set_y( ui_Label2, -1 );
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"<");
lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_up1 = lv_btn_create(ui_APFace);
lv_obj_set_width( ui_up1, 80);
lv_obj_set_height( ui_up1, 40);
lv_obj_set_x( ui_up1, 75 );
lv_obj_set_y( ui_up1, -94 );
lv_obj_set_align( ui_up1, LV_ALIGN_CENTER );
//lv_obj_add_flag( ui_up1, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_up1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_up1, lv_color_hex(0x383838), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_up1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_up1, lv_color_hex(0x8A8A8A), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_border_opa(ui_up1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_up1, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_up1, lv_color_hex(0x458A8A), LV_PART_MAIN | LV_STATE_PRESSED );
lv_obj_set_style_bg_opa(ui_up1, 255, LV_PART_MAIN| LV_STATE_PRESSED);

ui_Label3 = lv_label_create(ui_up1);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label3, -2 );
lv_obj_set_y( ui_Label3, -1 );
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,">");
lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label3, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_up10 = lv_btn_create(ui_APFace);
lv_obj_set_width( ui_up10, 80);
lv_obj_set_height( ui_up10, 40);
lv_obj_set_x( ui_up10, 75 );
lv_obj_set_y( ui_up10, 94 );
lv_obj_set_align( ui_up10, LV_ALIGN_CENTER );
//lv_obj_add_flag( ui_up10, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_up10, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_up10, lv_color_hex(0x383838), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_up10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_up10, lv_color_hex(0x8A8A8A), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_border_opa(ui_up10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_up10, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_up10, lv_color_hex(0x458A8A), LV_PART_MAIN | LV_STATE_PRESSED );
//lv_obj_set_style_bg_opa(ui_up10, 255, LV_PART_MAIN| LV_STATE_PRESSED);

ui_Label4 = lv_label_create(ui_up10);
lv_obj_set_width( ui_Label4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label4, -2 );
lv_obj_set_y( ui_Label4, -1 );
lv_obj_set_align( ui_Label4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label4,">>");
lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label4, &lv_font_montserrat_48, LV_PART_MAIN| LV_STATE_DEFAULT);


ui_connect = lv_btn_create(ui_APFace);
lv_obj_set_width( ui_connect, 25);
lv_obj_set_height( ui_connect, 50);
//lv_obj_set_x( ui_connect, 75 );
//lv_obj_set_y( ui_connect, 94 );
lv_obj_set_align( ui_connect, LV_ALIGN_BOTTOM_MID );
//lv_obj_add_flag( ui_up10, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_up10, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_connect, lv_color_hex(0x383838), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_bg_opa(ui_up10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_connect, lv_color_hex(0x8A8A8A), LV_PART_MAIN | LV_STATE_DEFAULT );
//lv_obj_set_style_border_opa(ui_up10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_connect, 1, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_connect, lv_color_hex(0x458A8A), LV_PART_MAIN | LV_STATE_PRESSED );
//lv_obj_set_style_bg_opa(ui_up10, 255, LV_PART_MAIN| LV_STATE_PRESSED);



lv_obj_add_event_cb(ui_toggle, ui_event_toggle, LV_EVENT_CLICKED, NULL);  //CHANGED FROM ALL TO CLICKED
lv_obj_add_event_cb(ui_down10, ui_event_down10, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_down1, ui_event_down1, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_up1, ui_event_up1, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_up10, ui_event_up10, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_connect, ui_event_connect, LV_EVENT_CLICKED, NULL);


}

//----------- AUTOPILOT UI EVENTS  ----------------//

void ToggleON(lv_event_t * e)
{
        APbutton = 1;
        APCommand.doit = 1;
         
        // Send message via ESP-NOW
        autopilot_wifi_event();
}

void ToggleOFF(lv_event_t * e)
{
        APbutton = 1;
        APCommand.doit = 1;
         
        // Send message via ESP-NOW
        autopilot_wifi_event();

}

void up1button(lv_event_t * e)
{
        APbutton = 2;
        APCommand.doit = 2;
         
        // Send message via ESP-NOW
        autopilot_wifi_event();
}

void down1button(lv_event_t * e)
{
        APbutton = 3;
        APCommand.doit = APbutton;
         
        // Send message via ESP-NOW
        autopilot_wifi_event();
}


void up10button(lv_event_t * e)
{
        APbutton = 4;
        APCommand.doit = APbutton;
         
        // Send message via ESP-NOW
        autopilot_wifi_event();
}

void down10button(lv_event_t * e)
{
           APbutton = 5;
        APCommand.doit = APbutton;
         
        // Send message via ESP-NOW
        autopilot_wifi_event();

   }

void connectbutton(lv_event_t * e)
{
        callautopilot();
}





