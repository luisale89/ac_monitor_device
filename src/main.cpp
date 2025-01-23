// logging
#include <esp_log.h>
static const char* TAG = "main";
// libraries
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <SPIFFS.h>
#include <secrets.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

//Monitor Pinout
#define AMBIENT_TEMP 19
#define DISCHARGE_TEMP 18
#define LIQUID_TEMP 17
#define VAPOR_TEMP 16
#define NETWORK_LED 26
#define ALARM_RELAY 25
#define ALARM_LED 32
#define BRKR_LED 27
#define ALARM_RESET 35
#define NOW_CNF 14
#define CONTROL_VOLTAGE_PRESENCE 33 // input that indicates if the ON signal to the compressor is present.

// Set your Board ID 
#define BOARD_ID 2
#define MAX_CHANNEL 11  // for North America

// oneWire and DallasTemperature
OneWire ow_ambient_temp(AMBIENT_TEMP);
OneWire ow_discharge_temp(DISCHARGE_TEMP);
OneWire ow_liquid_temp(LIQUID_TEMP);
OneWire ow_vapor_temp(VAPOR_TEMP);
DallasTemperature ambient_temp_sensor(&ow_ambient_temp);
DallasTemperature discharge_temp_sensor(&ow_discharge_temp);
DallasTemperature liquid_temp_sensor(&ow_liquid_temp);
DallasTemperature vapor_temp_sensor(&ow_vapor_temp);
int tempSensorResolution = 12; //bits
int tempRequestDelay = 0;
// temperature variables...
float ambient_temp_value = 24;
float discharge_temp_value = 24;
float liquid_temp_value = 24;
float vapor_temp_value = 24;
//sensor variables.
bool compressor_state = false;
float cc_rms = 0;
float current_readings[5] = {0,0,0,0,0}; // last 5 values for avg calculations.xs.
int sensor_readings_count = 0;
float ac_mains_voltage = 0; // 90 - 260VAC
float high_pressure_value = 0.5; //0V = open sensor, fault.. >= 0,5V = analog sensor connected... > 4,5V = pressure switch connected
float low_pressure_value = 0.5; //0V = open sensor, fault.. >= 0,5V = analog sensor connected... > 4,5V = pressure switch connected

// Handler de tareas de lectura de sensores para nucleo 0 o 1
TaskHandle_t Task1;

// ADS
Adafruit_ADS1115 pressure_ads;
Adafruit_ADS1115 power_ads;

// esp-now variables
esp_now_peer_info_t server_peer; // variable holds the data for esp-now communications.
const int MAX_PAIR_ATTEMPTS = 55; // 5 times on each channel.
const char SERIAL_DEFAULT[] = "ffffffffffff";
uint8_t server_mac_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
int radio_channel = 1; // esp-now communication channel.
int pair_request_attempts = 0;
bool postEspnowFlag = false;

enum PeerRoleID {SERVER, CONTROLLER, MONITOR, UNSET};
enum PairingStatusEnum {PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED, NOT_PAIRED};
enum MessageTypeEnum {PAIRING, DATA,};
enum SysModeEnum {AUTO_MODE, FAN_MODE, COOL_MODE};
enum SysStateEnum {SYSTEM_ON, SYSTEM_OFF, SYSTEM_SLEEP, UNKN};
enum LedAnimationStyle {PULSE, ALLWAYS_ON, ALLWAYS_OFF, BLINK_2HZ, BLINK_1HZ};
enum AlarmCode {NORMAL, LOW_P, HIGH_P, AMP_LIMIT, FROM_APP};
//-vars
PairingStatusEnum pairingStatus = NOT_PAIRED;
AlarmCode alarm_code_state = NORMAL;
AlarmCode prev_alarm_code = NORMAL;

typedef struct monitor_data_struct {
  MessageTypeEnum msg_type;     // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  uint8_t fault_code;           // (1 byte) 0=no_fault; 1..255 monitor_fault_codes.
  float ambient_temp;           // (4 bytes) ambient temperature readings [°C]
  float discharge_temp;         // (4 bytes) discharge temperature readings [°C]
  float liquid_temp;            // (4 bytes) liquid line temperature readings [°C]
  float vapor_temp;             // (4 bytes) vapor line temperature readings [°C]
  float low_pressure;           // (4 bytes) low pressure readings [volts, 0-5V]
  float high_pressure;          // (4 bytes) liquid line pressure [volts, 0-5V]
  float ac_mains_voltage;       // (4 bytes) ac mains voltage readinsg [volts, 90 - 260V]
  float compressor_current;     // (4 bytes) compressor_current readings [volts, 0-1V]
  bool compressor_state;        // (1 byte) compressor on|off state.
  AlarmCode alarm_code;         // (1 byte) alarm_code_state... enum value
  unsigned int seconds_since_last_cooling_rq;  // (4 bytes) seconds since last false->true compressor state change.
  unsigned int total_cooling_hours;            // (4 bytes) total cooling request hours. state in monitor device.

} monitor_data_struct;          // TOTAL = 46 bytes

typedef struct incoming_settings_struct {
  MessageTypeEnum msg_type;     // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  SysModeEnum system_mode;      // (1 byte)
  SysStateEnum system_state;    // (1 byte)
  float system_temp_sp;         // (4 bytes) [°C]
  float room_temp;              // (4 bytes) [°C]
  bool monitor_remote_alarm;    // (1 byte) > control over the alarm relay of the monitor.
  bool monitor_alarm_rstrt;     // (1 byte) > restart all alarms from the broker.
} incoming_settings_struct;     // TOTAL = 13 bytes

typedef struct pairing_data_struct {
  MessageTypeEnum msg_type;     // (1 byte)
  PeerRoleID sender_role;       // (1 byte)
  PeerRoleID device_new_role;   // (1 byte)
  uint8_t channel;              // (1 byte) - 0 is default, let this value for future changes.
} pairing_data_struct;          // TOTAL = 9 bytes

//Create struct_message 
monitor_data_struct outgoing_data;  // data to send
incoming_settings_struct settings_data = { // initial values.
  DATA, UNSET, FAN_MODE, UNKN, 24, 24, false, false
  };  // data received from server
pairing_data_struct pairing_data;

// time vars.
unsigned int secSinceCoolingReq = 0; // seconds since last cooling request.
unsigned int hourmeter_count = 0; // hourmeter count.
unsigned int coolingSecondsTick = 0; // total cooling request counter.
unsigned long lastTempRequest = 0;
unsigned long lastEspnowReceived = 0; // Stores last time data was published
unsigned long lastPairingRequest = 0;
unsigned long currentMillis = 0;
unsigned long lastNetworkLedBlink = 0;
unsigned long lastAlarmLedBlink = 0;
unsigned long lastNowBtnChange = 0;
unsigned long lastAlarmBtnChange = 0;
unsigned long lastAlarmRelayTurnOn = 0;
unsigned long lastSystemLog = 0;
unsigned long lastSecondTick = 0;
unsigned long lastADCReading = 0;
const unsigned long SYSTEM_LOG_DELAY = 1000L; // 1 second.
const unsigned long COMPRESSOR_SHORT_CYCLING_DELAY = 3L * 60000L; // 3 minutes for compressor short cycling prevention.
const unsigned long ESP_NOW_POST_INTERVAL = 750L; // 0.75 seconds after receiving data from the server.
const unsigned long ESP_NOW_WAIT_SERVER_MSG = 1L * 60000L; // 1 minute for server message to arrive before PAIRING mode is set.
const unsigned long ESP_NOW_WAIT_PAIR_RESPONSE = 2000; // Interval to wait for pairing response from server
const unsigned long BTN_DEBOUNCE_TIME = 100; // 100ms rebound time constant;

// gen vars
bool now_btn_state = false;
bool last_now_btn_state = false;
bool alarm_btn_state = false;
bool last_alarm_btn_state = false;
bool system_alarm_state = false;
int led_brightness = 0;
int led_fade_amount = 5;
bool network_led_state = false;
bool alarm_led_state = false;
bool adc_readed = false;
uint16_t alarm_led_count = 0;

//logger function
void debug_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "%s", message);
}

void info_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "%s", message);
}

void error_logger(const char *message) {
  ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "%s", message);
}

// network led animations...
void network_led_animation(LedAnimationStyle animation_style) {
  // pulse effect.
  currentMillis = millis();

  switch (animation_style)
  {
  case PULSE:
    if (currentMillis - lastNetworkLedBlink > 50L) { //tick, 50ms
      lastNetworkLedBlink = currentMillis;
      analogWrite(NETWORK_LED, led_brightness);
      led_brightness = led_brightness + led_fade_amount;
      //-validations
      if (led_brightness > 125) {led_brightness = 125;}
      if (led_brightness < 0) {led_brightness = 0;}
      //-
      if (led_brightness <= 0 || led_brightness >= 125) {
        led_fade_amount = -led_fade_amount;
      }
    }
    break;

  case ALLWAYS_OFF:
    analogWrite(NETWORK_LED, 0);
    break;

  case ALLWAYS_ON:
    analogWrite(NETWORK_LED, 255);
    break;

  case BLINK_2HZ:
    if (currentMillis - lastNetworkLedBlink >= 250) {
      lastNetworkLedBlink = currentMillis;
      if (network_led_state == false) {
        network_led_state = true;
      } else {
        network_led_state = false;
      }
    }
    digitalWrite(NETWORK_LED, network_led_state);
    break;

  case BLINK_1HZ:
    if (currentMillis - lastNetworkLedBlink >= 500) {
      lastNetworkLedBlink = currentMillis;
      if (network_led_state == false) {
        network_led_state = true;
      } else {
        network_led_state = false;
      }
    }
    digitalWrite(NETWORK_LED, network_led_state);
    break;
  
  default:
    digitalWrite(NETWORK_LED, 0);
    break;
  }
}

// network led animations...
void alarm_led_animation(AlarmCode alarm_code) {
// enum AlarmCode {NORMAL, LOW_P, HIGH_P, AMP_LIMIT};
  currentMillis = millis();

  if (currentMillis - lastAlarmLedBlink >= 250) {
    lastAlarmLedBlink = currentMillis;

    if (alarm_led_state == false) {
      alarm_led_state = true;
      alarm_led_count ++;
    } else {
      alarm_led_state = false;
    }

    if (alarm_led_count > 5) {
      alarm_led_count = 0;
    }

    switch (alarm_code)
    {

    case NORMAL:
      digitalWrite(ALARM_LED, false);
      alarm_led_count = 0;

      break;

    case FROM_APP:
    // Blink 2 times

      if (alarm_led_count > 1) {
        alarm_led_state = false;
      }

      digitalWrite(ALARM_LED, alarm_led_state);
      break;

    case LOW_P:
    // Blink 3 times

      if (alarm_led_count > 2) {
        alarm_led_state = false;
      }

      digitalWrite(ALARM_LED, alarm_led_state);
      break;

    case HIGH_P:
    // Blink 4 times

      if (alarm_led_count > 3) {
        alarm_led_state = false;
      }

      digitalWrite(ALARM_LED, alarm_led_state);
      break;

    case AMP_LIMIT:
    // Blink 5 times

      if (alarm_led_count > 4) {
        alarm_led_state = false;
      }

      digitalWrite(ALARM_LED, alarm_led_state);
      break;
    
    default:
      digitalWrite(ALARM_LED, false);
      break;
    }
  }
}

// función que carga una String que contiene toda la información dentro del target_file del SPIFFS.
String load_data_from_fs(const char *target_file) {
  //-
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "loading data from %s", target_file);
  //-
  File f = SPIFFS.open(target_file);
  if (!f) {
    error_logger("Error al abrir el archivo solicitado.");
    return "null";
  }

  String file_string = f.readString();
  f.close();

  //log
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "data loaded from SPIFFS: %s", file_string.c_str());
  return file_string;
}

// función que guarda una String en el target_file del SPIFFS.
void save_data_in_fs(String data_to_save, const char *target_file) {
  //savin data in filesystem.
  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "saving in:%s this data:%s", target_file, data_to_save.c_str());

  File f = SPIFFS.open(target_file, "w");
  if (!f){
    error_logger("Error al abrir el archivo solicitado.");
    return;
  }

  f.print(data_to_save);
  f.close();
  debug_logger("data saved in SPIFFS!");

  return;
}

// print mac address.
String print_device_mac(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

//get device id from mac address.
String print_device_serial(const uint8_t * mac_addr) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // return mac_str;
  String str = (char*) mac_str;
  return str;
}

//- mac address parser
void parse_mac_address(const char* str, char sep, byte* bytes, int maxBytes, int base) {
    for (int i = 0; i < maxBytes; i++) {
        bytes[i] = strtoul(str, NULL, base);  // Convert byte
        str = strchr(str, sep);               // Find next separator
        if (str == NULL || *str == '\0') {
            break;                            // No more separators, exit
        }
        str++;                                // Point to next character after separator
    }
}

// set default values for server peer in fs.
void set_defaults_in_fs(){
  info_logger("saving server default values in the fs.");
  JsonDocument json_doc;
  String data;

  //create json_doc
  json_doc["server_serial"] = "ffffffffffff";
  json_doc["server_mac"] = "FF:FF:FF:FF:FF:FF";
  json_doc["server_chan"] = 1;

  serializeJson(json_doc, data);
  save_data_in_fs(data, "/now_server.txt");

  return;

}

// save new server data in fs.
void save_server_in_fs(const uint8_t *new_mac_address, uint8_t new_channel) {
  //- save server info in spiffs.

  info_logger("Saving new server data in the fs.");
  JsonDocument server_json;
  String data;
  
  // create json
  server_json["server_serial"] = print_device_serial(new_mac_address);
  server_json["server_mac"] = print_device_mac(new_mac_address);
  server_json["server_chan"] = new_channel;
  //save json in filesystem.
  serializeJson(server_json, data);
  save_data_in_fs(data, "/now_server.txt");

  return;
}

//- load server mac address from filesystem.
void load_server_from_fs() {
  info_logger("Loading server data from fs.");

  JsonDocument server_json;
  String server_data = load_data_from_fs("/now_server.txt");
  DeserializationError error = deserializeJson(server_json, server_data);
  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "server_json Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const char *server_ser = server_json["server_serial"] | "null";
  const char *server_mac_str = server_json["server_mac"] | "null";
  uint8_t server_channel = server_json["server_chan"] | 1;

  //- validations.
  if (radio_channel < 1 || radio_channel > MAX_CHANNEL) {
    error_logger("Invalid Wifi channel stored in fs. setting channel to default value.");
    server_channel = 1;
  }

  if (strcmp(server_ser, "null") == 0) {
    error_logger("server id not found in fs.");
    return;
  }
  if (strcmp(server_mac_str, "null") == 0) {
    error_logger("server mac addres not found in fs.");
    return;
  }

  //reset server_peer variable.
  memset(&server_peer, 0, sizeof(esp_now_peer_info_t));

  //-
  parse_mac_address(server_mac_str, ':', server_mac_address, 6, 16);

  //- update server_peer variable.
  memcpy(&server_peer.peer_addr, server_mac_address, sizeof(uint8_t[6]));
  server_peer.encrypt = false;
  server_peer.channel = server_channel;
  radio_channel = server_channel;

  //- Set pairingStatus based on the server_serial loaded from fs.
  if (strcmp(server_ser, SERIAL_DEFAULT) == 0) {
    //- server address has never been set. configure idle mode for pairing process.
    info_logger("[esp-now] server mac address is the default value.");
    pairingStatus = NOT_PAIRED;
  } else {
    info_logger("[esp-now] ready for esp-now communication with the server");
    pairingStatus = PAIR_REQUEST;
  }

  info_logger("server data loaded from fs!");
  return;
}

//-functions
bool is_valid_temp(float measurement) {
  char message_buffer[40];
  char name[8] = "ds18B20";

  if (measurement == -127) {
    sprintf(message_buffer, "Error -127; [%s]", name);
    error_logger(message_buffer);
    return false;
  }
  return true;
}

// update temperature readings...
void update_temperature_readings()
{
  if (millis() - lastTempRequest >= tempRequestDelay) {
    //-
    float ambient_buffer = ambient_temp_sensor.getTempCByIndex(0);
    float discharge_buffer = discharge_temp_sensor.getTempCByIndex(0);
    float liquid_buffer = liquid_temp_sensor.getTempCByIndex(0);
    float vapor_buffer = vapor_temp_sensor.getTempCByIndex(0);

    if (is_valid_temp(ambient_buffer)){
      // update global variable.
      ambient_temp_value = ambient_buffer;
    } else {
      error_logger("invalid reading on ambient temperature sensor.");
    }

    if (is_valid_temp(discharge_buffer))
    {
      // update global variable.
      discharge_temp_value = discharge_buffer;
    } else {
      error_logger("invalid reading on discharge_temp sensor.");
    }

    if (is_valid_temp(liquid_buffer))
    {
      // update global variable.
      liquid_temp_value = liquid_buffer;
    } else {
      error_logger("invalid reading on liquid_temp sensor.");
    }

    if (is_valid_temp(vapor_buffer))
    {
      // update global variable.
      vapor_temp_value = vapor_buffer;
    } else {
      error_logger("invalid reading on vapor_temp sensor.");
    }
    
    //request new readings.
    ambient_temp_sensor.requestTemperatures();
    discharge_temp_sensor.requestTemperatures();
    liquid_temp_sensor.requestTemperatures();
    vapor_temp_sensor.requestTemperatures();
    //- set timer
    lastTempRequest = millis();
  }
  return;
}

// get adc inputs readings. Task1 (Core 0)
void update_adc_readings(void *pvParameters) {

  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  
  for (;;)
  {
    /*get readings*/
    int16_t lp_results;
    int16_t hp_results;
    float cc_result = 0;
    float cc_sum_square = 0;
    float cc_rms_sum = 0;
    int cc_readings_count = 0;
    float PWR_ADC_MULTI = 0.0625F; // GAIN_TWO
    const float TC_FACTOR = 100; /* 100/1 */
    // current readings:
    lastADCReading = millis();
    while(millis() - lastADCReading < 500) { // 30 cycles aprox.
      cc_result = power_ads.readADC_Differential_0_1() * PWR_ADC_MULTI; //mVolts
      cc_result /= 1000.0; // Volts
      cc_sum_square += sq(cc_result);
      cc_readings_count += 1;
    }

    cc_rms = sqrt(cc_sum_square/cc_readings_count) * TC_FACTOR; //
    compressor_state = cc_rms > 2 ? true : false; // 2 amp threshold. Compressor state (true|false)

    if (sensor_readings_count < 5) {
      current_readings[sensor_readings_count] = cc_rms;
      sensor_readings_count ++;
      
    } else {
      sensor_readings_count = 1;
      for (int i = 0; i < 5; i++) {
        cc_rms_sum += current_readings[i];
      }
      current_readings[0] = cc_rms_sum / 5; // AVG current value.
      adc_readed = true;
    }

    /* pressure readings */
    lp_results = pressure_ads.readADC_SingleEnded(0);
    hp_results = pressure_ads.readADC_SingleEnded(1);
    //compute volts and update global variables.
    low_pressure_value = pressure_ads.computeVolts(lp_results);
    high_pressure_value = pressure_ads.computeVolts(hp_results);

    // task delay
    vTaskDelay(xDelay);
  }


  return;
}

// update alarm state.
void update_alarm_state() {

  if (!adc_readed) {
    return;
  }

  if (settings_data.monitor_remote_alarm == true) {
    alarm_code_state = FROM_APP;
    // alarm generated on the app..

  } else if (current_readings[0] >= 60) { //TODO -> install a potentiometer to regulate the amp. limits.
    alarm_code_state = AMP_LIMIT;
    // compressor current alarm

  } else if (low_pressure_value <= 0.3) {
    alarm_code_state = LOW_P;
    // low pressure alarm

  } else if (high_pressure_value <= 0.3) {
    alarm_code_state = HIGH_P;
    // high pressure alarm.
  }

  if (prev_alarm_code != alarm_code_state) {
    // save state in fs.
    prev_alarm_code = alarm_code_state;

    info_logger("Saving new server data in the fs.");
    JsonDocument json;
    String data;
    
    // create json
    json["alarm"] = alarm_code_state;
    //save json in filesystem.
    serializeJson(json, data);
    save_data_in_fs(data, "/alarm_state.txt");
  }
  
  return;
}

// update inputs and outputs.
void update_IO()
{
  currentMillis = millis();
  //inputs
  const bool current_now_btn = digitalRead(NOW_CNF) ? false : true; //button is active low.
  const bool current_alarm_btn = digitalRead(ALARM_RESET) ? false : true; // alarm button is active low.

  // now button debounce.
  if (current_now_btn != last_now_btn_state) {
    lastNowBtnChange = currentMillis;
    last_now_btn_state = current_now_btn;
  }

  if (currentMillis - lastNowBtnChange >= BTN_DEBOUNCE_TIME) {
    // btn change.
    now_btn_state = current_now_btn;
  }

  // alarm button debounce.
  if (current_alarm_btn != last_alarm_btn_state) {
    lastAlarmBtnChange = currentMillis;
    last_alarm_btn_state = current_alarm_btn;
  }

  if (currentMillis - lastAlarmBtnChange >= BTN_DEBOUNCE_TIME){
    // float changed value.
    alarm_btn_state = current_alarm_btn;
  }

  //
  if (alarm_btn_state) { // if alarm-reset-button is pressed.
    //try to restart the fault code..
    alarm_code_state = NORMAL;
    alarm_btn_state = false;
  }

  //- outputs.
  // update alarm led animation.
  update_alarm_state(); // update alarm state... even if the rst button is pressed, the fault will appear again.
  // set alarm led animation
  alarm_led_animation(alarm_code_state);
  // set alarm relay state.

  if (alarm_code_state == NORMAL) {
    digitalWrite(ALARM_RELAY, LOW); //normally closed contacts
    //TODO: Implement a timer that prevent short-cycling the compressor.
  } else {
    digitalWrite(ALARM_RELAY, HIGH);
  }

  return;
}

// load alarm state from fs.
void load_alarm_state_from_fs() {
  info_logger("-> loading alarm state from fs");
  JsonDocument json;
  String alarm_data = load_data_from_fs("/alarm_state.txt");

  DeserializationError error = deserializeJson(json, alarm_data);
  if (error) {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "hourmeter Deserialization error raised with code: %s", error.c_str());
    return;
  }

  alarm_code_state = json["alarm"];
  prev_alarm_code = alarm_code_state;

  return;
}

// time counter functions
void load_hourmeter_from_fs() {
  info_logger("-> loading hourmeter from fs");
  JsonDocument json;
  String hourmeter_data = load_data_from_fs("/hourmeter.txt");

  DeserializationError error = deserializeJson(json, hourmeter_data);
  if (error) {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "hourmeter Deserialization error raised with code: %s", error.c_str());
    return;
  }

  hourmeter_count = json["hours"] | 0;

  return;
}

void update_hourmeter_in_fs() {

  // call this function every minute.

  info_logger("updating hourmeter in fs.");
  const char * target_file = "/hourmeter.txt";
  JsonDocument json;
  JsonDocument new_json;
  String new_hourmeter;
  String hourmeter_data = load_data_from_fs(target_file);

  DeserializationError error = deserializeJson(json, hourmeter_data);
  if (error)
  {
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "hourmeter Deserialization error raised with code: %s", error.c_str());
    return;
  }

  const int current_h = json["hours"] | 0;
  const int current_m = json["minutes"] | 0;

  if (current_m < 59) {
    new_json["minutes"] = current_m + 1;
    new_json["hours"] = current_h;
  } else {
    new_json["minutes"] = 0;
    new_json["hours"] = current_h + 1;
  }

  //update global value.
  hourmeter_count = new_json["hours"];

  serializeJson(new_json, new_hourmeter);
  save_data_in_fs(new_hourmeter, target_file);
}

void update_cr_counter() {

  currentMillis = millis();

  if (!compressor_state) {
    //nothing to count when the compressor is off.
    secSinceCoolingReq = 0; // set to 0 the seconds counter.
    return;
  }

  if (currentMillis - lastSecondTick >= 1000) {
    //1 second count
    lastSecondTick = currentMillis;
    secSinceCoolingReq ++; // sum 1 second to the counter.
    coolingSecondsTick ++;

    if (coolingSecondsTick >= 60) { // every minute.
      update_hourmeter_in_fs();
      coolingSecondsTick = 0;
    }
  }

  return;
}

//- *esp_now functions
bool add_peer_to_plist(const uint8_t * mac_addr, uint8_t channel){
  info_logger("[esp-now] adding new peer to peer list.");

  if (channel <= 0 || channel > MAX_CHANNEL) {
    error_logger("invalid channel value received. peer not added");
    return false;
  }

  ESP_LOG_LEVEL(ESP_LOG_DEBUG, TAG, "MAC: %s, channel: %d.", print_device_mac(mac_addr).c_str(), channel);
  //- set to 0 all data of the peerTemplate var.
  memset(&server_peer, 0, sizeof(esp_now_peer_info_t));

  //-set data
  //update global variable.
  memcpy(&server_peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  server_peer.channel = channel;
  server_peer.encrypt = false;

  // delete existing peer.
  if (esp_now_is_peer_exist(server_peer.peer_addr)) {
    debug_logger("peer already exists. deleting old data.");
    esp_now_del_peer(server_peer.peer_addr);
  }
  // save peer in peerlist
  esp_err_t result = esp_now_add_peer(&server_peer);

  //- update WiFi channel.
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
  info_logger("WiFi channel updated!");

  switch (result)
  {
  case ESP_OK:
    info_logger("New peer added successfully!..");
    return true;
  
  default:
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "Error: %d, while adding new peer.", esp_err_to_name(result));
    return false;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //- callback.
  String device_id = print_device_mac(mac_addr);
  switch (status)
  {
  case ESP_NOW_SEND_SUCCESS:
    ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] packet to: %s has been sent!", device_id.c_str());
    break;
  
  default:
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[esp-now] packet to: %s not sent.", device_id.c_str());
    break;
  }
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  //-
  String mac_str = print_device_mac(mac_addr);
  String sender_serial = print_device_serial(mac_addr);
  String server_serial = print_device_serial(server_peer.peer_addr);

  ESP_LOG_LEVEL(ESP_LOG_INFO, TAG, "[esp-now] %d bytes of data received from: %s", len, mac_str.c_str());

  //only accept messages from server device.
  uint8_t message_type = incomingData[0];
  uint8_t sender_role = incomingData[1];

  if (sender_role != SERVER) {
    info_logger("[esp-now] invalid device role. ignoring message");
    return;
  }

  lastEspnowReceived = millis(); // time of the last message received from the server.

  //get message_type message from first byte.
  switch (message_type) {
  case DATA :      // we received data from server
    //- DATA type
    //- validations
    if (pairingStatus != PAIR_PAIRED) {
      info_logger("device not paired yet! ignoring data.");
      break;
    }
    if (sender_serial != server_serial) {
      info_logger("message received from an unknown device, ignoring data.");
      break;
    }
    //- ingest incoming data.
    info_logger("[esp-now] message of type DATA received");
    memcpy(&settings_data, incomingData, sizeof(settings_data));
    postEspnowFlag = true; // flag to send a response to the server.
    //-
    // simulates a press to the alarm_btn...
    if (settings_data.monitor_alarm_rstrt) {
      alarm_btn_state = true;
    }
    //-
    break;

  case PAIRING:    // received pairing data from server
    //- PAIRING type
    info_logger("[esp-now] message of type PAIRING received");
    memcpy(&pairing_data, incomingData, sizeof(pairing_data));
    // add peer to peer list.
    if (!add_peer_to_plist(mac_addr, pairing_data.channel)){ // the server decides the channel.
      error_logger("[esp-now] server peer couldn't be saved, try again.");
      break;
    }
    // save server data in fs.
    save_server_in_fs(mac_addr, pairing_data.channel); // update server info in fs.
    
    // device PAIRED with server.
    char buff[100] = "";
    sprintf(buff, "device paired on channel %d after %d attempts", radio_channel, pair_request_attempts);
    info_logger(buff);
    pair_request_attempts = 0;
    pairingStatus = PAIR_PAIRED;  // set the pairing status
    //-
    break;
  }
}

void log_on_result(esp_err_t result) {
    //-swtch
  switch (result)
  {
  case ESP_OK:
    info_logger("[esp-now] message to the server has been sent.");
    break;
  
  default:
    ESP_LOG_LEVEL(ESP_LOG_ERROR, TAG, "[esp-now] error sending msg to peer, reason: %s",  esp_err_to_name(result));
    break;
  }

  return;
}

void system_logs() {
  currentMillis = millis();
  JsonDocument root;
  String doc;

  if (currentMillis - lastSystemLog >= SYSTEM_LOG_DELAY) {
    lastSystemLog = currentMillis;
    //- data
    root["amb_t"] = ambient_temp_value;
    root["dis_t"] = discharge_temp_value;
    root["liq_t"] = liquid_temp_value;
    root["vap_t"] = vapor_temp_value;
    root["low_p"] = low_pressure_value;
    root["hig_p"] = high_pressure_value;
    root["ac_volt"] = ac_mains_voltage;
    root["comp_amp"] = cc_rms;
    root["comp_stt"] = compressor_state;
    root["pair"] = pairingStatus;
    root["cr_cnt"] = secSinceCoolingReq;
    root["hourmt"] = hourmeter_count;

    //- output
    serializeJson(root, doc);
    debug_logger(doc.c_str());
  }

  return;
}

void espnow_loop(){
  // current time.
  currentMillis = millis();
  esp_err_t send_result;

  if (now_btn_state && currentMillis - lastNowBtnChange > 10000L) {
    // after 10 seconds of now button pressed, default values will be set.
    lastNowBtnChange = currentMillis;
    //-
    set_defaults_in_fs();
    //-
    info_logger("ESP restart...");
    delay(1000);
    ESP.restart();
  }

  // switch modes.
  switch(pairingStatus) {
    // PAIR REQUEST
    case PAIR_REQUEST:
      if (strcmp(SERIAL_DEFAULT, print_device_serial(server_mac_address).c_str()) == 0) // exits pair request process only if the server is default.
      {
        // check if no more attemps are allowed.
        if (pair_request_attempts >= MAX_PAIR_ATTEMPTS) {
          // ends pairing process.
          pairingStatus = NOT_PAIRED;
          pair_request_attempts = 0;
          info_logger("auto-pairing process couln't find the server.");
          break;
        }
      }
      pair_request_attempts ++;
      ESP_LOG_LEVEL(LOG_LOCAL_LEVEL, TAG, "Sending PR# %d on channel: %d", pair_request_attempts, radio_channel);
    
      // set pairing data to send to the server
      pairing_data.msg_type = PAIRING;
      pairing_data.sender_role = MONITOR;
      pairing_data.channel = radio_channel;

      // add peer and send request
      if (!add_peer_to_plist(server_mac_address, radio_channel)){ // mac address stored in global var.
        error_logger("[esp-now] couldn't add peer to peer list.");
        break;
      }

      //- send pair data.
      send_result = esp_now_send(server_peer.peer_addr, (uint8_t *) &pairing_data, sizeof(pairing_data));
      //-log
      log_on_result(send_result);

      lastPairingRequest = currentMillis;
      pairingStatus = PAIR_REQUESTED;
    break;

    // PAIR REQUESTED
    case PAIR_REQUESTED:
      // change wifi channel for continue with the pairing process.
      network_led_animation(BLINK_2HZ);

      // time out to allow receiving response from server
      if(currentMillis - lastPairingRequest > ESP_NOW_WAIT_PAIR_RESPONSE) {
        info_logger("[autopairing] time out expired, try on the next channel");
        radio_channel ++;
        if (radio_channel > MAX_CHANNEL){
          radio_channel = 1;
        }
        // set WiFi channel   
        pairingStatus = PAIR_REQUEST;

      }
    break;

    // NOT PAIRED
    case NOT_PAIRED:
    network_led_animation(BLINK_1HZ);
      if (now_btn_state && currentMillis - lastNowBtnChange > 3000L) {
        // after 3 seconds of now button pressed..
        pairingStatus = PAIR_REQUEST; // begin pair process.
      }
      // waiting for now button press to begin pair process. 
      // starts with the default address (ffx6) and wait for the response
      // to save the correct mac_address.
    break;
    
    // PAIRED
    case PAIR_PAIRED:
      //- posting data on posting intervals.
      network_led_animation(PULSE);

      if (currentMillis - lastEspnowReceived >= ESP_NOW_WAIT_SERVER_MSG) {
        info_logger("Timeout since last server message received. Setting up PR Mode.");
        pairingStatus = PAIR_REQUEST;
      }

      //- time based message sent.
      if (postEspnowFlag && currentMillis - lastEspnowReceived >= ESP_NOW_POST_INTERVAL) {
        // set flag to false.
        postEspnowFlag = false;
        //Set values to send
        outgoing_data.msg_type = DATA;
        outgoing_data.sender_role = MONITOR;
        outgoing_data.fault_code = 0x00;
        outgoing_data.ambient_temp = ambient_temp_value;
        outgoing_data.discharge_temp = discharge_temp_value;
        outgoing_data.vapor_temp = vapor_temp_value;
        outgoing_data.liquid_temp = liquid_temp_value;
        outgoing_data.low_pressure = low_pressure_value;
        outgoing_data.high_pressure = high_pressure_value;
        outgoing_data.ac_mains_voltage = ac_mains_voltage;
        outgoing_data.compressor_current = current_readings[0]; // send last avg value calculated.
        outgoing_data.compressor_state = compressor_state;
        outgoing_data.alarm_code = alarm_code_state;
        outgoing_data.seconds_since_last_cooling_rq = secSinceCoolingReq;
        outgoing_data.total_cooling_hours = hourmeter_count;

        send_result = esp_now_send(server_peer.peer_addr, (uint8_t *) &outgoing_data, sizeof(outgoing_data));
        log_on_result(send_result);
      }
      //-
    break;
  }

  //-end
  return;
}

// -setup
void setup() {
  // SETUP BEGIN
  // ***************** logger SETUP ******************
  esp_log_level_set("*", ESP_LOG_DEBUG); 
  Serial.begin(115200); 
  debug_logger("** setup init. **");
  // ***************** logger SETUP ******************

  // ***************** pinout SETUP ******************
  info_logger("pinout setup");
  pinMode(NETWORK_LED, OUTPUT);
  pinMode(ALARM_RELAY, OUTPUT);
  pinMode(ALARM_LED, OUTPUT);
  pinMode(BRKR_LED, OUTPUT);
  pinMode(ALARM_RESET, INPUT);
  pinMode(NOW_CNF, INPUT);
  pinMode(CONTROL_VOLTAGE_PRESENCE, INPUT);
  // ***************** pinout SETUP ******************

  // ***************** SPIFFS SETUP ******************
  info_logger("SPIFFS setup");
  if(!SPIFFS.begin(true)) {
    error_logger("Ocurrió un error al iniciar SPIFFS..");
    while (1){;}
  }
  // ***************** SPIFFS SETUP ******************

  // ***************** ADS SETUP ******************
  info_logger("ADS Setup");
  if (!pressure_ads.begin(0x49)) {
    Serial.println("Failed to initialize pressure_ads.");
    while (1);
  }
  // default sps
  pressure_ads.setGain(GAIN_TWOTHIRDS); /* 2/3x gain, ADC Range +/- 6.144V  (1 bit = 0.1875mV) -> POWER @ 5V*/
  //start first conversion.
  info_logger("Pressure ADS setup in address 0x49 completed.");

  if (!power_ads.begin(0x48)) {
    Serial.println("Failed to initialize power_ads.");
    while (1);
  }

  power_ads.setDataRate(RATE_ADS1115_860SPS); // 860 samples per second
  power_ads.setGain(GAIN_TWO); /* 2x gain, ADC Range: +/- 2.048V (1 bit = 0.0625mV) -> POWER @ 3.3V*/

  info_logger("Electric ADS setup in address 0x48 completed");
  // ***************** ADS SETUP ******************

  // ***************** OneWire SETUP **************
  info_logger("OneWire sensors setup");
  //--ambient temp. sensor
  ambient_temp_sensor.begin();
  ambient_temp_sensor.setResolution(tempSensorResolution);
  ambient_temp_sensor.setWaitForConversion(false);
  //--discharge temp. sensor
  discharge_temp_sensor.begin();
  discharge_temp_sensor.setResolution(tempSensorResolution);
  discharge_temp_sensor.setWaitForConversion(false);
  //--liquid temp. sensor
  liquid_temp_sensor.begin();
  liquid_temp_sensor.setResolution(tempSensorResolution);
  liquid_temp_sensor.setWaitForConversion(false);
  //--vapor temp. sensor
  vapor_temp_sensor.begin();
  vapor_temp_sensor.setResolution(tempSensorResolution);
  vapor_temp_sensor.setWaitForConversion(false);

  //first temp. request.
  info_logger("sending first temp request to oneWire sensors");
  ambient_temp_sensor.requestTemperatures();
  discharge_temp_sensor.requestTemperatures();
  liquid_temp_sensor.requestTemperatures();
  vapor_temp_sensor.requestTemperatures();
  //set timmers.
  lastTempRequest = millis();
  tempRequestDelay = 750 / (1 << (12 - tempSensorResolution));
  // ***************** OneWire SETUP **************

  // ***************** WiFi SETUP *****************
  info_logger("WiFi setup");
  WiFi.mode(WIFI_MODE_STA);
  info_logger("MAC Address:  ->");
  info_logger(WiFi.macAddress().c_str());
  WiFi.disconnect();
  // ***************** WiFi SETUP *****************

  // ***************** ESP-NOW SETUP **************
  info_logger("esp-now settings");
  uint8_t mac_buffer[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, mac_buffer);
  if (ret != ESP_OK) {
    error_logger("could not read mac address.");
  }
  //-
  if (esp_now_init() != ESP_OK) {
    error_logger("-- Error initializing ESP-NOW, please reboot --");
    while (1){;}
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  //-done
  // ***************** ESP-NOW SETUP **************

  // **************** SECOND CORE SETUP ***********
  info_logger("Creating second core task");
  xTaskCreatePinnedToCore(
    update_adc_readings , /* Function to implement the task */
    "Task1",     /* Name of the task */
    10000,       /* Stack size in words */
    NULL,        /* Task input parameter */
    0,           /* Priority of the task */
    &Task1,      /* Task handle. */
    0);          /* Core where the task should run */
  // **************** SECOND CORE SETUP ***********

  //LOAD DATA FROM FS
  info_logger("loading data from SPIFFS system");
  load_server_from_fs();
  load_hourmeter_from_fs();
  load_alarm_state_from_fs();

  //time var
  currentMillis = millis();
  info_logger("setup completed --!.");
  delay(500);
  // ***************** SETUP COMPLETED ************

}

void loop() {
  //-
  update_temperature_readings();
  update_IO();
  update_cr_counter();
  espnow_loop();
  // system_logs(); // debug only..
}