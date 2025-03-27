/************* Includes *************/
#include <MaterBox.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
//#include <time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>  // 
/************* End Includes *************/

/************* Define default values *************/
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr char CURRENT_FIRMWARE_TITLE[] = "MB-MushroomIncubator";
constexpr char CURRENT_FIRMWARE_VERSION[] = "0.1.0";
const char* deviceName            = "MB-Mushroom-Incubator";
unsigned long mtime               = 0;
int TIME_TO_SEND_TELEMETRY  = 30; //every x seconds to send tellemetry
/************* End Define default values *************/

/************* Double Reset config *************/
#define ESP_DRD_USE_LITTLEFS      true
#define ESP_DRD_USE_SPIFFS        false
#define ESP_DRD_USE_EEPROM        false
#define ESP8266_DRD_USE_RTC       false
#define DOUBLERESETDETECTOR_DEBUG true  //false
#include <ESP_DoubleResetDetector.h>    //https://github.com/khoih-prog/ESP_DoubleResetDetector
#define DRD_TIMEOUT 5 // Number of seconds after reset during which a subseqent reset will be considered a double reset.
#define DRD_ADDRESS 0

DoubleResetDetector* drd;
/************* End Double Reset config *************/

/************* Thingsboard *************/
#define THINGSBOARD_ENABLE_PROGMEM 0  // Disable PROGMEM because the ESP8266WiFi library, does not support flash strings.
//#define THINGSBOARD_ENABLE_STREAM_UTILS 1 // Enables sending messages that are bigger than the predefined message size
//#define THINGSBOARD_ENABLE_DYNAMIC 1
//#define THINGSBOARD_ENABLE_PSRAM 0

#include <Arduino_MQTT_Client.h>
#include <OTA_Firmware_Update.h>
#include <Server_Side_RPC.h>
#include <Client_Side_RPC.h>
#include <ThingsBoard.h>          //https://github.com/thingsboard/thingsboard-arduino-sdk

char THINGSBOARD_SERVER[40] = "materbox.io";
char TOKEN[40] = "TEST_TOKEN";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 512U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 512U;

// RPC
// Statuses for subscribing to rpc
bool subscribed = false;

constexpr char RPC_REQUEST_GET_CURRENT_TIME[] = "getCurrentTime";
constexpr const char RPC_SET_HOURS_OF_LIGHT[] = "setHoursOfLight";
constexpr const char RPC_TIME_TO_SEND_TELEMETRY[] = "timeToSendTelemetry";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;
constexpr uint8_t MAX_RPC_REQUEST = 5U;
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

// OTA
// Maximum amount of retries we attempt to download each firmware chunck over MQTT
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;

// Size of each firmware chunck downloaded over MQTT,
// increased packet size, might increase download speed
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;
// Statuses for updating
bool currentFWSent = false;
bool updateRequestSent = false;

WiFiClient espClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);

// Initialize used apis
OTA_Firmware_Update<> ota;
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
Client_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_REQUEST> rpc_request;
const std::array<IAPI_Implementation*, 3U> apis = {
  &rpc,
  &rpc_request,
  &ota
};

// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);
/************* End Thingsboard *************/

/************* Wifi Manager *************/
const char* modes[] = { "NULL", "STA", "AP", "STA+AP" };

WiFiManager wm;

String WM_DATA_FILE = "config.txt";
bool TEST_CP         = false; // always start the configportal, even if ap found
int  TESP_CP_TIMEOUT = 180; // test cp timeout
bool TEST_NET        = true; // do a network test after connect, (gets ntp time)
bool ALLOWONDEMAND   = false; // enable on demand
bool WMISBLOCKING    = true; // use blocking or non blocking mode, non global params wont work in non blocking
bool STAND_ALONE     = false; // use device without thingsboard server
bool RESET_SETTINGS  = false; //reset WIFI settings - for testing
bool WM_CONNECTED    = false;
bool DRD_DETECTED    = false;
bool SAVE_PARAMS     = false;
/************* End Wifi Manager *************/

/************* Lights control *************/
String LIGHTS_CONTROL_DATA_FILE = "lights.txt";

//Time Alarms
bool SET_TIME     = true;
bool SET_ALARMS   = true;
bool ALARMS_ARE_SET = false;
int ALARM_ID_ON;
int ALARM_ID_OFF;

/************* Relay control *************/
bool state = false;
//using namespace ace_button;
#define Relay1 5   // GPIO5-D1   morado
#define Relay2 4   // GPIO4-D2   naranja
#define Relay3 14  // GPIO14-D5   amarillo
#define Relay4 12  // GPIO12-D6    azul

// Helper macro to calculate array size
//#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

/************* End Relay control *************/
/************* End Lights control *************/

/************* Sensor BH1750 *************/
#include <Wire.h>
#include <BH1750.h>
BH1750 luxMeter;
bool BH1750_DETECTED = false;
/************* End Sensor BH1750 *************/

/************* Sensor BME280 *************/
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
bool BME280_DETECTED = false;
/************* End Sensor BME280 *************/

/************* Sensor MQ-series *************/
String MQ_DATA_FILE = "mqdata.txt";
bool MQ_DETECTED = false;
bool MQ_DATA_DELETE  = false; // delete MQ_DATA_FILE - for testing
#define BOARD "ESP8266"
#define VOLTAGE_RESOLUTION 5
#define MQ_ANALOG_PIN A0 //Analog input 0 of your arduino
#define MQ_TYPE "MQ-135" //MQ135
#define ADC_BIT_RESOLUTION 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
//#define calibration_button 13 //Pin to calibrate your sensor
#include <MQUnifiedsensor.h>      //https://github.com/miguel5612/MQSensorsLib
//Declare Sensor
MQUnifiedsensor MQ135(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ_ANALOG_PIN, MQ_TYPE);
/************* End Sensor MQ-series *************/

MATERBOX mb;
/************* Prototype functions *************/
void setTimeAlarms(int lOnHour=30, int lOnMin=0, int lOnSec=0, int lOffHour=0, int lOffMin=0, int lOffSec=0);
/************* End Prototype functions *************/

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  while (!Serial) ; // wait for Arduino Serial Monitor
  mb.enqueueMessage("Starting", "INFO");
  mb.enqueueMessage("Test", "ERROR");
  mb.enqueueMessage("Test", "WARN");

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  if (drd->detectDoubleReset()) {
    mb.enqueueMessage("Double Reset Detected", "INFO");
      DRD_DETECTED = true;
    } else {
      mb.enqueueMessage("No Double Reset Detected", "INFO");
      DRD_DETECTED = false;
    }

  mb.begin();
  setupWifiManager(DRD_DETECTED);

  setupBh1750Sensor();
  setupBme280Sensor();
  setupMqSensor();
  setupRelay();

  mtime = TIME_TO_SEND_TELEMETRY * 1000;
}

void loop() {
  if(millis()-mtime > (TIME_TO_SEND_TELEMETRY * 1000)){
    if(WiFi.status() == WL_CONNECTED){
      if (!tb.connected()) {
        // Reconnect to the ThingsBoard server,
        // if a connection was disrupted or has not yet been established
        mb.enqueueMessage("Connecting to: " + String(THINGSBOARD_SERVER) + " with token " + String(TOKEN), "INFO");
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          mb.enqueueMessage("Failed to connect", "ERROR");
          subscribed = false;
        }
      } else {
        if (!subscribed) {
            rpcSubscribe();
        }
        if (SET_TIME){ //Get current time if not set
          setLocalTime();
        }
        if(SET_ALARMS){ //Set hours of light
          setTimeAlarms();
        }
        if (BH1750_DETECTED){
          sendTelemetryJson(getBh1750DataJson());
        }
        if (BME280_DETECTED){
          sendTelemetryJson(getBme280DataJson());
        }
        if (MQ_DETECTED){
          sendTelemetryJson(getMqDataJson());
        }
      }
    } else {
      mb.enqueueMessage("WiFi.status() == WL_CONNECTED " + String(WiFi.status()), "ERROR");
    }
    mtime = millis();
  }
  tb.loop();
}

void setupBh1750Sensor(){
 // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();
  // On esp8266 you can select SCL and SDA pins using Wire.begin(D4, D3);
  // For Wemos / Lolin D1 Mini Pro and the Ambient Light shield use Wire.begin(D2, D1);
  BH1750_DETECTED = luxMeter.begin();
  if(BH1750_DETECTED) {
    mb.enqueueMessage("BH1750 Test begin", "INFO");
  } else {
    mb.enqueueMessage("BH1750 not found or damage", "ERROR");
  }
}

void setupMqSensor(){
  if(MQ_DATA_DELETE){
    mb.deleteFileData(MQ_DATA_FILE);
  }
  float calcR0 = 0;
  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init();
  MQ135.setRL(2); //If the RL value is different from 10K, assign new RL value

  JsonDocument json;
  json = mb.loadData(MQ_DATA_FILE);
  
  if (!json["ERROR"]){
  //if (json.containsKey("R0_VALUE")){
    calcR0 = json["R0_VALUE"];
    mb.enqueueMessage("MQ load config calibration R0 = " + String(calcR0), "INFO");
  } else {
    calcR0 = mqSensorCalibration();
    mb.enqueueMessage("MQ sensor calibration using R0 = " + String(calcR0), "INFO");
  }

  if(isinf(calcR0)) {
    mb.enqueueMessage("R0 is infinite (Open circuit detected)", "ERROR");
    MQ_DETECTED = false;
    while(1);
  } else if(calcR0 == 0){
    mb.enqueueMessage("R0 is zero (Analog pin shorts to ground)", "ERROR");
    MQ_DETECTED = false;
    while(1);
  } else {
    MQ135.setR0(calcR0);
    mb.enqueueMessage("MQ135 sensor started", "INFO");
    MQ_DETECTED = true;
  }
}

float mqSensorCalibration(){
/*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  mb.enqueueMessage("MQ135 sensor is being Calibrating, please wait", "INFO");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  calcR0 = calcR0/10;
  
  JsonDocument json;
  json["R0_VALUE"] = calcR0;
  mb.saveData(json, MQ_DATA_FILE);
  /*****************************  MQ CAlibration ********************************************/ 
  return calcR0;
}

void setupBme280Sensor(){
  BME280_DETECTED = bme.begin(0x76);
  if (!BME280_DETECTED) {
    mb.enqueueMessage("Could not find a valid BME280 sensor!", "ERROR");
  } else {
    mb.enqueueMessage("BME280 sensor started", "INFO");
  }

}

void  setupRelay() {
  /************* Relay control *************/
  //  pinMode(Relay1, OUTPUT);
  //  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  
  //During Start all Relays should TURN OFF
  //  digitalWrite(Relay1, HIGH);
  //  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);
  /************* End Relay control *************/
}

void sendTelemetryJson(const JsonDocument &data){
  tb.sendTelemetryJson(data, Helper::Measure_Json(data));
  serializeJsonPretty(data, Serial);
  Serial.println();
}

JsonDocument getBh1750DataJson(){
  JsonDocument json;
  json["lux"] = luxMeter.readLightLevel();
  return json;
}

JsonDocument getMqDataJson(){
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  JsonDocument json;
    MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  json["co"]      = MQ135.readSensor();
    MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  json["co2"]     = MQ135.readSensor();
    MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
  json["alcohol"] = MQ135.readSensor();
    MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
  json["toluen"]  = MQ135.readSensor();
    MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
  json["nh4"]     = MQ135.readSensor();
    MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
  json["aceton"]  = MQ135.readSensor();

  return json;
  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen   | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton   | 34.668 | -3.369
  */
}

JsonDocument getBme280DataJson(){
  float temperature = 0;
  float humidity    = 0;
  float pressure    = 0;
  int i; //Random variable used to control the loops

  JsonDocument json;
  for(i = 0;i < 10;i++){
    Alarm.delay(5); //delay between each reading to avoid an error
    temperature += bme.readTemperature();
    humidity    += bme.readHumidity();
    pressure    += bme.readPressure();
  }

  json["temperature"] = temperature / 10; //Dividing to get means
  json["humidity"] = humidity / 10; //Dividing to get means
  json["pressure"] = pressure / 10; //Dividing to get means
  
  return json;
}

/************* RPC callbacks *************/
void rpcSubscribe(){
 mb.enqueueMessage("Subscribing for RPC", "INFO");

  const std::array<RPC_Callback, 2U> callbacks = {
    RPC_Callback{ RPC_SET_HOURS_OF_LIGHT,                   processSetTimeAlarms},
    RPC_Callback{ RPC_TIME_TO_SEND_TELEMETRY,               processTimeToSendTelemetry}
  };

  // Perform a subscription. All consequent data processing will happen in
  // processTemperatureChange() and processSwitchChange() functions,
  // as denoted by callbacks array.
  if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
    mb.enqueueMessage("Failed to subscribe for RPC", "ERROR");
    return;
  }
  mb.enqueueMessage("Subscribe done", "INFO");
  subscribed = true;
}

/// @brief Processes function for RPC call "SetTimeAlarms"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
void processSetTimeAlarms(const JsonVariantConst &data, JsonDocument &response) {
  mb.enqueueMessage("Received RPC call SetTimeAlarms", "RCP");

  // Process data
  //Lights on time
  int lOnHour = data["lOnHour"];
  int lOnMin  = data["lOnMin"];
  int lOnSec  = data["lOnSec"];
  
  //Lights off time
  int lOffHour = data["lOffHour"];
  int lOffMin  = data["lOffMin"];
  int lOffSec  = data["lOffSec"];
  
  printActualTime();

  setTimeAlarms(lOnHour, lOnMin, lOnSec, lOffHour, lOffMin, lOffSec);

  response.set(42);
}

/// @brief Processes function for RPC call "SetTimeAlarms"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
void processTimeToSendTelemetry(const JsonVariantConst &data, JsonDocument &response) {
  mb.enqueueMessage("Received timeToSendTelemetry method", "RCP");
  TIME_TO_SEND_TELEMETRY = data["TIME_TO_SEND_TELEMETRY"];

  JsonDocument json;
  json = mb.loadData(WM_DATA_FILE);

  if (!json["ERROR"]){
    json["TIME_TO_SEND_TELEMETRY"] = TIME_TO_SEND_TELEMETRY;
  }

  mb.saveData(json, WM_DATA_FILE);

  mb.enqueueMessage("Send telemetry every " + String(TIME_TO_SEND_TELEMETRY) + " seconds", "RCP");
  response.set(42);
}

/// @brief Processes function for RPC response of "getCurrentTime".
/// If no response is set the callback is called with {"error": "timeout"}, after a few seconds
/// @param data Data containing the rpc response that was sent by the cloud
void processTime(JsonDocument const & data) {
  time_t time = data["time"];
  // Time Alarms
  setTime(time);
  SET_TIME = false;
  printActualTime();
}

/************* End RPC callbacks *************/

/************* OTA callbacks *************/
/// @brief Update starting callback method that will be called as soon as the shared attribute firmware keys have been received and processed
/// and the moment before we subscribe the necessary topics for the OTA firmware update.
/// Is meant to give a moment were any additional processes or communication with the cloud can be stopped to ensure the update process runs as smooth as possible.
/// To ensure that calling the ThingsBoardSized::Cleanup_Subscriptions() method can be used which stops any receiving of data over MQTT besides the one for the OTA firmware update,
/// if this method is used ensure to call all subscribe methods again so they can be resubscribed, in the method passed to the finished_callback if the update failed and we do not restart the device
void update_starting_callback() {
  // Nothing to do
}

/// @brief End callback method that will be called as soon as the OTA firmware update, either finished successfully or failed.
/// Is meant to allow to either restart the device if the udpate was successfull or to restart any stopped services before the update started in the subscribed update_starting_callback
/// @param success Either true (update successful) or false (update failed)
void finished_callback(const bool & success) {
  if (success) {
    Serial.println("Done, Reboot now");
    ESP.restart();
    return;
  }
  Serial.println("Downloading firmware failed");
}

/// @brief Progress callback method that will be called every time our current progress of downloading the complete firmware data changed,
/// meaning it will be called if the amount of already downloaded chunks increased.
/// Is meant to allow to display a progress bar or print the current progress of the update into the console with the currently already downloaded amount of chunks and the total amount of chunks
/// @param current Already received and processs amount of chunks
/// @param total Total amount of chunks we need to receive and process until the update has completed
void progress_callback(const size_t & current, const size_t & total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}
/************* End OTA callbacks *************/

/************* Wifi Manager *************/
void setupWifiManager(bool DRD_DETECTED){
  // get device id from macAddress
  char deviceid[32] = "";
  byte macAddressArray[6];
  WiFi.macAddress(macAddressArray);
  getDeviceId(macAddressArray, 6, deviceid);

  wm.setDebugOutput(false);
  wm.debugPlatformInfo();

  //reset settings - for testing
  if (RESET_SETTINGS){
    mb.deleteFileData(WM_DATA_FILE);
    wm.resetSettings();
    wm.erase();
  }

  JsonDocument json;
  json = mb.loadData(WM_DATA_FILE);
  
  if (!json["ERROR"]){
    strcpy(THINGSBOARD_SERVER, json["THINGSBOARD_SERVER"]);
    //strcpy(THINGSBOARD_PORT, json["THINGSBOARD_PORT"]);
    strcpy(TOKEN, json["TOKEN"]);
    STAND_ALONE = json["STAND_ALONE"];
    TIME_TO_SEND_TELEMETRY = json["TIME_TO_SEND_TELEMETRY"];
  }

  WiFiManagerParameter custom_server("server", "MaterBox server", THINGSBOARD_SERVER, 40);
  //WiFiManagerParameter custom_mqtt_port("port", "port", THINGSBOARD_PORT, 6);
  WiFiManagerParameter custom_api_token("apikey", "Token", TOKEN, 32);
  WiFiManagerParameter device_type("devicetype", "Tipo", deviceName, 40, " readonly");
  WiFiManagerParameter device_id("deviceid", "Device Id", deviceid, 40, " readonly");

  // callbacks
  wm.setAPCallback(configModeCallback);
  wm.setWebServerCallback(bindServerCallback);
  wm.setSaveConfigCallback(saveWifiCallback);
  wm.setSaveParamsCallback(saveParamCallback);
  
  // add all your parameters here
  wm.addParameter(&custom_server);
//  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_api_token);
  wm.addParameter(&device_type);
  wm.addParameter(&device_id);

  // invert theme, dark
  wm.setDarkMode(true);

  std::vector<const char *> menu = {"wifi","sep","exit"};
  wm.setMenu(menu); // custom menu, pass vector

  wm.setCountry("US"); // crashing on esp32 2.0

  // set Hostname
  wm.setHostname(("WM_" + wm.getDefaultAPName()).c_str());
  // wm.setHostname("WM_RANDO_1234");

  // show password publicly in form
  wm.setShowPassword(true);
  
  if(!WMISBLOCKING){
    wm.setConfigPortalBlocking(false);
  }

  //sets timeout until configuration portal gets turned off
  wm.setConfigPortalTimeout(180);
  
  // This is sometimes necessary, it is still unknown when and why this is needed
  // but it may solve some race condition or bug in esp SDK/lib
  // wm.setCleanConnect(true); // disconnect before connect, clean connect
  wm.setBreakAfterConfig(true); // needed to use saveWifiCallback

  if(DRD_DETECTED || TEST_CP){
    Alarm.delay(1000);
    if(!wm.startConfigPortal("MaterBox IoT", "123456789")){
      mb.enqueueMessage("Failed to connect and hit timeout", "INFO");
    } else {
      mb.enqueueMessage("Wifi connected :)", "INFO");
      wifiInfo();
    }
  } else {
    if(!wm.autoConnect("MaterBox IoT", "123456789")){
      mb.enqueueMessage("Failed to connect and hit timeout", "INFO");
    } else {
      mb.enqueueMessage("Wifi connected :)", "INFO");
      wifiInfo();
    }
  }

  //read updated parameters
  strcpy(THINGSBOARD_SERVER, custom_server.getValue());
  strcpy(TOKEN, custom_api_token.getValue());
  //strcpy(THINGSBOARD_PORT, custom_mqtt_port.getValue());

  if (SAVE_PARAMS){
    JsonDocument json;
    json["THINGSBOARD_SERVER"] = THINGSBOARD_SERVER;
    //json["mqtt_port"] = mqtt_port;
    json["TOKEN"] = TOKEN;
    json["STAND_ALONE"] = STAND_ALONE;
    json["TIME_TO_SEND_TELEMETRY"] = TIME_TO_SEND_TELEMETRY;
    mb.saveData(json, WM_DATA_FILE);
  }
}

void saveWifiCallback(){
  mb.enqueueMessage("wm save settings Callback fired ", "INFO");
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  mb.enqueueMessage("wm config Mode Callback fired", "INFO");
}

void saveParamCallback(){
  mb.enqueueMessage("wm save Parameters Callback fired", "INFO");
  SAVE_PARAMS = true;
}

void bindServerCallback(){
  wm.server->on("/custom",handleRoute); // this is now crashing esp32 for some reason
  // wm.server->on("/info",handleRoute); // you can override wm!
}

void handleRoute(){
  wm.server->send(200, "text/plain", "hello from user code");
  mb.enqueueMessage("wm handle route", "INFO");
}

void wifiInfo(){
  // can contain gargbage on esp32 if wifi is not ready yet
  mb.enqueueMessage("Wifi debug data", "INFO");

  JsonDocument json;
  json["SAVED"] = (String)(wm.getWiFiIsSaved() ? "YES" : "NO");
  json["SSID"] = (String)wm.getWiFiSSID();
  json["Password"] = (String)wm.getWiFiPass();
  json["Hostname"] = (String)WiFi.getHostname();
  
  // WiFi.printDiag(Serial);
  mb.enqueueMessageJson(json, "INFO", true);
}
/************* End Wifi Manager *************/

void getDeviceId(byte macAddressArray[], unsigned int len, char buffer[]){
    for (unsigned int i = 0; i < len; i++){
        byte nib1 = (macAddressArray[i] >> 4) & 0x0F;
        byte nib2 = (macAddressArray[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

void setLocalTime(){
  mb.enqueueMessage("Request time from server", "RPC");
  
  RPC_Request_Callback callback(RPC_REQUEST_GET_CURRENT_TIME, &processTime, nullptr, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut);

  // Perform a request of the given RPC method. Optional responses are handled in processTime
  if (!rpc_request.RPC_Request(callback)) {
    mb.enqueueMessage("Failed to request for RPC", "ERROR");
  } else {
    mb.enqueueMessage("Request done", "INFO");
    SET_TIME = false;
  }
}

/// @brief Attribute request did not receive a response in the expected amount of microseconds 
void requestTimedOut() {
  Serial.printf("RPC request timed out did not receive a response in (%llu) microseconds. Ensure client is connected to the MQTT broker and that the RPC method actually exist on the device Rule chain\n", REQUEST_TIMEOUT_MICROSECONDS);
}

void printActualTime(){
  mb.enqueueMessage("Time: " + String(hour()) + ":" + String(minute()) + ":" + String(second()), "INFO");
}

void setTimeAlarms(int lOnHour, int lOnMin, int lOnSec, int lOffHour, int lOffMin, int lOffSec){
  if (ALARMS_ARE_SET){  
    Alarm.free(ALARM_ID_ON);
    Alarm.free(ALARM_ID_OFF);
  }
  JsonDocument json;
  if (lOnHour == 30){
    json = mb.loadData(LIGHTS_CONTROL_DATA_FILE);
    if (!json["ERROR"]){
      //Lights on time
      lOnHour = json["lOnHour"];
      lOnMin  = json["lOnMin"];
      lOnSec  = json["lOnSec"];
      
      //Lights off time
      lOffHour= json["lOffHour"];
      lOffMin = json["lOffMin"];
      lOffSec = json["lOffSec"];
      mb.enqueueMessage("Setting alarms from lights control data file", "INFO");
    } else {
      //Lights on time
      lOnHour = 6;
      lOnMin  = 0;
      lOnSec  = 0;
      
      //Lights off time
      lOffHour= 14;
      lOffMin = 0;
      lOffSec = 0;
      mb.enqueueMessage("Setting alarms from default values", "INFO");
    }
  } else {
      mb.enqueueMessage("Setting alarms from RPC call", "INFO");
      //Lights on time
      json["lOnHour"] = lOnHour;
      json["lOnMin"] = lOnMin;
      json["lOnSec"] = lOnSec;

      //Lights off time
      json["lOffHour"] = lOffHour;
      json["lOffMin"] = lOffMin;
      json["lOffSec"] = lOffSec;
      mb.enqueueMessage("Saving alarms info to ligths control data file", "INFO");
      mb.saveData(json, LIGHTS_CONTROL_DATA_FILE);
  }
  ALARM_ID_ON = Alarm.alarmRepeat(lOnHour, lOnMin, lOnSec, turnLightsOn);
  mb.enqueueMessage("Encender: " + String(lOnHour) + ":" + String(lOnMin) + ":" + String(lOnSec), "INFO");

  ALARM_ID_OFF = Alarm.alarmRepeat(lOffHour, lOffMin, lOffSec, turnLightsOff);
  mb.enqueueMessage("Apagar: " + String(lOffHour) + ":" + String(lOffMin) + ":" + String(lOffSec), "INFO");

//  Alarm.timerRepeat(15, Repeats);           // timer for every 15 seconds
  if(ALARM_ID_ON == 255 || ALARM_ID_OFF == 255){
    mb.enqueueMessage("Alarms not set. Try again", "WARN");
    Alarm.free(ALARM_ID_ON);
    Alarm.free(ALARM_ID_OFF);
    ALARMS_ARE_SET = false;
    SET_ALARMS = true;
  } else {
    mb.enqueueMessage("Alarm ON set. Id: " + String(ALARM_ID_ON), "INFO");
    mb.enqueueMessage("Alarm OFF set. Id: " + String(ALARM_ID_OFF), "INFO");
    ALARMS_ARE_SET = true;
    SET_ALARMS = false;
  }
}

void turnLightsOn(){
  mb.enqueueMessage("Triggered turn lights on alarm", "INFO");
  digitalWrite(Relay3, LOW);
  //tb.sendTelemetryData("lights", 1);
}

void turnLightsOff(){
  mb.enqueueMessage("Triggered turn lights off alarm", "INFO");
  digitalWrite(Relay3, HIGH);
  //tb.sendTelemetryData("lights", 0);
}

struct tm getTime() {
  struct tm timeinfo;
  int tz           = -6;
  int dst          = 0;
  time_t now       = time(nullptr);
  unsigned timeout = 5000; // try for timeout
  unsigned start   = millis();
  configTime(tz * 3600, dst * 3600, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync: ");
  while (now < 8 * 3600 * 2 ) { // what is this ?
    Alarm.delay(100);
    Serial.print(".");
    now = time(nullptr);
    if((millis() - start) > timeout){
      Serial.println("\n[ERROR] Failed to get NTP time.");
      timeinfo.tm_hour = -1;
      return timeinfo;
    }
  }
  Serial.println("");
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.println(asctime(&timeinfo));

  return timeinfo;
}