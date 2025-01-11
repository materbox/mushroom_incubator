/************* Includes *************/
#include <FS.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
//#include <time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>  // 
#include <stdio.h>
#include <LittleFS.h>             //https://github.com/esp8266/Arduino/tree/master/libraries/LittleFS
/************* End Includes *************/

/************* Define default values *************/
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
#define CURRENT_FIRMWARE_TITLE    "MB-MushroomIncubator"
#define CURRENT_FIRMWARE_VERSION  "0.1.0"
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
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>          //https://github.com/thingsboard/thingsboard-arduino-sdk

char THINGSBOARD_SERVER[40] = "materbox.io";
char TOKEN[40] = "TEST_TOKEN";

constexpr uint16_t THINGSBOARD_PORT = 1883U;
// Maximum size packets will ever be sent or received by the underlying MQTT client
constexpr uint16_t MAX_MESSAGE_SIZE = 256U;
// RPC
// Statuses for subscribing to rpc
bool subscribed = false;
constexpr char RPC_RESPONSE_KEY[] = "RPC_RESPONSE_KEY";
constexpr char RPC_REQUEST_GET_CURRENT_TIME[] = "getCurrentTime";
constexpr const char RPC_SET_HOURS_OF_LIGHT[] = "setHoursOfLight";
constexpr const char RPC_TIME_TO_SEND_TELEMETRY[] = "timeToSendTelemetry";

WiFiClient espClient;
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
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
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

/************* End Relay control *************/
/************* End Lights control *************/

/************* Sensor BH1750 *************/
#include <Wire.h>
#include <BH1750.h>
BH1750 luxMeter;
bool BH1750_DETECTED = false;
/************* End Sensor BH1750 *************/

/************* Sensor BME280 *************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
bool BME280_DETECTED = false;
/************* End Sensor BME280 *************/

/************* Sensor MQ-series *************/
bool MQ_DETECTED = false;
bool MQ_DATA_DELETE  = false; // delete MQ_DATA_FILE - for testing
String MQ_DATA_FILE = "mqdata.txt";
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

/************* Prototype functions *************/
void setTimeAlarms(int lOnHour=30, int lOnMin=0, int lOnSec=0, int lOffHour=0, int lOffMin=0, int lOffSec=0);
/************* End Prototype functions *************/

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  while (!Serial) ; // wait for Arduino Serial Monitor
  enqueueMessage("Starting", "INFO");
  enqueueMessage("Test", "ERROR");
  enqueueMessage("Test", "WARN");

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  if (drd->detectDoubleReset()) {
    enqueueMessage("Double Reset Detected", "INFO");
      DRD_DETECTED = true;
    } else {
      enqueueMessage("No Double Reset Detected", "INFO");
      DRD_DETECTED = false;
    }

  setupWifiManager(DRD_DETECTED);

  setupBh1750Sensor();
  setupBme280Sensor();
  setupMqSensor();

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

void loop() {
  if(millis()-mtime > (TIME_TO_SEND_TELEMETRY * 1000)){
    if(WiFi.status() == WL_CONNECTED){
      if (!tb.connected()) {
        // Reconnect to the ThingsBoard server,
        // if a connection was disrupted or has not yet been established
        enqueueMessage("Connecting to: " + String(THINGSBOARD_SERVER) + " with token " + String(TOKEN), "INFO");
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          enqueueMessage("Failed to connect", "ERROR");
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
        sendTelemetryJson(getBh1750DataJson());
        sendTelemetryJson(getBme280DataJson());
        sendTelemetryJson(getMqDataJson());
      }
    } else {
      enqueueMessage("WiFi.status() == WL_CONNECTED " + String(WiFi.status()), "ERROR");
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

  luxMeter.begin();
  enqueueMessage("BH1750 Test begin", "INFO");
  BH1750_DETECTED = true;
}

void setupMqSensor(){
  if(MQ_DATA_DELETE){
    deleteFileData(MQ_DATA_FILE);
  }
  float calcR0 = 0;
  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init();
  MQ135.setRL(2); //If the RL value is different from 10K, assign new RL value

  JsonDocument json;
  json = loadData(MQ_DATA_FILE);
  
  if (!json["ERROR"]){
  //if (json.containsKey("R0_VALUE")){
    calcR0 = json["R0_VALUE"];
    enqueueMessage("MQ load config calibration R0 = " + String(calcR0), "INFO");
  } else {
    calcR0 = mqSensorCalibration();
    enqueueMessage("MQ sensor calibration using R0 = " + String(calcR0), "INFO");
  }

  if(isinf(calcR0)) {
    enqueueMessage("R0 is infinite (Open circuit detected)", "ERROR");
    MQ_DETECTED = false;
    while(1);
  } else if(calcR0 == 0){
    enqueueMessage("R0 is zero (Analog pin shorts to ground)", "ERROR");
    MQ_DETECTED = false;
    while(1);
  } else {
    MQ135.setR0(calcR0); // Se evita calibrar por valores muy distintos entre incubadoras
    enqueueMessage("MQ135 sensor started", "INFO");
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
  enqueueMessage("MQ135 sensor is being Calibrating, please wait", "INFO");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  calcR0 = calcR0/10;
  
  JsonDocument json;
  json["R0_VALUE"] = calcR0;
  saveData(json, MQ_DATA_FILE);
  /*****************************  MQ CAlibration ********************************************/ 
  return calcR0;
}

void setupBme280Sensor(){
  BME280_DETECTED = bme.begin(0x76);  
  if (!BME280_DETECTED) {
    enqueueMessage("Could not find a valid BME280 sensor!", "ERROR");
  } else {
    enqueueMessage("BME280 sensor started", "INFO");
  }
}

void sendTelemetryJson(const JsonVariantConst &data){
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
 enqueueMessage("Subscribing for RPC", "INFO");

  const std::array<RPC_Callback, 2U> callbacks = {
    RPC_Callback{ RPC_SET_HOURS_OF_LIGHT,                   processSetTimeAlarms},
    RPC_Callback{ RPC_TIME_TO_SEND_TELEMETRY,               processTimeToSendTelemetry}
  };

  // Perform a subscription. All consequent data processing will happen in
  // processTemperatureChange() and processSwitchChange() functions,
  // as denoted by callbacks array.
  if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
    enqueueMessage("Failed to subscribe for RPC", "ERROR");
    return;
  }
  enqueueMessage("Subscribe done", "INFO");
  subscribed = true;
}

/// @brief Processes function for RPC call "SetTimeAlarms"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
RPC_Response processSetTimeAlarms(const RPC_Data &data) {
  enqueueMessage("Received RPC call SetTimeAlarms", "RCP");

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

  return RPC_Response(RPC_RESPONSE_KEY, 42);
}

/// @brief Processes function for RPC call "SetTimeAlarms"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
RPC_Response processTimeToSendTelemetry(const RPC_Data &data) {
  enqueueMessage("Received timeToSendTelemetry method", "RCP");

  // Process data
  //Lights on time
  TIME_TO_SEND_TELEMETRY = data["TIME_TO_SEND_TELEMETRY"];
  
  enqueueMessage("Send telemetry every " + String(TIME_TO_SEND_TELEMETRY) + " seconds", "RCP");
  return RPC_Response(RPC_RESPONSE_KEY, 42);
}

/// @brief Processes function for RPC response of "getCurrentTime".
/// If no response is set the callback is called with {"error": "timeout"}, after a few seconds
/// @param data Data containing the rpc response that was sent by the cloud
void processTime(const JsonVariantConst &data) {
  time_t time = data["time"];
  // Time Alarms
  setTime(time);
  SET_TIME = false;
  printActualTime();
}

/************* End RPC callbacks *************/

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
    deleteFileData(WM_DATA_FILE);
    wm.resetSettings();
    wm.erase();
  }

  JsonDocument json;
  json = loadData(WM_DATA_FILE);
  
  if (!json["ERROR"]){
    strcpy(THINGSBOARD_SERVER, json["THINGSBOARD_SERVER"]);
    //strcpy(THINGSBOARD_PORT, json["THINGSBOARD_PORT"]);
    strcpy(TOKEN, json["TOKEN"]);
    STAND_ALONE = json["STAND_ALONE"];
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
      enqueueMessage("Failed to connect and hit timeout", "INFO");
    } else {
      enqueueMessage("Wifi connected :)", "INFO");
      wifiInfo();
    }
  } else {
    if(!wm.autoConnect("MaterBox IoT", "123456789")){
      enqueueMessage("Failed to connect and hit timeout", "INFO");
    } else {
      enqueueMessage("Wifi connected :)", "INFO");
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
    saveData(json, WM_DATA_FILE);
  }
}

void saveWifiCallback(){
  enqueueMessage("wm save settings Callback fired ", "INFO");
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  enqueueMessage("wm config Mode Callback fired", "INFO");
}

void saveParamCallback(){
  enqueueMessage("wm save Parameters Callback fired", "INFO");
  SAVE_PARAMS = true;
}

void bindServerCallback(){
  wm.server->on("/custom",handleRoute); // this is now crashing esp32 for some reason
  // wm.server->on("/info",handleRoute); // you can override wm!
}

void handleRoute(){
  wm.server->send(200, "text/plain", "hello from user code");
  enqueueMessage("wm handle route", "INFO");
}

void wifiInfo(){
  // can contain gargbage on esp32 if wifi is not ready yet
  enqueueMessage("Wifi debug data", "INFO");

  JsonDocument json;
  json["SAVED"] = (String)(wm.getWiFiIsSaved() ? "YES" : "NO");
  json["SSID"] = (String)wm.getWiFiSSID();
  json["Password"] = (String)wm.getWiFiPass();
  json["Hostname"] = (String)WiFi.getHostname();
  
  // WiFi.printDiag(Serial);
  enqueueMessageJson(json, "INFO", true);
}
/************* End Wifi Manager *************/
// Save data to eeprom in the specific file
void saveData(JsonDocument json, String fileName) {
      enqueueMessage("Saving data", "INFO");
      File dataFile = LittleFS.open("/" + fileName, "w");
      if (!dataFile) {
        enqueueMessage("failed to open " + fileName + " for writing", "ERROR");
      } else {
        serializeJson(json, dataFile);
      }
      dataFile.close();
}

JsonDocument loadData(String fileName) {
  enqueueMessage("Mounting " + fileName, "INFO");
  JsonDocument json;
  //https://www.hackster.io/Neutrino-1/littlefs-read-write-delete-using-esp8266-and-arduino-ide-867180
  //read file from FileSistem
  if(!LittleFS.begin()){
    enqueueMessage("Mounting file", "ERROR");
    Alarm.delay(1000);
    json["ERROR"] = true;
  } else {
    enqueueMessage("Mounted file system", "INFO");
    if (LittleFS.exists("/" + fileName)) {
      //file exists, reading and loading
      enqueueMessage("reading " + fileName, "INFO");
      File dataFile = LittleFS.open("/" + fileName, "r");
      if (dataFile) {
        enqueueMessage("Opened " + fileName, "INFO");
        size_t size = dataFile.size();
        
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        dataFile.readBytes(buf.get(), size);
        auto deserializeError = deserializeJson(json, buf.get());
        dataFile.close();

        if (!deserializeError) {
          enqueueMessage("Data loaded", "INFO");
          json["ERROR"] = false;
        } else {
          enqueueMessage("failed to load " + fileName, "ERROR");
          json["ERROR"] = true;
        }
      }
    } else {
      json["ERROR"] = true;
    }
  }
  return json;
}

void enqueueMessage(String message, String Type){
  Serial.print("[");
  Serial.print(Type);
  Serial.print("] ");
  Serial.println(message);
}

void enqueueMessageJson(JsonDocument json, String Type, bool Pretty){
  Serial.print("[");
  Serial.print(Type);
  Serial.print("] ");
  if(Pretty){
    serializeJsonPretty(json, Serial);
  } else {
    serializeJson(json, Serial);
  }
  
  Serial.println();
}

void deleteFileData(String fileName){
   //Remove the file
   LittleFS.remove("/" + fileName);
}

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
  enqueueMessage("Request time from server", "RPC");
  const RPC_Request_Callback callback(RPC_REQUEST_GET_CURRENT_TIME, &processTime);
  // Perform a request of the given RPC method. Optional responses are handled in processTime
  if (!tb.RPC_Request(callback)) {
    enqueueMessage("Failed to request for RPC", "ERROR");
  } else {
    enqueueMessage("Request done", "INFO");
    SET_TIME = false;
  }
}

void printActualTime(){
  enqueueMessage("Time: " + String(hour()) + ":" + String(minute()) + ":" + String(second()), "INFO");
}

void setTimeAlarms(int lOnHour, int lOnMin, int lOnSec, int lOffHour, int lOffMin, int lOffSec){
  if (ALARMS_ARE_SET){  
    Alarm.free(ALARM_ID_ON);
    Alarm.free(ALARM_ID_OFF);
  }
  bool SAVE_DATA =  false;
  JsonDocument json;
  if (lOnHour == 30){
    json = loadData(LIGHTS_CONTROL_DATA_FILE);
    if (!json["ERROR"]){
      //Lights on time
      lOnHour = json["lOnHour"];
      lOnMin  = json["lOnMin"];
      lOnSec  = json["lOnSec"];
      
      //Lights off time
      lOffHour= json["lOffHour"];
      lOffMin = json["lOffMin"];
      lOffSec = json["lOffSec"];
      enqueueMessage("Setting alarms from lights control data file", "INFO");
    } else {
      //Lights on time
      lOnHour = 6;
      lOnMin  = 0;
      lOnSec  = 0;
      
      //Lights off time
      lOffHour= 14;
      lOffMin = 0;
      lOffSec = 0;
      enqueueMessage("Setting alarms from default values", "INFO");
    }
  } else {
      enqueueMessage("Setting alarms from RPC call", "INFO");
      //Lights on time
      json["lOnHour"] = lOnHour;
      json["lOnMin"] = lOnMin;
      json["lOnSec"] = lOnSec;

      //Lights off time
      json["lOffHour"] = lOffHour;
      json["lOffMin"] = lOffMin;
      json["lOffSec"] = lOffSec;
      enqueueMessage("Saving alarms info to ligths control data file", "INFO");
      saveData(json, LIGHTS_CONTROL_DATA_FILE);
  }
  ALARM_ID_ON = Alarm.alarmRepeat(lOnHour, lOnMin, lOnSec, turnLightsOn);
  enqueueMessage("Encender: " + String(lOnHour) + ":" + String(lOnMin) + ":" + String(lOnSec), "INFO");

  ALARM_ID_OFF = Alarm.alarmRepeat(lOffHour, lOffMin, lOffSec, turnLightsOff);
  enqueueMessage("Apagar: " + String(lOffHour) + ":" + String(lOffMin) + ":" + String(lOffSec), "INFO");

//  Alarm.timerRepeat(15, Repeats);           // timer for every 15 seconds
  if(ALARM_ID_ON == 255 || ALARM_ID_OFF == 255){
    enqueueMessage("Alarms not set. Try again", "WARN");
    Alarm.free(ALARM_ID_ON);
    Alarm.free(ALARM_ID_OFF);
    ALARMS_ARE_SET = false;
    SET_ALARMS = true;
  } else {
    enqueueMessage("Alarm ON set. Id: " + String(ALARM_ID_ON), "INFO");
    enqueueMessage("Alarm OFF set. Id: " + String(ALARM_ID_OFF), "INFO");
    ALARMS_ARE_SET = true;
    SET_ALARMS = false;
  }
}

void turnLightsOn(){
  enqueueMessage("Triggered turn lights on alarm", "INFO");
  digitalWrite(Relay3, LOW);
  //tb.sendTelemetryData("lights", 1);
}

void turnLightsOff(){
  enqueueMessage("Triggered turn lights off alarm", "INFO");
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