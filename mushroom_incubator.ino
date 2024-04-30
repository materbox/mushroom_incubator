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
const int TIME_TO_SEND_TELEMETRY  = 30; //every x seconds to send tellemetry
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
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "example_set_switch";

constexpr const char RPC_TEMPERATURE_ALARM_MANAGEMENT_METHOD[] = "set_temperature_alarm";
constexpr const char RPC_CO2_ALARM_MANAGEMENT_METHOD[] = "set_co2_alarm";
constexpr const char RPC_ALARM_STATE_KEY[] = "state";

constexpr const char RPC_TEMPERATURE_KEY[] = "temp";
constexpr const char RPC_SWITCH_KEY[] = "switch";
constexpr const char RPC_RESPONSE_KEY[] = "example_response";
constexpr char RPC_REQUEST_CALLBACK_METHOD_NAME[] = "getCurrentTime";

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
/************* End Wifi Manager *************/

/************* Lights control *************/
//Time Alarms
bool SET_TIME   = true;
bool SET_ALARMS = true;

//Lights on time
char lOnHour[]   = "6";
char lOnMin[]    = "0";
char lOnSec[]    = "0";
//Lights off time
char lOffHour[]  = "14";
char lOffMin[]   = "0";
char lOffSec[]   = "0";

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

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println("\n Starting");
  Serial.println("Information- - TEST");
  Serial.println("[ERROR]  TEST");
  Serial.println("[INFO] TEST");  

  setupBh1750Sensor();
  setupBme280Sensor();
  setupMqSensor();

/************* Relay control *************/
//  pinMode(Relay1, OUTPUT);
//  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  
  //During Starting all Relays should TURN OFF
//  digitalWrite(Relay1, HIGH);
//  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);

/************* End Relay control *************/


  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  if (drd->detectDoubleReset()) {
      Serial.println("[INFO] Double Reset Detected");
      DRD_DETECTED = true;
    } else {
      Serial.println("[INFO] No Double Reset Detected");
      DRD_DETECTED = false;
    }

  setupWifiManager(DRD_DETECTED);
}

void loop() {
  if(millis()-mtime > (TIME_TO_SEND_TELEMETRY * 1000)){
    if(WiFi.status() == WL_CONNECTED){
      if (!tb.connected()) {
        // Reconnect to the ThingsBoard server,
        // if a connection was disrupted or has not yet been established
        Serial.printf("[INFO] Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          Serial.println("[ERROR] Failed to connect");
          subscribed = false;
        }
      } else {
          if (!subscribed) {
            rpcSubscribe();
        }
        if (SET_TIME){
          setTime();
        }
        sendTelemetryJson(getBh1750DataJson());
        sendTelemetryJson(getBme280DataJson());
        sendTelemetryJson(getMqDataJson());
      }
    } else {
      Serial.println("No Wifi");
      Serial.println("WiFi.status() == WL_CONNECTED ..." + WiFi.status());
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

  Serial.println(F("BH1750 Test begin"));
  BH1750_DETECTED = true;
}

void setupMqSensor(){
 //Set math model to calculate the PPM concentration and the value of constants
  if(MQ_DATA_DELETE){
    deleteFileData(MQ_DATA_FILE);
  }
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
  /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
      while(1);
  }
  /*****************************  MQ CAlibration ********************************************/ 
  MQ_DETECTED = true;
}

void setupBme280Sensor(){
  BME280_DETECTED = bme.begin(0x76);  
  if (!BME280_DETECTED) {
    Serial.println("Could not find a valid BME280 sensor!");
  } else {
    Serial.println("BME280 sensor started");
  }
}

void sendTelemetryJson(const JsonVariantConst &data){
  serializeJsonPretty(data, Serial);
  Serial.println();
  tb.sendTelemetryJson(data, Helper::Measure_Json(data));
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
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */
}

JsonDocument getBme280DataJson(){
  float tempBuff; //Buffer for the raw temperature taken by the sensor
  float humidityBuff; //Buffer for the raw humidity taken by the sensor
  float pressureBuff; //Buffer for the raw pressure taken by the sensor
  float allTemp; //The sum of all the temperatures taken by the sensor
  float allHumidity; //The sum of all the humidity taken by the sensor
  float allPressure; //The sum of all the pressure taken by the sensor
  int i; //Random variable used to control the loops

  JsonDocument json;

  allTemp = 0; //Restart the temperature sum
  allHumidity = 0; //Restart the humidity sum
  allPressure = 0; //Restart the pressure sum

  for(i = 0;i < 10;i++){
    Alarm.delay(5); //delay between each reading to avoid an error
    tempBuff = bme.readTemperature();
    humidityBuff = bme.readHumidity();
    pressureBuff = bme.readPressure();
    
    allTemp = allTemp + tempBuff;
    allHumidity = allHumidity + humidityBuff;
    allPressure = allPressure + pressureBuff;
  }

  json["temperature"] = allTemp / 10; //Dividing to get means
  json["humidity"] = allHumidity / 10; //Dividing to get means
  json["pressure"] = allPressure / 10; //Dividing to get means
  
  return json;
}

/************* RPC callbacks *************/
void rpcSubscribe(){
  Serial.println("Subscribing for RPC...");

  const std::array<RPC_Callback, 2U> callbacks = {
    RPC_Callback{ RPC_TEMPERATURE_ALARM_MANAGEMENT_METHOD,  processTemperatureAlarm },
    RPC_Callback{ RPC_CO2_ALARM_MANAGEMENT_METHOD,          processCo2Alarm }
  };

  // Perform a subscription. All consequent data processing will happen in
  // processTemperatureChange() and processSwitchChange() functions,
  // as denoted by callbacks array.
  if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
    Serial.println("Failed to subscribe for RPC");
    return;
  }
  Serial.println("Subscribe done");
  subscribed = true;
}

/// @brief Processes function for RPC call "set_temperature_alarm"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
RPC_Response processTemperatureAlarm(const RPC_Data &data) {
  Serial.println("Received the set temperature alarm RPC method");

  // Process data
  const int state = data[RPC_ALARM_STATE_KEY];
  if (state){
    Serial.println("Alarma de temperatura creada");
  }
  if (!state){
    Serial.println("Alarma de temperatura borrada");
  }
  return RPC_Response(RPC_RESPONSE_KEY, 42);
}

/// @brief Processes function for RPC call "set_co2_alarm"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
RPC_Response processCo2Alarm(const RPC_Data &data) {
  Serial.println("Received the set CO2 alarm RPC method");

  // Process data
  const int state = data[RPC_ALARM_STATE_KEY];
  if (state){
    Serial.println("Alarma de CO2 creada");
  }
  if (!state){
    Serial.println("Alarma de CO2 borrada");
  }
  return RPC_Response(RPC_RESPONSE_KEY, 42);
}

/// @brief Processes function for RPC response of "getCurrentTime".
/// If no response is set the callback is called with {"error": "timeout"}, after a few seconds
/// @param data Data containing the rpc response that was sent by the cloud
void processTime(const JsonVariantConst &data) {
  time_t time = data["time"];
  Serial.print("Time: ");
  Serial.println(time);
  // Time Alarms
  setTime (time);
  // (hour(24 format),minutes,seconds,month,day,year) set time to Saturday 8:29:00am Jan 1 2024
  //setTime(timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec,timeinfo.tm_mon,timeinfo.tm_mday,timeinfo.tm_year + 1900);+

  SET_TIME = false;
  if(SET_ALARMS){
    setTimeAlarms();
  }
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
    //Lights on time
    strcpy(lOnHour, json["lOnHour"]);
    strcpy(lOnMin, json["lOnMin"]);
    strcpy(lOnSec, json["lOnSec"]);
    //Lights off time
    strcpy(lOffHour, json["lOffHour"]);
    strcpy(lOffMin, json["lOffMin"]);
    strcpy(lOffSec, json["lOffSec"]);
  }

  WiFiManagerParameter custom_server("server", "MaterBox server", THINGSBOARD_SERVER, 40);
  //WiFiManagerParameter custom_mqtt_port("port", "port", THINGSBOARD_PORT, 6);
  WiFiManagerParameter custom_api_token("apikey", "Token", TOKEN, 32);
  WiFiManagerParameter device_type("devicetype", "Tipo", deviceName, 40, " readonly");
  WiFiManagerParameter device_id("deviceid", "Device Id", deviceid, 40, " readonly");

  //Lights on time
  WiFiManagerParameter custom_lights_on_hour("houron", "Encender: Hora", lOnHour, 4);
  WiFiManagerParameter custom_lights_on_min("minon", "Encender: Minuto", lOnMin, 4);
  WiFiManagerParameter custom_lights_on_sec("secon", "Encender: Segundo", lOnSec, 4);

  //Lights off time
  WiFiManagerParameter custom_lights_off_hour("houroff", "Apagar: Hora", lOffHour, 4);
  WiFiManagerParameter custom_lights_off_min("minoff", "Apagar: Minuto", lOffMin, 4);
  WiFiManagerParameter custom_lights_off_sec("secoff", "Apagar: Segundo", lOffSec, 4);

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

  //Lights on time
  wm.addParameter(&custom_lights_on_hour);
  wm.addParameter(&custom_lights_on_min);
  wm.addParameter(&custom_lights_on_sec);

  //Lights off time
  wm.addParameter(&custom_lights_off_hour);
  wm.addParameter(&custom_lights_off_min);
  wm.addParameter(&custom_lights_off_sec);

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
      Serial.println("[INFO] Failed to connect and hit timeout");
    } else {
      Serial.println("[INFO] Wifi connected:)");
      wifiInfo();
    }
  } else {
    if(!wm.autoConnect("MaterBox IoT", "123456789")){
      Serial.println("[INFO] Failed to connect and hit timeout");
    } else {
      Serial.println("[INFO] Wifi connected:)");
      wifiInfo();
    }
  }

  //read updated parameters
  strcpy(THINGSBOARD_SERVER, custom_server.getValue());
  strcpy(TOKEN, custom_api_token.getValue());
  //strcpy(THINGSBOARD_PORT, custom_mqtt_port.getValue());

  //Lights on time
  strcpy(lOnHour, custom_lights_on_hour.getValue());
  strcpy(lOnMin, custom_lights_on_min.getValue());
  strcpy(lOnSec, custom_lights_on_sec.getValue());

  //Lights off time
  strcpy(lOffHour, custom_lights_off_hour.getValue());
  strcpy(lOffMin, custom_lights_off_min.getValue());
  strcpy(lOffSec, custom_lights_off_sec.getValue());

  if (SAVE_PARAMS){
    JsonDocument json;
    json["THINGSBOARD_SERVER"] = THINGSBOARD_SERVER;
    //json["mqtt_port"] = mqtt_port;
    json["TOKEN"] = TOKEN;
    json["STAND_ALONE"] = STAND_ALONE;
    //Lights on time
    json["lOnHour"] = lOnHour;
    json["lOnMin"] = lOnMin;
    json["lOnSec"] = lOnSec;
    //Lights off time
    json["lOffHour"] = lOffHour;
    json["lOffMin"] = lOffMin;
    json["lOffSec"] = lOffSec;

    saveData(json, WM_DATA_FILE);
    //saveConfigData();
  }
}

void saveWifiCallback(){
  Serial.println("[CALLBACK] save settings Callback fired");

}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("[CALLBACK] configModeCallback fired");

}

void saveParamCallback(){
  Serial.println("[CALLBACK] saveParamCallback fired");
}

void bindServerCallback(){
  wm.server->on("/custom",handleRoute); // this is now crashing esp32 for some reason
  // wm.server->on("/info",handleRoute); // you can override wm!
}

void handleRoute(){
  Serial.println("[HTTP] handle route");
  wm.server->send(200, "text/plain", "hello from user code");
}

void wifiInfo(){
  // can contain gargbage on esp32 if wifi is not ready yet
  Serial.println("[WIFI] WIFI INFO DEBUG");
  // WiFi.printDiag(Serial);
  Serial.println("[WIFI] SAVED: " + (String)(wm.getWiFiIsSaved() ? "YES" : "NO"));
  Serial.println("[WIFI] SSID: " + (String)wm.getWiFiSSID());
  Serial.println("[WIFI] PASS: " + (String)wm.getWiFiPass());
  Serial.println("[WIFI] HOSTNAME: " + (String)WiFi.getHostname());
}
/************* End Wifi Manager *************/
// Save data to eeprom in the specific file
void saveData(JsonDocument json, String fileName) {
      printConfigInfo("Saving data");
      File dataFile = LittleFS.open("/" + fileName, "w");
      if (!dataFile) {
        Serial.println("failed to open config file for writing");
      } else {
        serializeJson(json, dataFile);     
      }
      dataFile.close();
      Serial.println();
}

JsonDocument loadData(String fileName) {
  JsonDocument json;
  //https://www.hackster.io/Neutrino-1/littlefs-read-write-delete-using-esp8266-and-arduino-ide-867180
  //read file from FileSistem
  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    //Print the error on display
    Serial.println("Mounting Error");
    Alarm.delay(1000);
    json["ERROR"] = true;
  } else {
    Serial.println("mounted file system");
    if (LittleFS.exists("/" + fileName)) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File dataFile = LittleFS.open("/" + fileName, "r");
      if (dataFile) {
        Serial.println("opened data file");
        size_t size = dataFile.size();
        
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        dataFile.readBytes(buf.get(), size);
        auto deserializeError = deserializeJson(json, buf.get());
        dataFile.close();
        json["ERROR"] = false;

        if (!deserializeError) {
          printConfigInfo("Data loaded");
        } else {
          Serial.println("failed to load data file");
          json["ERROR"] = true;
        }
      }
    } else {
      json["ERROR"] = true;
    }
  }
  return json;
}

void printConfigInfo(String FROM){
  Serial.println("[INFO] Print data fired from: " + FROM);
  Serial.println("\tthingsboard server: " + String(THINGSBOARD_SERVER));
  Serial.println("\ttoken: " + String(TOKEN));
  Serial.print("\tstand_alone: ");
  Serial.println((STAND_ALONE) ? "true" : "false");
}

void deleteConfigData(){
   //Remove the file
   LittleFS.remove("/config.json");
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

void setTime(){
  Serial.println("[RPC] request time from server...");
  const RPC_Request_Callback callback(RPC_REQUEST_CALLBACK_METHOD_NAME, &processTime);
  // Perform a request of the given RPC method. Optional responses are handled in processTime
  if (!tb.RPC_Request(callback)) {
    Serial.println("Failed to request for RPC");
  } else {
    Serial.println("Request done");
    SET_TIME = false;
  }
}

void setTimeAlarms(){
  Serial.println("Setting alarms time");
  // create the alarms, to trigger at specific times
  // 8:00pm every day (8,30,0, MorningAlarm)
  Alarm.alarmRepeat(atoi(lOnHour),atoi(lOnMin),atoi(lOnSec), turnLightsOn);
  Alarm.alarmRepeat(atoi(lOffHour),atoi(lOffMin),atoi(lOffSec), turnLightsOff);
  Alarm.timerRepeat(15, Repeats);           // timer for every 15 seconds
  SET_ALARMS = false;

}

void Repeats() {
  Serial.println("15 second timer");
  if (!state){
    digitalWrite(Relay3, LOW);
    state = true;
  } else {
    digitalWrite(Relay3, HIGH);
    state = false;
  }
}

void turnLightsOn(){
  Serial.println("Alarm: - turn lights on");
  digitalWrite(Relay3, LOW);
  tb.sendTelemetryData("lights", 1);
}

void turnLightsOff(){
  Serial.println("Alarm: - turn lights off");
  digitalWrite(Relay3, HIGH);
  tb.sendTelemetryData("lights", 0);
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