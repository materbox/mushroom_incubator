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
constexpr char CURRENT_FIRMWARE_VERSION[] = "0.1.1";
const char* deviceName            = "MB-Mushroom-Incubator";
unsigned long mtime               = 0;
unsigned long TIME_TO_SEND_TELEMETRY  = 30; //every x seconds to send tellemetry

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
#include <Arduino_ESP8266_Updater.h>
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
constexpr const char RPC_SET_RELAY[] = "setFanRelay";
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

// Initalize the Updater client instance used to flash binary to flash memory
Arduino_ESP8266_Updater updater;
// Statuses for updating

/************* End Thingsboard *************/

/************* System Config *************/
struct sysConfig {
  uint32_t magic;               // Firma de integridad: 0x4D42494E ("MBIN")
  char     server[40];          // Servidor ThingsBoard
  char     token[40];           // Token del dispositivo
  uint32_t timeToSendTelemetry; // Intervalo de telemetría en segundos (default 30)
  uint8_t  lOnHour, lOnMin, lOnSec;   // Horario encendido luces (default 06:00:00)
  uint8_t  lOffHour, lOffMin, lOffSec; // Horario apagado luces (default 18:00:00)
  float    mqR0Value;           // Valor R0 del MQ-135 (default 0.0)
  float    mqCleanAirRatio;     // Relación R_s/R_0 en aire limpio (default 3.6)
};

sysConfig config;
constexpr char CONFIG_BIN_FILE[] = "config.bin";
/************* End System Config *************/

/************* Wifi Manager *************/
const char* modes[] = { "NULL", "STA", "AP", "STA+AP" };

WiFiManager wm;
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
//Time Alarms
bool SET_TIME     = true;
bool TIME_SET     = false;
bool SET_ALARMS   = true;
bool ALARMS_ARE_SET = false;
int ALARM_ID_ON;
int ALARM_ID_OFF;

/************* Relay control *************/
bool state = false;
//using namespace ace_button;
//#define Relay1 5   // GPIO5-D1   morado
//#define Relay2 4   // GPIO4-D2   naranja
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

/************* Sensor DS18B20 *************/
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13 // GPIO13 = D7

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int numberOfDevices; 
DeviceAddress tempDeviceAddress; 
bool DS18B20_DETECTED = false;
/************* End Sensor DS18B20 *************/

/************* Sensor MQ-series *************/
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
MQUnifiedsensor MQ135(BOARD, VOLTAGE_RESOLUTION, ADC_BIT_RESOLUTION, MQ_ANALOG_PIN, MQ_TYPE);
/************* End Sensor MQ-series *************/

MATERBOX mb;
/************* Prototype functions *************/
void setTimeAlarms(int lOnHour=30, int lOnMin=0, int lOnSec=0, int lOffHour=0, int lOffMin=0, int lOffSec=0);
JsonDocument getMqDataJson(float temperature = NAN, float humidity = NAN);

/************* End Prototype functions *************/

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  while (!Serial) ; // wait for Arduino Serial Monitor
  mb.enqueueMessage(F("Starting"), F("INFO"));

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  if (drd->detectDoubleReset()) {
    mb.enqueueMessage(F("Double Reset Detected"), F("INFO"));
      DRD_DETECTED = true;
    } else {
      mb.enqueueMessage(F("No Double Reset Detected"), F("INFO"));
      DRD_DETECTED = false;
    }

  mb.begin();
  initSystemConfig();
  setupWifiManager(DRD_DETECTED);

  setupBh1750Sensor();
  setupBme280Sensor();
  setupDs18b20Sensor();
  setupMqSensor();
  setupRelay();

  mtime = millis();
}

void loop() {
  unsigned long now = millis(); // Obtiene el tiempo actual
  bool tbconnected = false;
  float bme280Temperature = NAN;
  float bme280Humidity = NAN;

  if (WiFi.status() == WL_CONNECTED) {
    if (!tb.connected()) {
      mb.enqueueMessagef("INFO", "Connecting to: %s with token %s", THINGSBOARD_SERVER, TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        mb.enqueueMessage(F("Failed to connect"), F("ERROR"));
        tbconnected = false;
        subscribed = false;
      } else {
        mb.enqueueMessage(F("Server connected"), F("INFO"));
        tbconnected = true;
        tb.sendAttributeData("TimeToSendTelemetry", TIME_TO_SEND_TELEMETRY);
        mb.enqueueMessagef("INFO", "Send telemetry every %lu seconds", TIME_TO_SEND_TELEMETRY);
        fanOff(); // if new, create fanState = off attribute
      }
    } else {
      tbconnected = true;
    }

    if (tbconnected && !subscribed) {
      rpcSubscribe();
    }
    if (SET_TIME){ //Get current time if not set
      setLocalTime();
    }
    if(SET_ALARMS && TIME_SET){ //Set hours of light
      setTimeAlarms();
    }

    if (tb.connected() && (now - mtime >= (TIME_TO_SEND_TELEMETRY * 1000))) {
      mtime = now;

      if (BH1750_DETECTED){
        sendTelemetryJson(getBh1750DataJson());
      } else {
        if (checkBh1750Sensor()) {
          BH1750_DETECTED = true;
          mb.enqueueMessage(F("BH1750 sensor started"), F("INFO"));
          sendTelemetryJson(getBh1750DataJson());
        }
      }

      if (BME280_DETECTED){
        JsonDocument jsonBme280 = getBme280DataJson();
        sendTelemetryJson(jsonBme280);
        bme280Temperature = jsonBme280["temperature"];
        bme280Humidity = jsonBme280["humidity"];
      } else {
        if (checkBme280Sensor()) {
          BME280_DETECTED = true;
          mb.enqueueMessage(F("BME280 sensor started"), F("INFO"));
          JsonDocument jsonBme280 = getBme280DataJson();
          sendTelemetryJson(jsonBme280);
          bme280Temperature = jsonBme280["temperature"];
          bme280Humidity = jsonBme280["humidity"];
        }
      }

      if (DS18B20_DETECTED) {
        sendTelemetryJson(getDs18b20DataJson());
      } else {
        if (checkDs18b20Sensor()) {
          DS18B20_DETECTED = true;
          mb.enqueueMessage(F("DS18B20 sensor started"), F("INFO"));
          sendTelemetryJson(getDs18b20DataJson());
        }
      }

      if (MQ_DETECTED){
        if (isnan(bme280Temperature) || isnan(bme280Humidity)){
          sendTelemetryJson(getMqDataJson());
        } else {
          sendTelemetryJson(getMqDataJson(bme280Temperature, bme280Humidity));
        }
      } else {
        if (checkMqSensor()) {
          MQ_DETECTED = true;
          mb.enqueueMessage(F("MQ135 sensor started"), F("INFO"));
          sendTelemetryJson(getMqDataJson(bme280Temperature, bme280Humidity));
        }
      }
    }
  }
  tb.loop();
}

void reportBh1750Status(const char* currentStatus) {
  static char lastStatus[32] = "";
  if (strcmp(lastStatus, currentStatus) != 0) {
    strncpy(lastStatus, currentStatus, sizeof(lastStatus) - 1);
    tb.sendAttributeData("bh1750Status", currentStatus);
    mb.enqueueMessagef("INFO", "BH1750 status: %s", currentStatus);
  }
}

const char* evaluateBh1750Health(float lux, bool isInitialized) {
  if (!isInitialized) {
    return "ERROR_NOT_FOUND";
  }
  if (isnan(lux) || lux < 0.0) {
    return "ERROR_READING_FAILED";
  }
  return "OK";
}

bool checkBh1750Sensor() {
  Wire.begin();
  bool initSuccess = luxMeter.begin();
  
  if (!initSuccess) {
    reportBh1750Status("ERROR_NOT_FOUND");
    return false;
  }

  float lux = luxMeter.readLightLevel();
  const char* health = evaluateBh1750Health(lux, true);
  reportBh1750Status(health);

  return (strcmp(health, "OK") == 0);
}

void setupBh1750Sensor(){
  Wire.begin();
  BH1750_DETECTED = false;
  mb.enqueueMessage(F("BH1750 sensor started"), F("INFO"));
}

void reportMqStatus(const char* currentStatus) {
  static char lastStatus[32] = "";
  if (strcmp(lastStatus, currentStatus) != 0) {
    strncpy(lastStatus, currentStatus, sizeof(lastStatus) - 1);
    tb.sendAttributeData("mqStatus", currentStatus);
    mb.enqueueMessagef("INFO", "MQ135 status: %s", currentStatus);
  }
}

const char* evaluateMqHealth(float co2Ppm, int rawAdc, float calcR0) {
  if (isinf(calcR0) || isnan(co2Ppm) || isinf(co2Ppm)) {
    return "ERROR_OPEN_CIRCUIT";
  }
  if (calcR0 <= 0.0f || rawAdc <= 10 || co2Ppm <= 0.0) {
    return "ERROR_ZERO_READING";
  }
  if (rawAdc >= 1015 || co2Ppm >= 10000.0) {
    return "ERROR_OUT_OF_RANGE_HIGH";
  }
  return "OK";
}

bool checkMqSensor() {
  float calcR0 = config.mqR0Value;
  if (calcR0 <= 0.0f) {
    calcR0 = mqSensorCalibration();
  }

  MQ135.setR0(calcR0);
  MQ135.update();
  int rawAdc = analogRead(MQ_ANALOG_PIN);
  MQ135.setA(110.47); MQ135.setB(-2.862);
  float co2Ppm = MQ135.readSensor();

  const char* health = evaluateMqHealth(co2Ppm, rawAdc, calcR0);
  reportMqStatus(health);

  bool isHardwarePresent = (strcmp(health, "ERROR_OPEN_CIRCUIT") != 0 &&
                            strcmp(health, "ERROR_ZERO_READING") != 0);

  return isHardwarePresent;
}

void setupMqSensor(){
  if(MQ_DATA_DELETE){
    config.mqR0Value = 0.0f;
    mb.saveStruct(CONFIG_BIN_FILE, config);
  }
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init();
  MQ135.setRL(2); //If the RL value is different from 10K, assign new RL value

  MQ_DETECTED = false;
  mb.enqueueMessage(F("MQ135 sensor started"), F("INFO"));
}

float mqSensorCalibration(){
/*****************************  MQ CAlibration ********************************************/ 
  mb.enqueueMessage(F("MQ135 sensor is being Calibrating, please wait"), F("INFO"));
  float ratioCleanAir = (config.mqCleanAirRatio > 0.0f) ? config.mqCleanAirRatio : 3.6f;
  float calcR0 = 0;
  for(int i = 1; i <= 10; i++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(ratioCleanAir);
  }
  calcR0 = calcR0 / 10.0f;
  
  config.mqR0Value = calcR0;
  mb.saveStruct(CONFIG_BIN_FILE, config);
  /*****************************  MQ CAlibration ********************************************/ 
  return calcR0;
}

void reportBme280Status(const char* currentStatus) {
  static char lastStatus[32] = "";
  if (strcmp(lastStatus, currentStatus) != 0) {
    strncpy(lastStatus, currentStatus, sizeof(lastStatus) - 1);
    tb.sendAttributeData("bme280Status", currentStatus);
    mb.enqueueMessagef("INFO", "BME280 status: %s", currentStatus);
  }
}

const char* evaluateBme280Health(float temp, float hum, float press, bool isInitialized) {
  if (!isInitialized) {
    return "ERROR_NOT_FOUND";
  }
  if (isnan(temp) || isnan(hum) || isnan(press)) {
    return "ERROR_READING_FAILED";
  }
  return "OK";
}

bool checkBme280Sensor() {
  Wire.begin();
  bool initSuccess = bme.begin(0x76);
  
  if (!initSuccess) {
    reportBme280Status("ERROR_NOT_FOUND");
    return false;
  }

  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0F;

  const char* health = evaluateBme280Health(t, h, p, true);
  reportBme280Status(health);

  return (strcmp(health, "OK") == 0);
}

void setupBme280Sensor(){
  Wire.begin();
  BME280_DETECTED = false;
  mb.enqueueMessage(F("BME280 sensor started"), F("INFO"));
}

void reportDs18b20Status(const char* currentStatus) {
  static char lastStatus[32] = "";
  if (strcmp(lastStatus, currentStatus) != 0) {
    strncpy(lastStatus, currentStatus, sizeof(lastStatus) - 1);
    tb.sendAttributeData("ds18b20Status", currentStatus);
    mb.enqueueMessagef("INFO", "DS18B20 status: %s", currentStatus);
  }
}

const char* evaluateDs18b20Health(int deviceCount, int validReadingsCount) {
  if (deviceCount <= 0) {
    return "ERROR_NOT_FOUND";
  }
  if (validReadingsCount <= 0) {
    return "ERROR_READING_FAILED";
  }
  return "OK";
}

bool checkDs18b20Sensor() {
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  
  if (numberOfDevices <= 0) {
    reportDs18b20Status("ERROR_NOT_FOUND");
    return false;
  }

  sensors.requestTemperatures();
  int validCount = 0;

  for (int i = 0; i < numberOfDevices; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      float temp = sensors.getTempC(tempDeviceAddress);
      if (!isnan(temp) && temp > DEVICE_DISCONNECTED_C) {
        validCount++;
      }
    }
  }

  const char* health = evaluateDs18b20Health(numberOfDevices, validCount);
  reportDs18b20Status(health);

  return (strcmp(health, "OK") == 0);
}

void setupDs18b20Sensor() {
  sensors.begin();
  DS18B20_DETECTED = false;
  mb.enqueueMessage(F("DS18B20 sensor started"), F("INFO"));
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
  float lux = luxMeter.readLightLevel();
  const char* healthStatus = evaluateBh1750Health(lux, true);

  if (strcmp(healthStatus, "OK") != 0) {
    mb.enqueueMessagef("ERROR", "Fallo de hardware en BH1750 detectado: %s", healthStatus);
    BH1750_DETECTED = false;
  }
  reportBh1750Status(healthStatus);

  JsonDocument json;
  json["lux"] = lux;
  return json;
}

JsonDocument getMqDataJson(float temperature, float humidity){
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  int rawAdc = analogRead(MQ_ANALOG_PIN);

  float co2Ppm = isnan(temperature) || isnan(humidity) 
    ? (MQ135.setA(110.47), MQ135.setB(-2.862), MQ135.readSensor()) 
    : (MQ135.setA(110.47), MQ135.setB(-2.862), MQ135.readSensor(false, mqCorrectionFactor(temperature, humidity)));

  float calcR0 = MQ135.getR0();
  const char* healthStatus = evaluateMqHealth(co2Ppm, rawAdc, calcR0);

  bool isHardwareFault = (strcmp(healthStatus, "ERROR_OPEN_CIRCUIT") == 0 ||
                          strcmp(healthStatus, "ERROR_ZERO_READING") == 0 ||
                          strcmp(healthStatus, "ERROR_R0_OUT_OF_BOUNDS") == 0);

  if (isHardwareFault) {
    mb.enqueueMessagef("ERROR", "Fallo de hardware en MQ135 detectado: %s", healthStatus);
    MQ_DETECTED = false;
  }
  reportMqStatus(healthStatus);

  JsonDocument json;
  MQ135.setA(605.18); MQ135.setB(-3.937);
  json["co"]      = MQ135.readSensor();
  json["co2"]     = co2Ppm;
  MQ135.setA(77.255); MQ135.setB(-3.18);
  json["alcohol"] = MQ135.readSensor();
  MQ135.setA(44.947); MQ135.setB(-3.445);
  json["toluen"]  = MQ135.readSensor();
  MQ135.setA(102.2 ); MQ135.setB(-2.473);
  json["nh4"]     = MQ135.readSensor();
  MQ135.setA(34.668); MQ135.setB(-3.369);
  json["aceton"]  = MQ135.readSensor();

  return json;
}

/// @brief Calcula el factor de corrección por temperatura y humedad para el sensor MQ-135.
/// @param temp Temperatura actual en °C (obtenida del BME280).
/// @param hum Humedad relativa actual en % (obtenida del BME280).
/// @return Factor multiplicador/aditivo de corrección para RS/R0.
float mqCorrectionFactor(float temp, float hum) {
  const float CORA = 0.00035;
  const float CORB = 0.02718;
  const float CORC = 1.39538;
  const float CORD = 0.0018;
  // Cálculo del factor de corrección ambiental
  float factor = CORA * temp * temp - CORB * temp + CORC - (hum - 33.0) * CORD;
  
  // Protección para evitar factores extremos o negativos
//  if (factor < 0.1) factor = 0.1;
  return factor;
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

JsonDocument getDs18b20DataJson(){
  JsonDocument json;
  sensors.requestTemperatures(); 

  int validCount = 0;

  for(int i = 0; i < numberOfDevices; i++) {
    if(sensors.getAddress(tempDeviceAddress, i)){
      float temp = sensors.getTempC(tempDeviceAddress);
      if (!isnan(temp) && temp > DEVICE_DISCONNECTED_C) {
        json[tempDeviceAddress] = temp;
        validCount++;
      }
    }   
  }

  const char* healthStatus = evaluateDs18b20Health(numberOfDevices, validCount);

  if (strcmp(healthStatus, "OK") != 0) {
    mb.enqueueMessagef("ERROR", "Fallo de hardware en DS18B20 detectado: %s", healthStatus);
    DS18B20_DETECTED = false;
  }
  reportDs18b20Status(healthStatus);

  return json;
}

/************* RPC callbacks *************/
void rpcSubscribe(){
 mb.enqueueMessage(F("Subscribing for RPC"), F("INFO"));

  const std::array<RPC_Callback, 3U> callbacks = {
    RPC_Callback{ RPC_SET_HOURS_OF_LIGHT,     processSetTimeAlarms},
    RPC_Callback{ RPC_TIME_TO_SEND_TELEMETRY, processTimeToSendTelemetry},
    RPC_Callback{ RPC_SET_RELAY,              processSetRelay}
  };

  // Perform a subscription. All consequent data processing will happen in
  // processTemperatureChange() and processSwitchChange() functions,
  // as denoted by callbacks array.
  if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
    mb.enqueueMessage(F("Failed to subscribe for RPC"), F("ERROR"));
    return;
  }
  mb.enqueueMessage(F("Subscribe done"), F("INFO"));
  
  mb.enqueueMessage(F("OTA Firwmare Update Subscription..."), F("INFO"));
  const OTA_Update_Callback callback(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION, &updater, &finished_callback, &progress_callback, &update_starting_callback, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
  updateRequestSent = ota.Subscribe_Firmware_Update(callback);

  subscribed = true;
}

/// @brief Processes function for RPC call "SetTimeAlarms"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
void processSetTimeAlarms(const JsonVariantConst &data, JsonDocument &response) {
  mb.enqueueMessage(F("Received RPC call SetTimeAlarms"), F("RCP"));

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

/// @brief Processes function for RPC call "timeToSendTelemetry"
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
void processTimeToSendTelemetry(const JsonVariantConst &data, JsonDocument &response) {
  mb.enqueueMessage(F("Received timeToSendTelemetry method"), F("RPC"));
  TIME_TO_SEND_TELEMETRY = data["TIME_TO_SEND_TELEMETRY"];

  config.timeToSendTelemetry = TIME_TO_SEND_TELEMETRY;
  mb.saveStruct(CONFIG_BIN_FILE, config);
  tb.sendAttributeData("TimeToSendTelemetry", TIME_TO_SEND_TELEMETRY);

  mb.enqueueMessagef("RPC", "Send telemetry every %lu seconds", TIME_TO_SEND_TELEMETRY);
  response.set(TIME_TO_SEND_TELEMETRY);
}

/// @brief Callback para atender la llamada RPC "setRelay" enviada desde ThingsBoard Rule Engine al cambiar el umbral de CO2.
/// @param data Contiene el parámetro boolean (true/false) enviado por el servidor.
/// @param response Respuesta enviada de vuelta al servidor ThingsBoard.
void processSetRelay(const JsonVariantConst &data, JsonDocument &response) {
  mb.enqueueMessage(F("Received RPC call setRelay"), F("RPC"));

  if (data == "on") {
    fanOn();
  } else {
    fanOff();
  }

  response.set(1);
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
  TIME_SET = true;
}

/************* End RPC callbacks *************/

/************* OTA *************/
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
    mb.enqueueMessage(F("Downloading firmware success"), F("OTA"));

    return;
  }
  mb.enqueueMessage(F("Downloading firmware failed"), F("OTA"));
  Serial.println();
}

/// @brief Progress callback method that will be called every time our current progress of downloading the complete firmware data changed,
/// meaning it will be called if the amount of already downloaded chunks increased.
/// Is meant to allow to display a progress bar or print the current progress of the update into the console with the currently already downloaded amount of chunks and the total amount of chunks
/// @param current Already received and processs amount of chunks
/// @param total Total amount of chunks we need to receive and process until the update has completed
void progress_callback(const size_t & current, const size_t & total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}

/************* End OTA *************/

void initSystemConfig() {
  if (!mb.loadStruct(CONFIG_BIN_FILE, config) || config.magic != 0x4D42494E) {
    mb.enqueueMessage(F("Configuración binaria no encontrada. Inicializando defaults..."), F("WARN"));
    config.magic = 0x4D42494E;
    strncpy(config.server, "materbox.io", sizeof(config.server) - 1);
    strncpy(config.token, "TEST_TOKEN", sizeof(config.token) - 1);
    config.timeToSendTelemetry = 30;
    config.lOnHour = 6;   config.lOnMin = 0;  config.lOnSec = 0;
    config.lOffHour = 18; config.lOffMin = 0; config.lOffSec = 0;
    config.mqR0Value = 0.0f;
    config.mqCleanAirRatio = 3.6f;
    mb.saveStruct(CONFIG_BIN_FILE, config);
  }

  // Cargar parámetros binarios guardados en las variables de conexión
  strncpy(THINGSBOARD_SERVER, config.server, sizeof(THINGSBOARD_SERVER) - 1);
  strncpy(TOKEN, config.token, sizeof(TOKEN) - 1);
  TIME_TO_SEND_TELEMETRY = config.timeToSendTelemetry;
}

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
    mb.deleteFileData(CONFIG_BIN_FILE);
    wm.resetSettings();
    wm.erase();
  }

  WiFiManagerParameter custom_server("server", "MaterBox server", THINGSBOARD_SERVER, 40);
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

  // show password publicly in form
  wm.setShowPassword(true);
  
  if(!WMISBLOCKING){
    wm.setConfigPortalBlocking(false);
  }

  //sets timeout until configuration portal gets turned off
  wm.setConfigPortalTimeout(180);
  wm.setBreakAfterConfig(true); // needed to use saveWifiCallback

  if(DRD_DETECTED || TEST_CP){
    Alarm.delay(1000);
    if(!wm.startConfigPortal("MaterBox IoT", "123456789")){
      mb.enqueueMessage(F("Failed to connect and hit timeout"), F("INFO"));
    } else {
      mb.enqueueMessage(F("Wifi connected :)"), F("INFO"));
      wifiInfo();
    }
  } else {
    if(!wm.autoConnect("MaterBox IoT", "123456789")){
      mb.enqueueMessage(F("Failed to connect and hit timeout"), F("INFO"));
    } else {
      mb.enqueueMessage(F("Wifi connected :)"), F("INFO"));
      wifiInfo();
    }
  }

  //read updated parameters
  strcpy(THINGSBOARD_SERVER, custom_server.getValue());
  strcpy(TOKEN, custom_api_token.getValue());

  if (SAVE_PARAMS){
    strncpy(config.server, THINGSBOARD_SERVER, sizeof(config.server) - 1);
    strncpy(config.token, TOKEN, sizeof(config.token) - 1);
    config.timeToSendTelemetry = TIME_TO_SEND_TELEMETRY;
    mb.saveStruct(CONFIG_BIN_FILE, config);
  }
}

void saveWifiCallback(){
  mb.enqueueMessage(F("wm save settings Callback fired "), F("INFO"));
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  mb.enqueueMessage(F("wm config Mode Callback fired"), F("INFO"));
}

void saveParamCallback(){
  mb.enqueueMessage(F("wm save Parameters Callback fired"), F("INFO"));
  SAVE_PARAMS = true;
}

void bindServerCallback(){
  wm.server->on("/custom",handleRoute); // this is now crashing esp32 for some reason
  // wm.server->on("/info",handleRoute); // you can override wm!
}

void handleRoute(){
  wm.server->send(200, "text/plain", "hello from user code");
  mb.enqueueMessage(F("wm handle route"), F("INFO"));
}

void wifiInfo(){
  // can contain gargbage on esp32 if wifi is not ready yet
  mb.enqueueMessage(F("Wifi debug data"), F("INFO"));

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
  mb.enqueueMessage(F("Request time from server"), F("RPC"));
  
  RPC_Request_Callback callback(RPC_REQUEST_GET_CURRENT_TIME, &processTime, nullptr, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut);

  // Perform a request of the given RPC method. Optional responses are handled in processTime
  if (!rpc_request.RPC_Request(callback)) {
    mb.enqueueMessage(F("Failed to request time from server"), F("ERROR"));
  } else {
    mb.enqueueMessage(F("Request done"), F("RPC"));
    SET_TIME = false;
  }
}

/// @brief Attribute request did not receive a response in the expected amount of microseconds 
void requestTimedOut() {
  Serial.printf("RPC request timed out did not receive a response in (%llu) microseconds. Ensure client is connected to the MQTT broker and that the RPC method actually exist on the device Rule chain\n", REQUEST_TIMEOUT_MICROSECONDS);
}

void printActualTime(){
  mb.enqueueMessagef("INFO", "Time: %02d:%02d:%02d", hour(), minute(), second());
}

void setTimeAlarms(int lOnHour, int lOnMin, int lOnSec, int lOffHour, int lOffMin, int lOffSec){
  if (ALARMS_ARE_SET){  
    Alarm.free(ALARM_ID_ON);
    Alarm.free(ALARM_ID_OFF);
  }

  if (lOnHour == 30){
    lOnHour  = config.lOnHour;
    lOnMin   = config.lOnMin;
    lOnSec   = config.lOnSec;
    lOffHour = config.lOffHour;
    lOffMin  = config.lOffMin;
    lOffSec  = config.lOffSec;
    mb.enqueueMessage(F("Setting alarms from system config"), F("INFO"));
  } else {
    mb.enqueueMessage(F("Setting alarms from RPC call"), F("INFO"));
    config.lOnHour  = lOnHour;
    config.lOnMin   = lOnMin;
    config.lOnSec   = lOnSec;
    config.lOffHour = lOffHour;
    config.lOffMin  = lOffMin;
    config.lOffSec  = lOffSec;
    mb.saveStruct(CONFIG_BIN_FILE, config);
  }
  ALARM_ID_ON = Alarm.alarmRepeat(lOnHour, lOnMin, lOnSec, turnLightsOn);
  mb.enqueueMessagef("INFO", "Encender: %02d:%02d:%02d", lOnHour, lOnMin, lOnSec);

  ALARM_ID_OFF = Alarm.alarmRepeat(lOffHour, lOffMin, lOffSec, turnLightsOff);
  mb.enqueueMessagef("INFO", "Apagar: %02d:%02d:%02d", lOffHour, lOffMin, lOffSec);

//  Alarm.timerRepeat(15, Repeats);           // timer for every 15 seconds
  if(ALARM_ID_ON == 255 || ALARM_ID_OFF == 255){
    mb.enqueueMessage(F("Alarms not set. Try again"), F("WARN"));
    Alarm.free(ALARM_ID_ON);
    Alarm.free(ALARM_ID_OFF);
    ALARMS_ARE_SET = false;
    SET_ALARMS = true;
  } else {
    mb.enqueueMessagef("INFO", "Alarm ON set. Id: %d", ALARM_ID_ON);
    mb.enqueueMessagef("INFO", "Alarm OFF set. Id: %d", ALARM_ID_OFF);
    ALARMS_ARE_SET = true;
    SET_ALARMS = false;
  }
}

void turnLightsOn(){
  mb.enqueueMessage(F("Triggered turn lights on alarm"), F("INFO"));
  digitalWrite(Relay3, LOW);
  //tb.sendTelemetryData("lights", 1);
}

void turnLightsOff(){
  mb.enqueueMessage(F("Triggered turn lights off alarm"), F("INFO"));
  digitalWrite(Relay3, HIGH);
  //tb.sendTelemetryData("lights", 0);
}

void fanOn(){
  mb.enqueueMessage(F("Relay CO2 (Relay4): ENCENDIDO (ON)"), F("INFO"));
  digitalWrite(Relay4, LOW);
  tb.sendAttributeData("fanState", "on");
}

void fanOff(){
  mb.enqueueMessage(F("Relay CO2 (Relay4): APAGADO (OFF)"), F("INFO"));
  digitalWrite(Relay4, HIGH);
  tb.sendAttributeData("fanState", "off");
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