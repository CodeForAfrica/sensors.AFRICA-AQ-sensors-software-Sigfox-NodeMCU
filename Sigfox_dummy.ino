#include <Wire.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include "conversions.h"

#define SDS_PIN_RX D1
#define SDS_PIN_TX D2

#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_MIN_INFO 3
#define DEBUG_MED_INFO 4
#define DEBUG_MAX_INFO 5

bool is_SDS_running = true;
bool send_now = true;
int sds_pm10_sum = 0;
int sds_pm25_sum = 0;
int sds_val_count = 0;
int sds_pm10_max = 0;
int sds_pm10_min = 20000;
int sds_pm25_max = 0;
int sds_pm25_min = 20000;
int  debug = 3;

float last_value_SDS_P1 = 0;
float last_value_SDS_P2 = 0;

String esp_chipid;

#define SIGFOX_FRAME_LENGTH 12
#define INTERVAL 145000 //600000, 
#define DEBUG 1

#define RxNodePin D7
#define TxNodePin D8

#define DHT_TYPE DHT22
#define DHT_PIN D5

#define SDS_API_PIN 1
// Serial confusion: These definitions are based on SoftSerial
// TX (transmitting) pin on one side goes to RX (receiving) pin on other side
// SoftSerial RX PIN is D1 and goes to SDS TX
// SoftSerial TX PIN is D2 and goes to SDS RX
#define SDS_PIN_RX D1
#define SDS_PIN_TX D2

// Setup UART Communication with 
SoftwareSerial SigFox =  SoftwareSerial(RxNodePin, TxNodePin);

SoftwareSerial serialSDS(SDS_PIN_RX, SDS_PIN_TX, false, 128);

DHT dht(DHT_PIN, DHT_TYPE);

unsigned long previousSendTime = 0;

typedef struct __attribute__ ((packed)) data {
  uint16_t humidity;
  uint16_t temperature;
  uint16_t p1;
  uint16_t p2;
};


void setup() {
  Serial.begin(115200);
  WiFi.softAPdisconnect(true);
  //smeHumidity.begin();
  //smePressure.begin();
  
  esp_chipid = String(ESP.getChipId());
  
  delay(200);
  
  dht.begin();
  
  pinMode(RxNodePin, INPUT);
  pinMode(TxNodePin, OUTPUT);
  SigFox.begin(9600);
  delay(100);
  Serial.println("\n***** START *****");
  //initSigfox();
  serialSDS.begin(9600);
  
  
}

void loop() {
  data frame;
  float h = dht.readHumidity(); //Read Humidity
  float t = dht.readTemperature(); //Read Temperature
  if (isnan(t) || isnan(h)) {
      delay(100);
      h = dht.readHumidity(true); //Read Humidity
      t = dht.readTemperature(false, true); //Read Temperature
    }
 
  frame.humidity = convertoFloatToUInt8(h, 100);
  frame.temperature = convertoFloatToUInt8(t, 100);
  
  
  sensorSDS();
  
  frame.p1 = convertoFloatToUInt16(last_value_SDS_P1, 2000);
  frame.p2 = convertoFloatToUInt16(last_value_SDS_P2, 2000);

   if (DEBUG) {
    Serial.print("Temp \t");
    Serial.print(t);
    Serial.print("\t");
    Serial.println(frame.temperature);
    Serial.print("Humidity \t");
    Serial.print(h);
    Serial.print("\t");
    Serial.println(frame.humidity);
    Serial.print("PM 10 ");
    Serial.println(frame.p1);
    Serial.print("PM 2.5 ");
    Serial.println(frame.p2);
    
  }
  
  Serial.print("Size of payload\t");
  Serial.print(sizeof(frame));
  Serial.println(" ");
 
  bool answer = sendSigfox(&frame, sizeof(data));
  

  delay(INTERVAL);
}


void initSigfox(){
  //SigFox.print("+++");
  while (!SigFox.available()){
    Serial.println("Sigfox not available, waiting to connect.....");
    delay(100);
  }
  while (SigFox.available()){
    byte serialByte = SigFox.read();
    if (DEBUG){
      Serial.print(serialByte);
    }
  }
  if (DEBUG){
    Serial.println("\n ** Setup OK **");
  }
}

String getSigfoxFrame(const void* data, uint8_t len){
  String frame = "";
  uint8_t* bytes = (uint8_t*)data;

  
  if (len < SIGFOX_FRAME_LENGTH){
    //fill with zeros
    uint8_t i = SIGFOX_FRAME_LENGTH;
    while (i-- > len){
      frame += "00";
    }
  }

  //0-1 == 255 --> (0-1) > len
  for(uint8_t i = len-1; i < len; --i) {
    if (bytes[i] < 16) {frame+="0";}
    frame += String(bytes[i], HEX);
  }
  
  return frame;
}


bool sendSigfox(const void* data, uint8_t len){
  String frame = getSigfoxFrame(data, len);
  String status = "";
  char output;
  if (DEBUG){
    Serial.print("AT$SF=");
    Serial.println(frame);
  }
  SigFox.print("AT$SF=");
  SigFox.print(frame);
  SigFox.print("\r");
  while (!SigFox.available()){
    delay(1000);
    Serial.println("Sigfox not avaibale! Please wait as we try to connect to it....");
  }
  
  while(SigFox.available()){
    output = (char)SigFox.read();
    status += output;
    delay(10);
  }
  if (DEBUG){
    Serial.print("Status \t");
    Serial.println(status);
  }
  if (status == "OK\r"){
    //Success :)
    return true;
  }
  else{
    return false;
  }
}


/*****************************************************************
/* start SDS011 sensor                                           *
/*****************************************************************/
void start_SDS() {
  const uint8_t start_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
  serialSDS.write(start_SDS_cmd, sizeof(start_SDS_cmd)); is_SDS_running = true;
}

/*****************************************************************
/* stop SDS011 sensor                                            *
/*****************************************************************/
void stop_SDS() {
  const uint8_t stop_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
  serialSDS.write(stop_SDS_cmd, sizeof(stop_SDS_cmd)); is_SDS_running = false;
}



/*****************************************************************
/* read SDS011 sensor values                                     *
/*****************************************************************/
String sensorSDS() {
  String s = "";
  String value_hex;
  char buffer;
  int value;
  int len = 0;
  int pm10_serial = 0;
  int pm25_serial = 0;
  int checksum_is;
  int checksum_ok = 0;
  int position = 0;

  Serial.println("Start reading SDS011");
    delay(3000);
    if (!is_SDS_running) {
      start_SDS();
    } 
    delay(3000);

    while (serialSDS.available() > 0) {
      Serial.println("Inside");
      buffer = serialSDS.read();
      debug_out(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO, 1);
//      "aa" = 170, "ab" = 171, "c0" = 192
      value = int(buffer);
      switch (len) {
      case (0): if (value != 170) { len = -1; }; break;
      case (1): if (value != 192) { len = -1; }; break;
      case (2): pm25_serial = value; checksum_is = value; break;
      case (3): pm25_serial += (value << 8); checksum_is += value; break;
      case (4): pm10_serial = value; checksum_is += value; break;
      case (5): pm10_serial += (value << 8); checksum_is += value; break;
      case (6): checksum_is += value; break;
      case (7): checksum_is += value; break;
      case (8):
        debug_out(F("Checksum is: "), DEBUG_MED_INFO, 0); debug_out(String(checksum_is % 256), DEBUG_MED_INFO, 0);
        debug_out(F(" - should: "), DEBUG_MED_INFO, 0); debug_out(String(value), DEBUG_MED_INFO, 1);
        if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
      case (9): if (value != 171) { len = -1; }; break;
      }
      len++;
      if (len == 10 && checksum_ok == 1) {
        if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
          sds_pm10_sum += pm10_serial;
          sds_pm25_sum += pm25_serial;
          if (sds_pm10_min > pm10_serial) { sds_pm10_min = pm10_serial; }
          if (sds_pm10_max < pm10_serial) { sds_pm10_max = pm10_serial; }
          if (sds_pm25_min > pm25_serial) { sds_pm25_min = pm25_serial; }
          if (sds_pm25_max < pm25_serial) { sds_pm25_max = pm25_serial; }
          sds_val_count++;
        }
        len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
      }
      yield();
    }
  if (send_now) {
    last_value_SDS_P1 = 0;
    last_value_SDS_P2 = 0;
    if (sds_val_count > 2) {
      sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
      sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
      sds_val_count = sds_val_count - 2;
    }
    if (sds_val_count > 0) {
      
      last_value_SDS_P1 = float(sds_pm10_sum) / (sds_val_count * 10.0);
      last_value_SDS_P2 = float(sds_pm25_sum) / (sds_val_count * 10.0);
      Serial.print("SDS Values \t");
      Serial.print(last_value_SDS_P1);
      Serial.print("\t");
      Serial.println(last_value_SDS_P2);
      
    }
    sds_pm10_sum = 0; sds_pm25_sum = 0; sds_val_count = 0;
    sds_pm10_max = 0; sds_pm10_min = 20000; sds_pm25_max = 0; sds_pm25_min = 20000;
     
    stop_SDS();
  }

  debug_out(F("End reading SDS011"), DEBUG_MED_INFO, 1);

  return s;
}



/*****************************************************************
/* Debug output                                                  *
/*****************************************************************/
void debug_out(const String& text, const int level, const bool linebreak) {
  if (level <= debug) {
    if (linebreak) {
      Serial.println(text);
    } else {
      Serial.print(text);
    }
  }
}
