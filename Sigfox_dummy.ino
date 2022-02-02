#include <Wire.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include "conversions.h"

#define PMS_PIN_RX D1
#define PMS_PIN_TX D2

#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_MIN_INFO 3
#define DEBUG_MED_INFO 4
#define DEBUG_MAX_INFO 5

bool is_PMS_running = true;
bool send_now = true;
int pms_pm10_sum = 0;
int pms_pm25_sum = 0;
int pms_val_count = 0;
int pms_pm10_max = 0;
int pms_pm10_min = 20000;
int pms_pm25_max = 0;
int pms_pm25_min = 20000;
int  debug = 3;

float last_value_PMS_P1 = 0;
float last_value_PMS_P2 = 0;

const unsigned long READINGTIME_PMS_MS = 5000;

String esp_chipid;

#define SIGFOX_FRAME_LENGTH 12
#define INTERVAL 145000 //600000, 
#define DEBUG 1

#define RxNodePin D7
#define TxNodePin D8

#define DHT_TYPE DHT22
#define DHT_PIN D5

#define PMS_API_PIN 1
// Serial confusion: These definitions are based on SoftSerial
// TX (transmitting) pin on one side goes to RX (receiving) pin on other side
// SoftSerial RX PIN is D1 and goes to PMS TX
// SoftSerial TX PIN is D2 and goes to PMS RX
#define PMS_PIN_RX D1
#define PMS_PIN_TX D2

// Setup UART Communication with 
SoftwareSerial SigFox =  SoftwareSerial(RxNodePin, TxNodePin);

SoftwareSerial serialPMS(PMS_PIN_RX, PMS_PIN_TX, false, 128);

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
  serialPMS.begin(9600);
  
  
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
 
  frame.humidity = convertoFloatToUInt16(h, 100);
  frame.temperature = convertoFloatToUInt16(t, 100);
  
  
  fetchSensorPMS();
  
  frame.p1 = convertoFloatToUInt16(last_value_PMS_P1, 2000);
  frame.p2 = convertoFloatToUInt16(last_value_PMS_P2, 2000);

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
/* start PMS5003 sensor                                          *
/*****************************************************************/
void start_PMS() {
  const uint8_t start_PMS_cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
  serialPMS.write(start_PMS_cmd, sizeof(start_PMS_cmd)); is_PMS_running = true;
}

/*****************************************************************
/* stop PMS5003 sensor                                           *
/*****************************************************************/
void stop_PMS() {
  const uint8_t stop_PMS_cmd[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
  serialPMS.write(stop_PMS_cmd, sizeof(stop_PMS_cmd)); is_PMS_running = false;
}

/*****************************************************************
 * read Plantronic PM sensor sensor values                       *
 *****************************************************************/
static void fetchSensorPMS(String &s)
{
  char buffer;
  int value;
  int len = 0;
  int pm10_serial = 0;
  int pm25_serial = 0;
  int checksum_is = 0;
  int checksum_should = 0;
  bool checksum_ok = false;
  int frame_len = 24; // min. frame length

  debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_PMSx003));
    if (!is_PMS_running)
    {
      start_PMS();
    }
    delay(3000);

    while (serialPMS.available() > 0)
    {
      buffer = serialPMS.read();
      debug_outln(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO);
      //			"aa" = 170, "ab" = 171, "c0" = 192
      value = int(buffer);
      switch (len)
      {
      case 0:
        if (value != 66)
        {
          len = -1;
        };
        break;
      case 1:
        if (value != 77)
        {
          len = -1;
        };
        break;
      case 2:
        checksum_is = value;
        break;
      case 3:
        frame_len = value + 4;
        break;
      case 10:
        pm1_serial += (value << 8);
        break;
      case 11:
        pm1_serial += value;
        break;
      case 12:
        pm25_serial = (value << 8);
        break;
      case 13:
        pm25_serial += value;
        break;
      case 14:
        pm10_serial = (value << 8);
        break;
      case 15:
        pm10_serial += value;
        break;
      case 22:
        if (frame_len == 24)
        {
          checksum_should = (value << 8);
        };
        break;
      case 23:
        if (frame_len == 24)
        {
          checksum_should += value;
        };
        break;
      case 30:
        checksum_should = (value << 8);
        break;
      case 31:
        checksum_should += value;
        break;
      }
      if ((len > 2) && (len < (frame_len - 2)))
      {
        checksum_is += value;
      }
      len++;
      if (len == frame_len)
      {
        debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_IS), String(checksum_is + 143));
        debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_SHOULD), String(checksum_should));
        if (checksum_should == (checksum_is + 143))
        {
          checksum_ok = true;
        }
        else
        {
          len = 0;
        };
        if (checksum_ok && (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_PMS_MS)))
        {
          if ((!isnan(pm1_serial)) && (!isnan(pm10_serial)) && (!isnan(pm25_serial)))
          {
            pms_pm10_sum += pm10_serial;
            pms_pm25_sum += pm25_serial;
            if (pms_pm25_min > pm25_serial)
            {
              pms_pm25_min = pm25_serial;
            }
            if (pms_pm25_max < pm25_serial)
            {
              pms_pm25_max = pm25_serial;
            }
            if (pms_pm10_min > pm10_serial)
            {
              pms_pm10_min = pm10_serial;
            }
            if (pms_pm10_max < pm10_serial)
            {
              pms_pm10_max = pm10_serial;
            }
            debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial));
            debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial));
            pms_val_count++;
          }
          len = 0;
          checksum_ok = false;
          pm10_serial = 0;
          pm25_serial = 0;
          checksum_is = 0;
        }
      }
      yield();
    }
  }
  if (send_now)
  {
    last_value_PMS_P1 = -1;
    last_value_PMS_P2 = -1;
    if (pms_val_count > 2)
    {
      pms_pm10_sum = pms_pm10_sum - pms_pm10_min - pms_pm10_max;
      pms_pm25_sum = pms_pm25_sum - pms_pm25_min - pms_pm25_max;
      pms_val_count = pms_val_count - 2;
    }
    if (pms_val_count > 0)
    {
      last_value_PMS_P1 = float(pms_pm10_sum) / float(pms_val_count);
      last_value_PMS_P2 = float(pms_pm25_sum) / float(pms_val_count);
      add_Value2Json(s, F("PMS_P1"), F("PM10:  "), last_value_PMS_P1);
      add_Value2Json(s, F("PMS_P2"), F("PM2.5: "), last_value_PMS_P2);
      debug_outln_info(FPSTR(DBG_TXT_SEP));
    }
    pms_pm10_sum = 0;
    pms_pm25_sum = 0;
    pms_val_count = 0;
    pms_pm10_max = 0;
    pms_pm10_min = 20000;
    pms_pm25_max = 0;
    pms_pm25_min = 20000;
    
    stop_PMS();
  }

  debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_PMSx003));
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
