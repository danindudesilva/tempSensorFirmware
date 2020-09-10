/**********************************************
   _     _                                 _   
  (_)   | |                               | |  
   _  __| | ___  __ _ _ __ ___   __ _ _ __| |_ 
  | |/ _\ |/ _ \/ _\ | |_ \ _ \ / _\ | |__| __|
  | | (_| |  __/ (_| | | | | | | (_| | |  | |_ 
  |_|\__/_|\___|\__/_|_| |_| |_|\__/_|_|   \__|
  --------------------------------------------
  COLD ROOM TEMPERATURE MONITORING DEVICE v1.0
  --------------------------------------------
                  
 * @project    COLD ROOM TEMPERATURE MONITORING
 * @version    3.1
 * @file       main.cpp
 * @author     Danindu de Silva
 * @group      Ideamart
 * @copyright  Dialog Axiata PLC
 * @date       September 2020
 * @mcu        Atmega328p
 * @com_module SIM7000
 * @framework  Arduino

***********************************************/

#define TINY_GSM_MODEM_SIM7000
#define FIRMWARE_VER "TempSensor_V3.0"

//Sensor Pins
#define DHTPIN1 2
#define DHTPIN2 3
#define MQTT_STATUS  SDA

// ideaBoard pin configuration
#define ideaBoard_PWRKEY 13
#define ideaBoard_RX      8
#define ideaBoard_TX      7
#define ideaBoard_RST    11

#define EVENT_TOPIC     "ideamart/sensor/v2/common"
#define MQTT_USER       "ideamart-sensor-v2_4939"
#define MQTT_PASS       "1598981175_4939"
#define MQTT_BROKER     "mqtt.iot.ideamart.io"
#define MQTT_PORT       1883
#define GSM_APN         "dialogbb"
#define NBIOT_APN       "nbiot"
#define EEPROM_ADDR     0
#define EEPROM_SIZE     1
#define SMS_PORT        "87981"
#define SMS_NUMBER      "+94777333295"

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "DHT.h"

SoftwareSerial SerialAT(7,8); // RX, TX
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

String ID;
String clientId;
int  rstCnt, battlevel;

int8_t  apnNo, siglevel;
int8_t  modem_retries = 0;
boolean  STATUS_CODE = 0;
int8_t  minSig = 31;
int8_t  maxSig = 0;
int  minBatt = 4500;
int  maxBatt = 0;

float h1 = 0.0;
float h2 = 0.0;
float t1 = 0.0;
float t2 = 0.0;

float tx1 = 0.0;
float hx1 = 0.0;
float tx2 = 0.0;
float hx2 = 0.0;

int8_t  t1_count = 0;
int8_t  h1_count = 0;
int8_t  t2_count = 0;
int8_t  h2_count = 0;
    
#define PUB_INTERVAL 60000 //interval to publish data
unsigned long PREV_PUBLISH = 0;
unsigned long CUR_PUBLISH = 0;
int8_t  NO_READ_THRES = 90;       //start reading data when only this percentage of PUB_INTERVAL is elapsed
int8_t  READ_COUNT = 10;
unsigned long PREV_READ = 0;
unsigned long CUR_READ = 0;
unsigned long PUBLISH_COUNT = 0;
#define SMS_INTERVAL 86400000
unsigned long CUR_SMS = 0;
unsigned long PREV_SMS = 0;

#define MIN_BATTERY_THRESHOLD 3770
#define MIN_SIGNAL_THRESHOLD 6
boolean BATTERY_LOW = false;
boolean SIGNAL_LOW = false;
boolean MODEM_OFF = false;
#define RECHARGE_INTERVAL 60000  //interval for modem power down unti battery charge in milliseconds
#define MAX_BATTERY_THRESHOLD 3900
#define GPRS_ATTEMPTS 10
#define MQTT_ATTEMPTS 10
#define PUBLISH_ATTEMPTS 10
unsigned long RECHARGE_CUR = 0;
unsigned long RECHARGE_PREV = 0;

DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

void setup() {
  //Initializing all variables
  modem_retries = 0;
  
  //Set all output pins pins to LOW
  pinMode(MQTT_STATUS, OUTPUT);
  digitalWrite(MQTT_STATUS, LOW);
  delay(100);

  pinMode(ideaBoard_RST, OUTPUT);
  digitalWrite(ideaBoard_RST, LOW);
  delay(100);
  
  //Power up modem
  pinMode(ideaBoard_PWRKEY, OUTPUT);

  modemPowerUp();
  
  //Initialize Serial ports
  Serial.begin(115200);
  SerialAT.begin(4800);
  delay(100);


  Serial.println(F("   _     _                                 _   "));
  Serial.println(F("  (_)   | |                               | |  "));
  Serial.println(F("   _  __| | ___  __ _ _ __ ___   __ _ _ __| |_ "));
  Serial.println(F("  | |/ _\\ |/ _ \\/ _\\ | |_ \\ _ \\ / _\\ | |__| __|"));
  Serial.println(F("  | | (_| |  __/ (_| | | | | | | (_| | |  | |_ "));
  Serial.println(F("  |_|\\__/_|\\___|\\__/_|_| |_| |_|\\__/_|_|   \\__|"));
  Serial.println(F("  --------------------------------------------"));
  Serial.println(F("  COLD ROOM TEMPERATURE MONITORING DEVICE v1.0"));
  Serial.println(F("  --------------------------------------------"));
  Serial.println(F(" "));
  Serial.flush();
  
  //Flash session id
  rstCnt = EEPROM.read(EEPROM_ADDR);
  rstCnt= rstCnt + 1; 
  EEPROM.write(EEPROM_ADDR, rstCnt);
  //Serial.print(F("RESET COUNT: ")); Serial.println(EEPROM.read(EEPROM_ADDR));

  //Flash session id
  apnNo = EEPROM.read(EEPROM_ADDR+1);
  
  //Getting modem IMEI no.
  Serial.print(F("GETTING MODEM IMEI: "));
  
  boolean imei_success = getModemIMEI();
  if(imei_success){
    modem_retries = 0;
    Serial.println(F("READ IMEI SUCCESS"));
  }else {
    while(!imei_success){
      if(modem_retries == 10){
        Serial.println(F("CAN'T READ IMEI. CHECK MODEM"));
        STATUS_CODE = 1;
        break;
      }
      Serial.print(F("GETTING MODEM IMEI: "));
      imei_success = getModemIMEI();
      delay(100);
      modem_retries += 1;
    }
  }

  // MQTT Broker setup
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  //mqtt.setCallback(cb);
  Serial.println(F("INITIALIZING SENSOR 1"));
  dht1.begin();
  
  Serial.println(F("INITIALIZING SENSOR 2"));
  dht2.begin();
  Serial.println(F("WAITING TO READ DATA...")); 
  alertSMS("DEVICE POWER UP");
}

void loop() {
  //Serial.println(F("IN LOOP"));
    //CHECK BATTERY LOW
  if(BATTERY_LOW){
    //do battery low action
    Serial.println(F("%%%%%%%%%%%%%%%%%%%%%%%% BATTERY LOW %%%%%%%%%%%%%%%%%%%%%%%%%"));
    if(MODEM_OFF){
//      Serial.println(F("MODEM OFF"));
      modemPowerUp();
      pauseFor(5000);
      
      int x = modem.getBattVoltage();
      if(x != 0) battlevel = x;
      
      if(battlevel < MAX_BATTERY_THRESHOLD){     //if the battery level has not increased more than 1.1 times the threshold
        //POWER OFF MODEM AGAIN
        Serial.println(battlevel);
        modemPowerDown();

        //PAUSING CODE HERE UNTIL HALF OF RECHARGE INTERVAL IS PASSED
        pauseFor(RECHARGE_INTERVAL/2);
      
      }else { //if the battery has charged        
        modemPowerUp();
        pauseFor(1000);
        boolean sentAlert = alertSMS("Device " + ID + "\nPOWERED UP AND CHARGING");
        if(!sentAlert){
          while(!sentAlert){
            sentAlert = alertSMS("Device " + ID + "\nPOWERED UP AND CHARGING");
          }
        }
//        Serial.println(F("MODEM ON"));
        MODEM_OFF = false;
        BATTERY_LOW = false;
      }
    }else{
      String message = "Device " + ID + "\nBATTERY LOW, NOT CHARGING!\nDEVICE POWERING DOWN. Device will power up when charged.";
      alertSMS(message);
      //pauseFor(100);
      dailyUpdate();
      Serial.println(F("MODEM POWERING DOWN"));
      modemPowerDown();
      MODEM_OFF = true;
      pauseFor(RECHARGE_INTERVAL);
    }
    

  }else{ //IF BATTERY OK
    
    CUR_PUBLISH = millis();
    if ((unsigned long)(CUR_PUBLISH - PREV_PUBLISH) >= PUB_INTERVAL){
      Serial.println(F("##############################################################"));
      Serial.println(F("GETTING READY TO PUBLISH DATA"));
      
      averageReadings();
      
      //modemPowerUp();
      
      if(apnNo == 1){
        boolean gprs_connected = 0;
        for(int8_t i=1; i<=GPRS_ATTEMPTS; i++){
          Serial.println(F("NETWORK: NBIOT"));
          Serial.print(F("CONNECTING TO NETWORK: ATTEMPT ")); Serial.print(i);
          gprs_connected = modem.gprsConnect("nbiot", "", "");
          if(gprs_connected){
            Serial.println(F(": SUCCESS"));
            Serial.print(F("Signal level: ")); Serial.println(modem.getSignalQuality());
            break;
          } else{
            Serial.println(F(": FAILED"));
            if(i == GPRS_ATTEMPTS){
              //Self Reset
              
            }
          }
        }
      }else {
        boolean gprs_connected = 0;
        for(int8_t i=1; i<=GPRS_ATTEMPTS; i++){
          Serial.println(F("NETWORK: GPRS"));
          Serial.print(F("CONNECTING TO NETWORK: ATTEMPT ")); Serial.print(i);
          gprs_connected = modem.gprsConnect("dialogbb", "", "");
          if(gprs_connected){
            Serial.println(F(": SUCCESS"));
            Serial.print(F("Signal level: ")); Serial.println(modem.getSignalQuality());
            break;
          } else{
            Serial.println(F(": FAILED"));
            if(i == GPRS_ATTEMPTS){
              //Self Reset
              
            }
          }
        }   
      }
  
      //CONNECT TO MQTT SERVER
      boolean mqtt_connected = 0;
      for(int8_t i=1; i<=MQTT_ATTEMPTS; i++){
        Serial.print(F("CONNECTING TO MQTT BROKER: ATTEMPT ")); Serial.print(i);
        mqtt_connected = mCon();
        if(mqtt_connected){
            Serial.println(F(": SUCCESS"));
            Serial.print(F("Client ID :  ")); Serial.println(clientId);
            digitalWrite(MQTT_STATUS, HIGH);
            break;
          } else{
            Serial.println(F(": FAILED"));
            if(i == MQTT_ATTEMPTS){
              //Self Reset
              Serial.println(F("CAN'T CONNECT TO MQTT BROKER"));
              STATUS_CODE = 1;
              mqttFail();
            }
          }
      }
      
      //PUBLISH DATA
      boolean published = 0;
      for(int8_t i=1; i<PUBLISH_ATTEMPTS; i++){
        Serial.print(F("DATA PUBLISH ATTEMPT ")); Serial.println(i);
        published = publishData();
        if(published){
          PUBLISH_COUNT +=1;
          Serial.println(F("DATA PUBLISH SUCCESSFUL"));
          Serial.println(F("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"));
          Serial.print(F("PUBLISH COUNT: ")); Serial.println(PUBLISH_COUNT);
          mqtt.disconnect();
          Serial.println(F("DISCONNED FROM MQTT BROKER"));
          modem.gprsDisconnect();
          Serial.println(F("TCP CONNECTION SHUT"));
          //modemPowerDown();
          //Serial.println(F("MODEM POWERED DOWN"));
          PREV_PUBLISH = millis();
          break;
        }else {
          Serial.println(F("DATA PUBLISH FAILED"));
          if(i == PUBLISH_ATTEMPTS){
            //Self Reset
            Serial.println(F("CAN'T PUBLISH DATA"));
            //modemPowerDown();
            //Serial.println(F("MODEM POWERED DOWN"));
            PREV_PUBLISH = millis();
            STATUS_CODE = 400;
          }
        }
      }
      
    }else {
  //    Serial.print("NO_READ_THRES: "); Serial.println(NO_READ_THRES);
  //    Serial.print("PUB_INTERVAL: "); Serial.println(PUB_INTERVAL);
  
      unsigned long READING_PART_OF_PUBLISH_INTERVAL = PUB_INTERVAL*NO_READ_THRES/100;
  //    Serial.print("READING_PART_OF_PUBLISH_INTERVAL: "); Serial.println(READING_PART_OF_PUBLISH_INTERVAL);
      if(((unsigned long)(CUR_PUBLISH - PREV_PUBLISH) >= READING_PART_OF_PUBLISH_INTERVAL)){
    //    Serial.println(F("============================================================="));
    //    Serial.println(F("STARTING SENSOR DATA READING"));
        
        CUR_READ = millis();
    //    Serial.print("CUR_READ: "); Serial.println(CUR_READ);
    //    Serial.print("PREV_READ: "); Serial.println(PREV_READ);
        unsigned long DATA_READ_INTERVAL = (PUB_INTERVAL-READING_PART_OF_PUBLISH_INTERVAL)/READ_COUNT;
    //    Serial.print("DATA_READ_INTERVAL: "); Serial.println(DATA_READ_INTERVAL);
        if((unsigned long)(CUR_READ - PREV_READ) >= DATA_READ_INTERVAL){
          readDHT();      
          PREV_READ = CUR_READ;
        }
      }
    }
  //  Serial.print("millis(): "); Serial.println(millis());
  //  Serial.print("STATUS CODE: "); Serial.println(STATUS_CODE);
    CUR_SMS = millis();
    if((unsigned long)(CUR_SMS - PREV_SMS) >= SMS_INTERVAL){
      boolean DU = dailyUpdate();
      if(DU) Serial.println(F("DAILY UPDATE SENT"));
      PREV_SMS = CUR_SMS;
    }
  }  
}

//This function is used to block the code for a specified amount of time (in milliseconds) without using delay()
void pauseFor(unsigned long interval){
  unsigned long prev = millis();
  unsigned long cur = millis();
  
  while((cur - prev) <= interval){
    cur = millis();
  }
}

void modemPowerUp(){
  digitalWrite(ideaBoard_PWRKEY, HIGH);
  //pausFor(1000);
}

void modemPowerDown(){
  digitalWrite(ideaBoard_PWRKEY, LOW);
  //pausFor(200);
}

void averageReadings(){
  
  Serial.println(F("Averaging sensor readings"));
  if(t1_count > 0){
    t1 = tx1 / t1_count;
  }else {
    t1 = -1000.0;
  }
  Serial.print(F("t1: ")); Serial.println(t1);
  
  if(h1_count > 0){
    h1 = hx1 / h1_count;
  }else {
    h1 = -1000.0;
  }
  Serial.print(F("h1: ")); Serial.println(h1);
  
  if(t2_count > 0){
    t2 = tx2 / t2_count;
  }else {
    t2 = -1000.0;
  }
  Serial.print(F("t2: ")); Serial.println(t2);
  
  if(t2_count > 0){
    h2 = hx2 / h2_count;
  }else {
    h2 = -1000.0;
  }
  Serial.print(F("h2: ")); Serial.println(h2);

  t1_count = 0;
  h1_count = 0;
  t2_count = 0;
  h2_count = 0;
  tx1 = 0.0;
  hx1 = 0.0;
  tx2 = 0.0;
  hx2 = 0.0;

}
void readDHT(){
  Serial.println(F(".............................................................."));
  Serial.println(F("READING DATA FROM SENSORS"));
  Serial.println(millis());
  Serial.print(F("Reading t1 data point: ")); Serial.println(t1_count+1);
  if(!isnan(dht1.readTemperature())){
    tx1 = tx1 + dht1.readTemperature(); 
    t1_count += 1;
    Serial.println(tx1); 
  }
  
  Serial.print(F("Reading h1 data point: ")); Serial.println(h1_count+1);
  if(!isnan(dht1.readHumidity())){
    hx1 = hx1 + dht1.readHumidity(); 
    h1_count += 1;
    Serial.println(hx1); 
  }
  
  Serial.print(F("Reading t2 data point: ")); Serial.println(t2_count+1);
  if(!isnan(dht2.readTemperature())){
    tx2 = tx2 + dht2.readTemperature(); 
    t2_count += 1;
    Serial.println(tx2); 
  }
  
  Serial.print(F("Reading h2 data point: ")); Serial.println(h2_count+1);
  if(!isnan(dht2.readHumidity())){
    hx2 = hx2 + dht2.readHumidity(); 
    h2_count += 1;
    Serial.println(hx2); 
  }
}

boolean getModemIMEI() {

  ID = modem.getIMEI();
//  Serial.println(ID);

  //check imei validity
  if (ID.length() == 15){
//    Serial.println("checking imei validity");
    int sum = 0;
//    Serial.print("Sum: "); Serial.println(sum);
    for(int8_t  i = 0; i<ID.length(); i++){
//      Serial.println(i);
      int8_t  x = ID.substring(i, i+1).toInt();
      if (i % 2 == 0){
//        Serial.println("i is even");
//        Serial.print("x is: "); Serial.println(x);
        sum = sum + x;
      }else {
//        Serial.println("i is odd");
//        Serial.print("x is: "); Serial.println(x*2);

        if(x*2 > 9){
//          Serial.println("x*2 > 9");
          sum = sum + (x*2) % 10 + (x*2) / 10;
//          Serial.print("Sum: "); Serial.println(sum);
        }else {
//          Serial.println("x*2 < 9");
          sum = sum + x*2;
//          Serial.print("Sum: "); Serial.println(sum);
        }
      }
    }
    
//    Serial.print("Final sum: "); Serial.println(sum);
    if(sum % 10 == 0){
      //THIS IS VALID IMEI
      Serial.println(ID);
      STATUS_CODE = 0;
      return true;
    }else{
      Serial.println(F("INVALID IMEI. CHECKSUM FAILED"));
      return false;
    }
  }else {
    Serial.println(F("INVALID IMEI. DIGITS COUNT MISMATCH"));
    return false;
  }
}

boolean mCon() {
  randomSeed(millis()); //Set randomsource
  clientId = ID + String(random(10,99));
  
  if (!mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    //Serial.println(F("fail"));
    return false;
  }  
  return mqtt.connected();
}

boolean publishData(){

  siglevel  = modem.getSignalQuality();
  if(siglevel > maxSig) maxSig = siglevel;
  if(siglevel < minSig) minSig = siglevel;
  if(minSig <= MIN_SIGNAL_THRESHOLD) SIGNAL_LOW = true;

  int x = modem.getBattVoltage();
      if(x != 0) battlevel = x;
      
  if(battlevel > maxBatt) maxBatt = battlevel;
  if(battlevel < minBatt) minBatt = battlevel;
  if(minBatt <= MIN_BATTERY_THRESHOLD) BATTERY_LOW = true;
    
  Serial.print(F("Battery Level: ")); Serial.println((float)battlevel/1000.0);
  String csvMessage = String(siglevel) + "," + String(battlevel) + "," + String(h1) + "," + String(t1) + "," + String(h2) + "," + String(t2) + "," + String(ID);  
  Serial.println(csvMessage);
  Serial.print(F("PUBLISHING DATA TO MQTT TOPIC: ")); Serial.println(EVENT_TOPIC);
  boolean pub = mqtt.publish(EVENT_TOPIC,csvMessage.c_str());
  
  return pub;
}

void mqttFail(){

  siglevel  = modem.getSignalQuality();
  int x = modem.getBattVoltage();
      if(x != 0) battlevel = x;
 
 //send SMS when MQTT loop failed
    modem.sendSMS(SMS_NUMBER,
    ID+","
    +ID+","
    +"mqttfail"+","
    +String(rstCnt)+","
    +String(siglevel)+","
    +String(battlevel)+","
    +FIRMWARE_VER
    );
}

boolean dailyUpdate(){
  
  //send SMS when MQTT loop failed
  boolean sent = 0;
  for(int8_t i=1; i<=10; i++){
    sent = modem.sendSMS(SMS_NUMBER, "DEVICE ID: " + ID + "\nMAX SIG: " + String(maxSig) + "\nMIN SIG: " + String(minSig) + "\nMAX BAT: " + String(maxBatt) + "\nMIN BAT: " + String(minBatt) + "\nPUB COUNT: " + String(PUBLISH_COUNT));
    //pauseFor(1000);
    if(sent) break;
  }
  
  maxSig = 0;
  minSig = 33;
  maxBatt = 0;
  minBatt = 4500;
  PUBLISH_COUNT = 0;
  return sent;
}

boolean alertSMS(String message){
  //Send Alert to owner
  boolean sent = 0;
  for(int8_t i=1; i<=10; i++){
    sent = modem.sendSMS(SMS_NUMBER, message);
    //pauseFor(1000);
    if(sent) break;
  }
  return sent;
}
