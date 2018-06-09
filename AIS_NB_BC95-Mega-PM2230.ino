#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <Magellan.h>
SoftwareSerial mySerial(10, 11); // RX(1), TX(4) MAX487 
#define MAX485_DE      6
#define MAX485_RE_NEG  6

char auth[]="0ef72bd0-1540-11e8-969b-933c93ed7f88"; 
Magellan magel;
#define OLED_RESET 5
Adafruit_SSD1306 display(OLED_RESET);

//#define MY_DEBUG
//#define USE_BME280

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

//#include "AIS_NB_BC95.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)

float minute_data = 0.0;
float hour_data = 0.0;

unsigned long delayTime;

String apnName = "devkit.nb";
String serverIP = "35.201.5.171"; // Your Server IP
String serverPort = "12347"; // Your Server Port

//AIS_NB_BC95 AISnb;
char buf[50];
#define LENG 23 
const long interval = 20000;  //millisecond
unsigned long previousMillis = 0;

String ip1; 
float Temperature = 0;
float Humidity = 0;
float Pressure = 0;
float Altitude = 0;

float x,Va,Vb,Vc,VLN,Qt,St,KWh,KVArh,KVAh,F = 0.0;  //VLN,VLL,Ia,Ib,Ic,In,Wt,KWh,KVArh,F
float Ia,Ib,Ic,In;
float Vab,Vbc,Vca,VLL;
float Wa,Wb,Wc,Wt;
float PFa,PFb,PFc,PFt;  
float Last_demand,Present_demand,Peak_demand;
float temperature,humidity;

//PM2230 Register 
#define reg_Va 3027 
#define reg_Vb 3029 
#define reg_Vc 3031 
#define reg_VLN 3035 
#define reg_Vab 3019 
#define reg_Vbc 3021 
#define reg_Vca 3023 
#define reg_VLL 3025 
#define reg_Ia 2999 
#define reg_Ib 3001 
#define reg_Ic 3003 
#define reg_In 3005 
#define reg_Wa 3053
#define reg_Wb 3055 
#define reg_Wc 3057 
#define reg_Wt 3053 
#define reg_Qt 3061 
#define reg_St 3075
#define reg_KWh 2699 
#define reg_KVArh 2707
#define reg_KVAh 2715
#define reg_PFa 3077
#define reg_PFa 3079
#define reg_PFc 3081
#define reg_F 3109
#define reg_Last_demand 3763
#define reg_Present_demand 3765
#define reg_peak_demand; 3769
  
ModbusMaster node;

bool state = true;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

float reform_uint16_2_float32(uint16_t u1, uint16_t u2)
{  
  uint32_t num = ((uint32_t)u1 & 0xFFFF) << 16 | ((uint32_t)u2 & 0xFFFF);
    float numf;
    memcpy(&numf, &num, 4);
    return numf;
}

float getRTU(uint16_t m_startAddress){
  uint8_t m_length =2;
  uint16_t result;
  float x;
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);  
  result = node.readHoldingRegisters(m_startAddress, m_length);  //readInputRegisters readHoldingRegisters
  if (result == node.ku8MBSuccess) {
     return reform_uint16_2_float32(node.getResponseBuffer(0),node.getResponseBuffer(1));
  }
}  

void setup(){ 
  
bool status;
magel.begin(auth); 
#ifdef MY_DEBUG  
  Serial.begin(9600);
  Serial.println(F("Starting"));
#endif

 //Start Modbus   
    pinMode(MAX485_RE_NEG, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
    digitalWrite(MAX485_RE_NEG, 0);
    digitalWrite(MAX485_DE, 0);
 
    mySerial.begin(9600);//pmsSerial.begin(9600);
    // Modbus slave ID 1
    node.begin(1, mySerial);
    // Callbacks allow us to configure the RS485 transceiver correctly
    delay(50);
/*
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Connecting to network");  
  display.println();
  display.display();
  delay(1000);
*/
//AISnb.debug = true;
  //AISnb.setupDevice(serverPort);
  //ip1 = AISnb.getDeviceIP();  
/*
  display.print("IP: ");
  display.println(ip1);
  display.display();
*/    
  //delayTime = 5000;
  //pingRESP pingR = AISnb.pingIP(serverIP);

  /*
#ifdef USE_BME280
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
#endif
*/  
  //previousMillis = millis();
}

int i = 0;
  
void loop(){ 
    unsigned long currentMillis = millis();
    int minat = 0;
    int maxat = 0;

    Measure_data();   
    
#ifdef MY_DEBUG      
    Serial.print(Va);Serial.print(" ");Serial.print(Vb);Serial.print(" ");Serial.print(Vc);Serial.print(" ");Serial.println(VLN);
    Serial.print(Vab);Serial.print(" ");Serial.print(Vbc);Serial.print(" ");Serial.print(Vca);Serial.print(" ");Serial.println(VLL);
    Serial.print(Ia);Serial.print(" ");Serial.print(Ib);Serial.print(" ");Serial.print(Ic);Serial.print(" ");Serial.println(In);
    Serial.print(Wt);Serial.print(" ");Serial.print(Qt);Serial.print(" ");Serial.println(St);
    Serial.print(KWh);Serial.print(" ");Serial.print(KVArh);Serial.print(" ");Serial.print(KVAh);Serial.print(" ");Serial.println(F);    
    Serial.println();
#endif    
  //printValues();

  String DataSend ="{\"id\":\"NB-IoT-Gen\",\"VLN\":"+String(VLN)+",\"VLL\":"+String(VLL)+",\"Ia\":"+String(Ia)+",\"Ib\":"+String(Ib)+",\"Ic\":"+String(Ic)+",\"In\":"+String(In)+",\"PFa\":"+String(PFa)+",\"PFb\":"+String(PFb)+",\"PFc\":"+String(PFc)+",\"Wa\":"+String(Wa)+",\"Wb\":"+String(Wb)+",\"Wc\":"+String(Wc)+",\"KWh\":"+String(KWh)+"}";     
  magel.post(DataSend);
  //  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, DataSend);
  //  UDPReceive resp = AISnb.waitResponse();
  delay(30000);    
}

void printValues() {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("BME280 & Dust Sensor");
    display.setCursor(0,20);
    display.print("Temp.:  ");
    display.println(bme.readTemperature());
    display.print("Humid.: ");  
    display.println(bme.readHumidity());
    display.display();
}

void Measure_data(){
/*
#ifdef USE_BME280
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
#endif 
*/  
    Va = getRTU(reg_Va); 
    delay(100);
    Vb = getRTU(reg_Vb); 
    delay(100);
    Vc = getRTU(reg_Vc); 
    delay(100);        
    VLN = getRTU(reg_VLN); 
    delay(100);
    
    Vab = getRTU(reg_Vab); 
    delay(10);
    Vbc = getRTU(reg_Vbc); 
    delay(10);
    Vca = getRTU(reg_Vca); 
    delay(10);     
    VLL = getRTU(reg_VLL);
    delay(10); 
    Ia = getRTU(reg_Ia); 
    delay(10);
    Ib = getRTU(reg_Ib); 
    delay(10);
    Ic = getRTU(reg_Ic); 
    delay(10);  
    In = getRTU(reg_In); 
    delay(10);    
    Wa = getRTU(reg_Wa); 
    delay(10);
    Wb = getRTU(reg_Wb); 
    delay(10);
    Wc = getRTU(reg_Wc); 
    delay(10);  
    Wt = getRTU(reg_Wt); 
    delay(10); 
    Qt = getRTU(reg_Qt); 
    delay(10);     
    St = getRTU(reg_St); 
    delay(10); 
    KWh = getRTU(reg_KWh); 
    delay(10);
    KVArh = getRTU(reg_KVArh); 
    delay(10);
    KVAh = getRTU(reg_KVAh); 
    delay(10);
    F = getRTU(reg_F); 
    
}
