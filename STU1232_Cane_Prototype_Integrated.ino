
#include<SoftwareSerial.h>//for HC-05
#include <Wire.h>//for BNO055
#include <Adafruit_Sensor.h>//for BNO055
#include <Adafruit_BNO055.h>//for BNO055
#include <utility/imumaths.h>//for BNO055

//#define LED 13

Adafruit_BNO055 bno = Adafruit_BNO055(55);//for BNO055


#define TxD 3
#define RxD 2
#define LED_PIN 13

SoftwareSerial bluetoothSerial(TxD, RxD);

char c;

void setup() {
  bluetoothSerial.begin(9600);
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);

  //for BNO055 code
  //Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
  }

void loop() {
  if(bluetoothSerial.available()){
    c = bluetoothSerial.read();
    Serial.println(c);
    if(c=='1'){
      digitalWrite(LED_PIN, HIGH);          
    }
    if(c=='0'){
      digitalWrite(LED_PIN, LOW);
    }
  }
}
