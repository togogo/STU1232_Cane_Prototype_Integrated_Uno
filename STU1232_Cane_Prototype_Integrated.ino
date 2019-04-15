#include<SoftwareSerial.h>//for HC-05
#include <Wire.h>//for BNO055
#include <Adafruit_Sensor.h>//for BNO055
#include <Adafruit_BNO055.h>//for BNO055
#include <utility/imumaths.h>//for BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55);//for BNO055

//#define LED 13//not used at the moment

//PINS
#define TxD 3
#define RxD 2
#define LED_PIN 13

SoftwareSerial Bluetooth(TxD, RxD);
char c;

void setup() {

  //start comm. for both serial and bluetooth
  Bluetooth.begin(9600);
  Serial.begin(9600);
  
  pinMode(LED_PIN, OUTPUT);


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

  /*BNO055 code: Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  //this sends serial signal from PC -> Arduino
  //Going to use this to turn on the LED Lamp
  if(Bluetooth.available()){
    c = Bluetooth.read();
    Serial.println(c);
    
    if(c=='1'){
      digitalWrite(LED_PIN, HIGH);          
    }
    if(c=='0'){
      digitalWrite(LED_PIN, LOW);
    }
  }

  /* Display the floating point data for SERIAL comm. */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  /*Display the floating point data for BLUETOOTH comm. */
  Bluetooth.print("X: ");
  Bluetooth.print(event.orientation.x, 4);
  Bluetooth.print("\tY: ");
  Bluetooth.print(event.orientation.y, 4);
  Bluetooth.print("\tZ: ");
  Bluetooth.print(event.orientation.z, 4);
  Bluetooth.println("");
  
  delay(100);//need to play around with this variable
}
