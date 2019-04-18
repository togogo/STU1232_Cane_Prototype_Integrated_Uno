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

//Define FSR pin
#define fsrpin A0 //The FSR and 10K pulldown resistor are connected to A0.

//Define variables
int fsrreading; //Variable to store FSR value

SoftwareSerial Bluetooth(TxD, RxD);
char c;

int led = 9;//digital 9 for the white LED
int brightness = 0;
int fadeAmount = 5;

int vibrationMotor = 6;

void setup() {

  //start comm. for both serial and bluetooth
  Bluetooth.begin(9600);
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  pinMode(led, OUTPUT);//for led
  pinMode(vibrationMotor, OUTPUT);


  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop() {

  fsrreading = analogRead(fsrpin);  //Reads the FSR pin and stores the output as fsrreading
  //Prints the fsrreading in the serial monitor.
  Serial.print("Analog reading = ");  //Print the string "Analog reading = ".
  Serial.print(fsrreading);           //Prints the fsrreading.
  Serial.println("");

  //for the vibration motor part...could be refined
  analogWrite(vibrationMotor, 150);
  delay(200);
  analogWrite(vibrationMotor, 0);

  //for the white led...could be refined.
  analogWrite(led, brightness);
  brightness = brightness + fadeAmount;
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  /*BNO055 code: Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  //this sends serial signal from PC -> Arduino
  //Going to use this to turn on the LED Lamp
  if (Bluetooth.available()) {
    c = Bluetooth.read();
    Serial.println(c);

    if (c == '1') {
      digitalWrite(LED_PIN, HIGH);
    }
    if (c == '0') {
      digitalWrite(LED_PIN, LOW);
    }
  }

  /* Display the floating point data for SERIAL comm. */
  /*
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  */

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
