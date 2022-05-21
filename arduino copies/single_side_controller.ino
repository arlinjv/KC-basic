#include <Wire.h>
#include <SPI.h>

/*   Single side controller
Objective:
- Enable simultaneous control of two Kelly controllers and motors
- Leave room for implementing joystick and regular FR control methods on Kelly side

Setup:
- Using 5V Arduino pro micro (Program as Leonardo. There is no LED pin)
  - Pro micro info page: https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/all
- Using mcp4251 SPI dual digital pot (tutorial: http://matthewcmcmillan.blogspot.com/2014/03/arduino-using-digital-potentiometers.html)
 - program writes same value to both pots
-I2C 
  - Keep available in case do Hall sensor control input
  - on Pro Micro pin 2 is SDA, pin 3 is SCL
- Additional connections:
  - Kelly controller 'meter' inputs
  - brakes output (feeds into opto. do parallel to direct switch input)
  - Fwd/Rev outputs (feeds into opto. do parallel to direct switch input)
*/

const int motorsPin = 10;//Chip Select for digital pot controlling motors
const int meterPinA = 9;//reserved for future implementation - don't forget to level shift
const int meterPinB = 8;
const int revPin = 5;
const int fwdPin = 6;
const int brakesPin = 7;
const int potPin = A0;

//SPI pins - don't actually need these declarations - left over from teensy code
const int mcp4251_SCK = 15;
const int mcp4251_MOSI = 16; //to SDI pin
const int mcp4251_MISO = 14; //to SDO pin


const int potMin = 168;
const int potMax = 1022;

int potVal = 0;
int mcpVal;

uint16_t cmdWord = 0;

/* Using Teensy 3.5
 *  
 * mcp4251 datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/22060b.pdf
 */

 void setup()
{  
  Serial.begin(115200);
  //delay(1000); // On Teensy, want to have serial monitor already open. Need delay because Teensy is usb based, not serial

  pinMode (potPin, INPUT);
  pinMode (motorsPin, OUTPUT); 

  digitalWrite(motorsPin, HIGH);// chip select is active low
  
  SPI.begin();
  //teensy code below commented out
  //SPI.setMOSI(mcp4251_MOSI);
  //SPI.setSCK(mcp4251_SCK);
  //SPI.setMISO(mcp4251_MISO);

  Serial.println("starting");
}

void loop(){
  
  potVal = analogRead(potPin);
  mcpVal = map(potVal, potMin, potMax, 0, 256);
  Serial.print(potVal);Serial.print(" - ");Serial.println(mcpVal);
  drive(uint16_t(mcpVal));
  delay(1);
  
  
}

void drive(uint16_t val){ 
  //Write drive val to both outputs in digital pot
  constrain (val, 0, 256); //257 possible values
  digitalWrite(motorsPin, LOW);
  //write to address 0
  //write command is 00 so we don't need to do anything fancy)
  SPI.transfer16(val);
  digitalWrite(motorsPin, HIGH);

  //write to address 1
  digitalWrite(motorsPin, LOW);
  bitSet(val, 12);
  SPI.transfer16(val);
  digitalWrite(motorsPin, HIGH);

}

/*if (abs(potVal-lastPotVal) > 2){
    Serial.println(potVal);
  }
  */
