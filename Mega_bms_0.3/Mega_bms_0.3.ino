/*

  HelloWorld.ino

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#define VersionNumber "     Ver. 230226.12"

#include <Arduino.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Adafruit_ADS1015.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#define CutoffLowVoltageAddress 0 //EEPROM address
#define CutoffHighVoltageAddress 10 //EEPROM address
#define Factor1_address 20
#define Factor2_address 30
#define Factor3_address 40
#define Factor4_address 50
#define Factor5_address 60
#define Factor6_address 70
#define Factor7_address 80
#define Factor8_address 90

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

Adafruit_ADS1115 ads1(0x48);
Adafruit_ADS1115 ads2(0x49);
/*

Scanning...
I2C device found at address 0x3C  !
I2C device found at address 0x48  !

0x3C är display
0x48 är ADC

*/
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

/*
  U8glib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.


    Lilla displayen (pchbutik 1667 har I2C-adress 0x3C
*/
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

const int MenuButtonPin   = 10; //GPIO14 = D5
const int EnterButtonPin  = 12;
const int PlusButtonPin   = 14;
const int MinusButtonPin  = 8;
const int LoadRelayOffPin = 26;
const int LoadRelayOnPin = 28;
const int ChargeRelayOffPin = 22;
const int ChargeRelayOnPin = 24;
const int SummerPin = 19; //D19

#define menu_Cells 0
#define menu_AnalogReading 1
#define menu_MaxMin 3
#define menu_RelayControl 4

#define LoadRelayOn 1
#define LoadRelayOff 2
#define ChargeRelayOn 3
#define ChargeRelayOff 4

unsigned long previousMillis = 0;        // will store last time LED was updated
byte displaymode = 0;
boolean buttonpressed = false;
byte menuCursor = 0;
float S[8] = {3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8};
float U1[4] = {1, 2, 3, 4};
float U2[4] = {1, 2, 3, 4};
float U3[4] = {1, 2, 3, 4};
float U4[4] = {1, 2, 3, 4};
float U5[4] = {1, 2, 3, 4};
float U6[4] = {1, 2, 3, 4};
float U7[4] = {1, 2, 3, 4};
float U8[4] = {1, 2, 3, 4};

byte readCounter = 0;
/*
float factor1 = 7.8;
float factor2 = 7.7;
float factor3 = 7.7;
float factor4 = 7.7;
float factor5 = 7.7;
float factor6 = 7.7;
float factor7 = 7.7;
float factor8 = 7.7;
*/
float factor[8] = {1, 2, 2.44, 3.25, 6.68, 10, 7.89, 7.86};

float CutoffLowVoltage = 2.8;
int NumCutoffLow = 0;
float CutoffHighVoltage = 3.5;
int NumCutoffHigh = 0;

// constants won't change :
const long interval = 200;       // interval at which to blink (milliseconds)
const long shortInterval = 1000;  // milliseconds
int counter = 0;
byte displayMode = 0;

void setup(void) {
  display.begin();

  pinMode(SummerPin, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MenuButtonPin, INPUT_PULLUP);
  pinMode(EnterButtonPin, INPUT_PULLUP);
  pinMode(PlusButtonPin, INPUT_PULLUP);
  pinMode(MinusButtonPin, INPUT_PULLUP);
  
  pinMode(LoadRelayOffPin, OUTPUT);
  digitalWrite(LoadRelayOffPin, HIGH);
  pinMode(LoadRelayOnPin, OUTPUT);
  digitalWrite(LoadRelayOnPin, HIGH);
  pinMode(ChargeRelayOnPin, OUTPUT);
  digitalWrite(ChargeRelayOnPin, HIGH);
  pinMode(ChargeRelayOffPin, OUTPUT);
  digitalWrite(ChargeRelayOffPin, HIGH);
    
  Serial.begin(9600);
  
  ads1.setGain(GAIN_TWOTHIRDS);  
  ads1.begin();
  
  ads2.setGain(GAIN_TWOTHIRDS);
  ads2.begin();
  
  sensors.begin();
  delay(10);
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
  sensors.requestTemperatures(); // Send the command to get temperatures
  float f;
  f = sensors.getTempC(insideThermometer);
  Serial.println("Temperatur:");
  Serial.println(f);

  EEPROM.get(Factor1_address, factor[0]);
  EEPROM.get(Factor2_address, factor[1]);
  EEPROM.get(Factor3_address, factor[2]);
  EEPROM.get(Factor4_address, factor[3]);
  EEPROM.get(Factor5_address, factor[4]);
  EEPROM.get(Factor6_address, factor[5]);
  EEPROM.get(Factor7_address, factor[6]);
  EEPROM.get(Factor8_address, factor[7]);

  EEPROM.get(CutoffLowVoltageAddress, CutoffLowVoltage);
  EEPROM.get(CutoffHighVoltageAddress, CutoffHighVoltage);

  ReadCellVoltage();
}

void loop(void) {
  CheckButtons();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {    
    digitalWrite(LED_BUILTIN, HIGH);
    if (readCounter == 4) {
      
      SortDec(U1, 4);
      S[0] = (U1[1] + U1[2]) / 2;
      SortDec(U2, 4);
      S[1] = (U2[1] + U2[2]) / 2;
      SortDec(U3, 4);
      S[2] = (U3[1] + U3[2]) / 2;      
      SortDec(U4, 4);
      S[3] = (U4[1] + U4[2]) / 2;
      SortDec(U5, 4);
      S[4] = (U5[1] + U5[2]) / 2;
      SortDec(U6, 4);
      S[5] = (U6[1] + U6[2]) / 2;
      SortDec(U7, 4);
      S[6] = (U7[1] + U7[2]) / 2;
      SortDec(U8, 4);
      S[7] = (U8[1] + U8[2]) / 2;
        
      CheckCutoff();
      displayscreen();
      readCounter = 0;
    } else {
      ReadCellVoltage();
      
      U1[readCounter] = S[0];
      U2[readCounter] = S[1];
      U3[readCounter] = S[2];
      U4[readCounter] = S[3];
      U5[readCounter] = S[4];
      U6[readCounter] = S[5];
      U7[readCounter] = S[6];
      U8[readCounter] = S[7];
      
      readCounter++;      
      
    }
    previousMillis = currentMillis;
    
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void ReadCellVoltage() {
  long vccmW = readVcc();
  S[0] = 5000/vccmW*0.1875 * ads1.readADC_SingleEnded(0)/ 1000 * factor[0];
  S[1] = 5000/vccmW*0.1875 * ads1.readADC_SingleEnded(1)/ 1000 * factor[1] - S[0];  
  S[2] = 5000/vccmW*0.1875 * ads1.readADC_SingleEnded(2)/ 1000 * factor[2] - S[0] - S[1];  
  S[3] = 5000/vccmW*0.1875 * ads1.readADC_SingleEnded(3)/ 1000 * factor[3] - S[0] - S[1] - S[2];  
  S[4] = 5000/vccmW*0.1875 * ads2.readADC_SingleEnded(0)/ 1000 * factor[4] - S[0] - S[1] - S[2] - S[3];
  S[5] = 5000/vccmW*0.1875 * ads2.readADC_SingleEnded(1)/ 1000 * factor[5] - S[0] - S[1] - S[2] - S[3] - S[4];
  S[6] = 5000/vccmW*0.1875 * ads2.readADC_SingleEnded(2)/ 1000 * factor[6] - S[0] - S[1] - S[2] - S[3] - S[4] - S[5];
  S[7] = 5000/vccmW*0.1875 * ads2.readADC_SingleEnded(3)/ 1000 * factor[7] - S[0] - S[1] - S[2] - S[3] - S[4] - S[5] - S[6];
}

void CheckButtons() {
  int MenuButton = digitalRead(MenuButtonPin);
  int PlusButton = digitalRead(PlusButtonPin);
  int MinusButton = digitalRead(MinusButtonPin);
  int EnterButton = digitalRead(EnterButtonPin);  
  
  if (PlusButton == LOW || MinusButton == LOW || EnterButton == LOW || MenuButton == LOW) {
    displayscreen();
  }
  
  if(buttonpressed) {      
      while (PlusButton == LOW || MinusButton == LOW || EnterButton == LOW || MenuButton == LOW) {
        MenuButton = digitalRead(MenuButtonPin);  
        PlusButton = digitalRead(PlusButtonPin);
        MinusButton = digitalRead(MinusButtonPin);
        EnterButton = digitalRead(EnterButtonPin);  
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= shortInterval) {
          previousMillis = currentMillis;
          if (PlusButton == LOW) {  
            if (menuCursor==1) factor[0] = factor[0] + .002;
            if (menuCursor==2) factor[1] = factor[1] + .002;
            if (menuCursor==3) factor[2] = factor[2] + .002;
            if (menuCursor==4) factor[3] = factor[3] + .002;
            if (menuCursor==5) factor[4] = factor[4] + .002;
            if (menuCursor==6) factor[5] = factor[5] + .002;
            if (menuCursor==7) factor[6] = factor[6] + .002;
            if (menuCursor==8) factor[7] = factor[7] + .002;
          }
          if (MinusButton == LOW) {
            if (menuCursor==1) factor[0] = factor[0] - .002;
            if (menuCursor==2) factor[1] = factor[1] - .002;
            if (menuCursor==3) factor[2] = factor[2] - .002;
            if (menuCursor==4) factor[3] = factor[3] - .002;
            if (menuCursor==5) factor[4] = factor[4] - .002;
            if (menuCursor==6) factor[5] = factor[5] - .002;
            if (menuCursor==7) factor[6] = factor[6] - .002;
            if (menuCursor==8) factor[7] = factor[7] - .002;
            ReadCellVoltage();
            buttonpressed=true;
          }
          ReadCellVoltage();
          displayscreen();
        }    
        delay(20);        
      }      
      buttonpressed = false;
  }
  
  switch (displaymode) {
    case menu_Cells:
      if (EnterButton == LOW) {
        pip();
        if (menuCursor==0) menuCursor++;          
        if (menuCursor==9) {
          SaveCalibration();
          menuCursor = 0;
          displaymode = menu_Cells;          
          pip();
        }
        if (menuCursor==10) {
          SetAllVoltagesEqual();
          menuCursor = 9;
          displaymode = menu_Cells;
        }
      }
      if (MenuButton == LOW) {
        if (menuCursor > 0) menuCursor++;  
        if (menuCursor > 10) menuCursor = 0;
        pip();
      }
      if (PlusButton == LOW) {                
        if (menuCursor==1) factor[0] = factor[0] + .0005;
        if (menuCursor==2) factor[1] = factor[1] + .0005;
        if (menuCursor==3) factor[2] = factor[2] + .0005;
        if (menuCursor==4) factor[3] = factor[3] + .0005;
        if (menuCursor==5) factor[4] = factor[4] + .0005;
        if (menuCursor==6) factor[5] = factor[5] + .0005;
        if (menuCursor==7) factor[6] = factor[6] + .0005;
        if (menuCursor==8) factor[7] = factor[7] + .0005;
        ReadCellVoltage();
        buttonpressed=true;
        pip();
      }     
      if (MinusButton == LOW) {
        if (menuCursor==1) factor[0] = factor[0] - .0005;
        if (menuCursor==2) factor[1] = factor[1] - .0005;
        if (menuCursor==3) factor[2] = factor[2] - .0005;
        if (menuCursor==4) factor[3] = factor[3] - .0005;
        if (menuCursor==5) factor[4] = factor[4] - .0005;
        if (menuCursor==6) factor[5] = factor[5] - .0005;
        if (menuCursor==7) factor[6] = factor[6] - .0005;
        if (menuCursor==8) factor[7] = factor[7] - .0005;
        ReadCellVoltage();
        buttonpressed=true;
        pip();
      }
      if (MenuButton == LOW) {
        if (menuCursor==0) displaymode = menu_AnalogReading;        
        //menuCursor = 0;
        buttonpressed=true;
        pip();
      }           
    break;
    case menu_AnalogReading:
      if (MenuButton == LOW) {
        displaymode = menu_MaxMin;        
        menuCursor = 0;     
        buttonpressed=true;
        pip();
      }     
      if (EnterButton == LOW) {
        if (menuCursor==0) menuCursor = 1;  
        if (menuCursor==1) {
          menuCursor = 0;     
          pip();
          //SetAllVoltagesEqual();   
        }
      }
    break;    
    case menu_MaxMin:
      if (MenuButton == LOW) {
        buttonpressed=true;
        pip();
        if(menuCursor == 0) {
          displaymode = menu_RelayControl;  
        }
        if(menuCursor >0)  {
          menuCursor++;
          if(menuCursor ==4) menuCursor = 0;
        }
        
      }
      if (PlusButton == LOW) {
        buttonpressed=true;
        if (menuCursor == 1) CutoffLowVoltage = CutoffLowVoltage + 0.01;                
        if (menuCursor == 2) CutoffHighVoltage = CutoffHighVoltage + 0.01;                
        pip();
      }
      if (MinusButton == LOW) {
        buttonpressed=true;
        if (menuCursor == 1) CutoffLowVoltage = CutoffLowVoltage - 0.01;                
        if (menuCursor == 2) CutoffHighVoltage = CutoffHighVoltage - 0.01;                
        pip();
      }
      if (EnterButton == LOW) {
        buttonpressed=true;
        if (menuCursor == 0) {
          menuCursor = 1;
        }
        if (menuCursor == 3) {
          //Save
          saveVoltage(true, CutoffLowVoltage);
          saveVoltage(false, CutoffHighVoltage);          
          menuCursor = 0;
          pip();
          display.clearBuffer();
          display.setCursor(30,30);
          display.print(F("Saved"));
          display.sendBuffer();          // transfer internal memory to the display
          delay(500);
          //CutoffLowVoltage = getV(true);
        }     
      }
    break;
    case menu_RelayControl:
      if (MenuButton == LOW) {
        displaymode = menu_Cells;  
        //ReadFactors();
        menuCursor = 0;      
        buttonpressed=true;
      }
      if (PlusButton == LOW) {
        pip();
        if (menuCursor == 4) {
          menuCursor = 1;                
        } else {
          menuCursor++;
        }
        buttonpressed=true;
      }     
      if (MinusButton == LOW) {
        pip();
        if (menuCursor == 1) {
          menuCursor = 4;                
        } else {
          menuCursor--;
        }
        buttonpressed=true;
      }
      if (EnterButton == LOW) {
        pip();
        buttonpressed=true;
        if (menuCursor == 1) {
          operateRelay(LoadRelayOff);
          menuCursor = 1;
        }
        if (menuCursor == 2) {
          operateRelay(LoadRelayOn);
          menuCursor = 1;
        }
        if (menuCursor == 3) {
          operateRelay(ChargeRelayOff);
          menuCursor = 1;
        }
        if (menuCursor == 4) {
          operateRelay(ChargeRelayOn);
          menuCursor = 1;
        }
      }
    break;  
  } 
}

void displayscreen() {
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_t0_11_te); 
  
  switch (displaymode) {
    case menu_Cells:        
      CellVoltageMenu();              
    break;
    case menu_AnalogReading:
      CellVoltageSorted(true);                    
    break;
    case menu_MaxMin:
      showMenu_MaxMin();
    break;
    case menu_RelayControl:
      showRelayControlMenu();
    break;      
  }
}

void CellVoltageSorted(bool ShowTotal){
  float S1 = S[0];
  float S2 = S[1];
  float S3 = S[2];
  float S4 = S[3];
  float S5 = S[4];
  float S6 = S[5];
  float S7 = S[6];
  float S8 = S[7];
  
  SortDec(S, 8);
  byte numOfDecimals = 3;  
  String s = "S0";

  for (int i=0; i<8; i++){
    if(S[i] == S1) s = "S1";
    if(S[i] == S2) s = "S2";
    if(S[i] == S3) s = "S3";
    if(S[i] == S4) s = "S4";
    if(S[i] == S5) s = "S5";
    if(S[i] == S6) s = "S6";
    if(S[i] == S7) s = "S7";
    if(S[i] == S8) s = "S8";

    //if(((i / 2) & 1) == 0) display.setCursor(0, 10 + (11 * i));    
    if (i==0) display.setCursor(0, 10);    
    if (i==2) display.setCursor(0, 21);    
    if (i==4) display.setCursor(0, 32);    
    if (i==6) display.setCursor(0, 43);    
    printNumeric(s, true, S[i], true, numOfDecimals);
  }
  if (ShowTotal) {
    display.setCursor(0, 63);     
    printNumeric(F(" Details:"), true, S[7] + S[6] + S[5]  + S[4]  + S[3]  + S[2]  + S[1]  + S[0], false, 1);  
  }
  display.sendBuffer();  
  Serial.println();
  Serial.print("F1: ");
  Serial.print(factor[0]);
  Serial.print(" S1: ");
  Serial.print(S[0]);
  Serial.print(" F2: ");
  Serial.print(factor[1]);
  Serial.print(" S2: ");
  Serial.print(S[1]);
  Serial.print(" F3: ");
  Serial.print(factor[2]);
  Serial.print(" F4: ");
  Serial.print(factor[3]);
  
}

void CellVoltageMenu() {
  byte numOfDecimals = 2;  
  if(menuCursor>0) numOfDecimals = 3;
  
  display.setCursor(0, 10);
  if(menuCursor==1) display.print(F("*"));
  printNumeric(F("S1"), true, S[0], true, numOfDecimals);  
  if(menuCursor==5) display.print(F("*"));
  printNumeric(F("S5"), true, S[4], false, numOfDecimals);
  
  display.setCursor(0, 21);
  if(menuCursor==2) display.print(F("*"));
  printNumeric(F("S2"), true, S[1], true, numOfDecimals);  
  if(menuCursor==6) display.print(F("*"));
  printNumeric(F("S6"), true, S[5], false, numOfDecimals);

  display.setCursor(0, 32);
  if(menuCursor==3) display.print(F("*"));
  printNumeric(F("S3"), true, S[2], true, numOfDecimals);
  if(menuCursor==7) display.print(F("*"));
  printNumeric(F("S7"), true, S[6], false, numOfDecimals);

  display.setCursor(0, 43);
  if(menuCursor==4) display.print(F("*"));
  printNumeric(F("S4"), true, S[3], true, numOfDecimals);
  if(menuCursor==8) display.print(F("*"));
  printNumeric(F("S8"), true, S[7], false, numOfDecimals);

  display.setCursor(0, 63);
  if(menuCursor==9) {        
      display.print(F("* Save calibration"));  
  }
  if(menuCursor==10) {        
      display.print(F("*Set all equal to S1"));  
  }       
  switch (menuCursor) {      
    case 0:       
      
      printNumeric(F("   Total:"), true, S[7] + S[6] + S[5]  + S[4]  + S[3]  + S[2]  + S[1]  + S[0], false, 1);
      
      break;
    case 1:        
      printNumeric(F("Calibrate S1"), false, factor[0], false, 4);
      break;
    case 2:        
      printNumeric(F("Calibrate S2"), false, factor[1], false, 4);
      break;
    case 3:        
      printNumeric(F("Calibrate S3"), false, factor[2], false, 4);
      break;
    case 4:        
      printNumeric(F("Calibrate S4"), false, factor[3], false, 4);
      break;
    case 5:        
      printNumeric(F("Calibrate S5"), false, factor[4], false, 4);
      break;
    case 6:        
      printNumeric(F("Calibrate S6"), false, factor[5], false, 4);
      break;
    case 7:        
      printNumeric(F("Calibrate S7"), false, factor[6], false, 4);
      break;
    case 8:        
      printNumeric(F("Calibrate S8"), false, factor[7], false, 4);
      break;      
  }   
  display.sendBuffer();
}
void printNumeric(String s, bool voltage, float num, bool spaceAfter, byte decimals) {
  char myStg[8];
  display.print(s);
  display.print(F(" "));
  if (decimals==4) dtostrf(num, 6, 4, myStg); 
  if (decimals==3) dtostrf(num, 4, 3, myStg); 
  if (decimals==2) dtostrf(num, 4, 2, myStg);   
  if (decimals==1) dtostrf(num, 3, 1, myStg);   
  if (decimals==0) dtostrf(num, 4, 0, myStg);   
  //dtostrf(num, 3, 2, myStg); 
  display.print(myStg);
  if (voltage) display.print(F("V"));
  if (spaceAfter) display.print(F("  "));  
}
void showAnalogReadings(){
//  
  char myStg[8];
  display.setCursor(0, 11);
  display.print(F("Analog readings etc."));

  display.setCursor(0, 22);
  printNumeric("S1", false, ads1.readADC_SingleEnded(0), false, 0);
  delay(10);
  display.setCursor(63, 22);
  printNumeric("S2", false, ads1.readADC_SingleEnded(1), false, 0);
  Serial.println(ads1.readADC_SingleEnded(1));
  
  delay(10);
  display.setCursor(0, 33);
  printNumeric("S6", false, ads2.readADC_SingleEnded(1), false, 0);
  display.setCursor(63, 33);
  printNumeric("S7", false, ads2.readADC_SingleEnded(2), false, 0);
  display.setCursor(0, 63);
  printNumeric("S8", false, ads2.readADC_SingleEnded(3), false, 0);
//  
//  display.setCursor(0, 33);
//  display.print(F("A2(2): "));
//  dtostrf(ads2.readADC_SingleEnded(2), 6, 0, myStg); 
//  display.print(myStg);
//
////  display.setCursor(0, 44);
////  display.print(F("A3(2): "));
////  dtostrf(ads2.readADC_SingleEnded(3), 6, 0, myStg); 
////  display.print(myStg);
//
//  display.setCursor(0, 54);
//  display.print(F("VCC: "));
//  dtostrf(readVcc(), 5, 0, myStg); 
//  display.print(myStg);
//  display.println(F("mV"));
  display.sendBuffer();
}

void showMenu_MaxMin() {
  char myStg[8];
  display.setCursor(0, 11);
  display.print(F("Max and min settings"));
  display.drawLine(0, 12, 128-1, 12);
  display.setCursor(0, 23);
  displaySelectMarkerForCursor(1);  
  display.print(F(" Low cutoff:  "));
  dtostrf(CutoffLowVoltage, 3, 2, myStg); 
  display.print(myStg);
  display.println(F("V"));

  display.setCursor(0, 34);
  displaySelectMarkerForCursor(2); 
  display.println(F(" High cutoff: "));
  dtostrf(CutoffHighVoltage, 3, 2, myStg); 
  display.print(myStg);
  display.println(F("V"));

  display.setCursor(0, 45);
  displaySelectMarkerForCursor(3);
  display.println(F(" Save"));  
  display.setCursor(0, 56);
  display.println(F(VersionNumber));  
  
  
  display.sendBuffer();
}
void showRelayControlMenu(){
  char myStg[8];
  display.setCursor(0,10);
  display.print(F("Manual Relay control"));
  
  display.drawLine(0, 11, 128-1, 11);
  
  display.setCursor(0, 23);
  displaySelectMarkerForCursor(1);
  display.println(F(" Turn Load OFF"));
  display.setCursor(0, 34);
  displaySelectMarkerForCursor(2);
  display.println(F(" Turn Load ON"));
  display.setCursor(0, 45);
  displaySelectMarkerForCursor(3);
  display.println(F(" Turn Charge OFF"));
  display.setCursor(0, 56);
  displaySelectMarkerForCursor(4);
  display.println(F(" Turn Charge ON"));

  display.sendBuffer();
}
void displaySelectMarkerForCursor(int cursorValue) {
  if (menuCursor == cursorValue) {
    display.print(F("*"));
  } else {
    display.print(F(" "));
  }
}

void SaveCalibration() {
  display.clearBuffer();
  display.setCursor(10,60);  
  display.print(F("Sparar....."));  
  display.sendBuffer();         

  EEPROM.put(Factor1_address, factor[0]);
  EEPROM.put(Factor2_address, factor[1]);
  EEPROM.put(Factor3_address, factor[2]);
  EEPROM.put(Factor4_address, factor[3]);
  EEPROM.put(Factor5_address, factor[4]);
  EEPROM.put(Factor6_address, factor[5]);
  EEPROM.put(Factor7_address, factor[6]);
  EEPROM.put(Factor8_address, factor[7]);
  delay(1000);   
} 

void operateRelay(byte relayNumber){
  delay(10);
  if (relayNumber == LoadRelayOff) {    
    digitalWrite(LoadRelayOffPin, LOW);
    delay(400);
    digitalWrite(LoadRelayOffPin, HIGH);
  } 
  if (relayNumber == LoadRelayOn) {    
    NumCutoffLow = 0;
    digitalWrite(LoadRelayOnPin, LOW);
    delay(300);
    digitalWrite(LoadRelayOnPin, HIGH);
  }
  if (relayNumber == ChargeRelayOn) {    
    NumCutoffHigh = 0;
    digitalWrite(ChargeRelayOnPin, LOW);
    delay(300);
    digitalWrite(ChargeRelayOnPin, HIGH);
  }
  if (relayNumber == ChargeRelayOff) {        
    digitalWrite(ChargeRelayOffPin, LOW);
    delay(300);
    digitalWrite(ChargeRelayOffPin, HIGH);
  }
}

long readVcc() {
  //4888
  long result = 4800; //analogRead(A0);
  
  return result;
}
void saveVoltage(bool LowVoltage, float U) {
  if (LowVoltage) {
    EEPROM.put(CutoffLowVoltageAddress, CutoffLowVoltage);
  } else {
    EEPROM.put(CutoffHighVoltageAddress, CutoffHighVoltage);
  }
}
void CheckCutoff() {
  if (S[0]  > CutoffHighVoltage || S[1] > CutoffHighVoltage || S[2] > CutoffHighVoltage || S[3] > CutoffHighVoltage || S[4]  > CutoffHighVoltage || S[5] > CutoffHighVoltage || S[6] > CutoffHighVoltage|| S[7] > CutoffHighVoltage) {
    NumCutoffHigh++;
    display.clearBuffer();
    CellVoltageSorted(false);
    display.setCursor(0,63);
    display.print(F("Overcharge detected "));
    display.sendBuffer();
    delay(800); 
    if (NumCutoffHigh > 10) {
      NumCutoffHigh = 0;
      display.clearBuffer();
      CellVoltageSorted(false);
      display.setCursor(0,63);
      display.print(F("Battery full   "));
      display.sendBuffer();
      operateRelay(ChargeRelayOff);
      delay(4000);  
    }
  }
  if (S[0] < CutoffLowVoltage || S[1]  < CutoffLowVoltage || S[2] < CutoffLowVoltage || S[3] < CutoffLowVoltage || S[4] < CutoffLowVoltage || S[5]  < CutoffLowVoltage || S[6] < CutoffLowVoltage || S[7] < CutoffLowVoltage) {     
    NumCutoffLow++;
    
    if (NumCutoffLow==5) {
      operateRelay(LoadRelayOff);
      display.clearBuffer();
      //display.setFont(u8g2_font_t0_11_te);         
      display.setCursor(27,11);
      display.print(F("LOW BAT - Charge"));  
      display.sendBuffer();

      for (int z=0; z<10; z++) {
        digitalWrite(SummerPin, HIGH);
        delay(100);  
        digitalWrite(SummerPin, LOW);
        delay(200);  
      }
      
      digitalWrite(SummerPin, HIGH);
      delay(1000);  
      digitalWrite(SummerPin, LOW);
      delay(2500);  
      operateRelay(ChargeRelayOn);
    }
    
    if (NumCutoffLow>100) {
      NumCutoffLow = 10;
      //
      display.clearBuffer();
      //display.setFont(u8g2_font_t0_11_te);         
      display.setCursor(27,11);
      display.print(F("Low bat"));  
      display.sendBuffer();
      //operateRelay(ChargeRelayOff);

      for (int z=0; z<30; z++) {
        digitalWrite(SummerPin, HIGH);
        delay(50);  
        digitalWrite(SummerPin, LOW);
        delay(450);  
      }
      
      delay(500);  
    }
  }
}

int FindMax(float ARRAY[], byte START, byte END)
{
  float MAXIMUM;
  byte LOCATION;
  
  MAXIMUM = ARRAY[START];
  LOCATION = START;
  for(byte i = START + 1; i < END; i++)
  {
    if(ARRAY[i] > MAXIMUM)
    {
      MAXIMUM = ARRAY[i];
      LOCATION = i;
    }
  }
  
  return LOCATION;
}

void Swap(float ARRAY[], byte a, byte b)
{
  float temp;
  byte location;
  
  temp = ARRAY[a];
  ARRAY[a] = ARRAY[b];
  ARRAY[b] = temp;
}

void SortDec(float ARRAY[], byte SIZE)
{
  byte location;
  
  for(byte i = 0; i < SIZE - 1; i++)
  {
    location = FindMax(ARRAY, i, SIZE);
    
    Swap(ARRAY, i, location);
  }
}

void SetAllVoltagesEqual() {
  long vccmW = readVcc();
  Serial.println(F("SetAllVoltagesEqual"));
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println();
  Serial.print("Analog read S8: ");
  Serial.print(ads2.readADC_SingleEnded(3));
  Serial.println();
  Serial.println();
  
  int f = 1;
  float a;
  float ar[5];
  
  while (f < 8) {
    Serial.println();
    Serial.print(F("factor["));
    Serial.print(f);
    Serial.print("]: ");
    Serial.print(factor[f]);
    factor[f] = factor[f] / 2;
    int i = 0;
    while (i<5) {
      if(f<4) ar[i] = ads1.readADC_SingleEnded(f); 
      if(f>=4) ar[i] = ads2.readADC_SingleEnded(f-4); 
      Serial.println(ar[i]);
      delay(500); 
      i++;
    }
    Serial.println(F(" Read analog completed"));
    SortDec(ar, 5);
    a = ar[2];
    Serial.println(a);            

    Serial.println();
    Serial.print("Starting from ");
    Serial.println(factor[f]);
    S[f] = 0;
    
    while (S[f] < S[0]) {
      factor[f] = factor[f] + 0.0001;
      if(f==1) S[1] = 5000/vccmW*0.1875 * a/ 1000 * factor[1] - S[0];  
      if(f==2) S[2] = 5000/vccmW*0.1875 * a/ 1000 * factor[2] - S[0] - S[1];
      if(f==3) S[3] = 5000/vccmW*0.1875 * a/ 1000 * factor[3] - S[0] - S[1] - S[2];  
      if(f==4) S[4] = 5000/vccmW*0.1875 * a/ 1000 * factor[4] - S[0] - S[1] - S[2] - S[3];
      if(f==5) S[5] = 5000/vccmW*0.1875 * a/ 1000 * factor[5] - S[0] - S[1] - S[2] - S[3] - S[4];
      if(f==6) S[6] = 5000/vccmW*0.1875 * a/ 1000 * factor[6] - S[0] - S[1] - S[2] - S[3] - S[4] - S[5];
      if(f==7) S[7] = 5000/vccmW*0.1875 * a/ 1000 * factor[7] - S[0] - S[1] - S[2] - S[3] - S[4] - S[5] - S[6];
    }
    
    Serial.print("Ending at ");
    Serial.print(factor[f]);
    Serial.println();
    f++;
  }
  
//  Serial.println(F("Start S3"));
//  a = ads1.readADC_SingleEnded(1);
//  while (S[2] < S[0]) {
//    factor[2] = factor[2] + 0.0002;
//    S[2] = 5000/vccmW*0.1875 * a/ 1000 * factor[2] - S[0] - S[1];
//  }
//
//  a = ads1.readADC_SingleEnded(3);
//  while (S[3] < S[0]) {
//    factor[3] = factor[3] + 0.0001;
//    S[3] = 5000/vccmW*0.1875 * a/ 1000 * factor[3] - S[0] - S[1] - S[2];  
//  }
//  a = ads2.readADC_SingleEnded(0);
//  while (S[4] < S[0]) {
//    factor[4] = factor[4] + 0.0001;
//    S[4] = 5000/vccmW*0.1875 * a/ 1000 * factor[4] - S[0] - S[1] - S[2] - S[3];
//  }
//  a = ads2.readADC_SingleEnded(1);
//  while (S[5] < S[0]) {
//    factor[5] = factor[5] + 0.0001;
//    S[5] = 5000/vccmW*0.1875 * a/ 1000 * factor[5] - S[0] - S[1] - S[2] - S[3] - S[4];
//  }
//  a = ads2.readADC_SingleEnded(2);
//  while (S[6] < S[0]) {
//    factor[6] = factor[6] + 0.0001;
//    S[6] = 5000/vccmW*0.1875 * a/ 1000 * factor[6] - S[0] - S[1] - S[2] - S[3] - S[4] - S[5];
//  }
//  a = ads2.readADC_SingleEnded(3);
//  while (S[7] < S[0]) {
//    factor[7] = factor[7] + 0.0001;
//    S[7] = 5000/vccmW*0.1875 * a/ 1000 * factor[7] - S[0] - S[1] - S[2] - S[3] - S[4] - S[5] - S[6];
//  }
  

   digitalWrite(LED_BUILTIN, LOW);
}
void pip(void) {
  digitalWrite(SummerPin, HIGH);
  delay(50);  
  digitalWrite(SummerPin, LOW);  
}
