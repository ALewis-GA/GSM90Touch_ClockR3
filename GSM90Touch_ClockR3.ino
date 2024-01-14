/*****************************************************************
// Upgraded to include RTC 2024-01  AML
// Will not run on Uno-R3 because RTC I2C communications clash with TFT screen control pins on A4
// (SDA/SCL are connected in parallel to A4, A5 on Uno R3)  
// Works well with a Mega 2560 R3  (I2C on D20 and D21)
//
// Compiled using Arduino IDE V 2.2.1
// used about 13% of program storage space on a Mega 2560 R3
// and 11% of dynamic memory
// 
// Serial Comms to GSM90 magnetometer for std absolute observations with TFT touch screen display
// assumes: Arduino Mega 2560 R3
//        : TFT LCD TouchScreen 240 x 320  ID 0x9595 05 Ox8230 (Jaycar XC4630)
//        : RS-232 shield (must be enabled with on-board switch) (Core Electronics DFR0258)
//        : Adafruit_GFX library; TouchScreen library; EEPROM Library; DFRobit RTC library
//        : MCUFriend-kbv library
//        : DFRobot DS1307 "Gravity" RTC module with CR1202 backup battery
//        : The RT clock modeule can be powered from VCC and GND pins accessible on RS-232 shield but this
//        : requires and angled 2 or 4-pin header block to connect onto the RS-232 pins.
//
// With disabled RS-232 shield all serial comms are 
// directed to Arduino IDE GUI and can be viewed from the "serial monitor" in the 
// for checking/debugging.
// Menu control via TouchScreen buttons to set RTC clock time, adjust Baud_rate, 
// adjust magnetometer_tune_value (in microTesla) and adjust number_of_obs_per_run. 
// The current value of the latter three  variables are stored to persistent EEPROM
// memory whenever they are updated and are used after reboot/power cycle.
// RS-232 parity, data_bits, stop_bits are hardwired into code as N81
// Magnetometer sampling interval is hardwired at about 10 seconds
//
// Version 1.1 2022-10-18 increase wait period at end of comms
// Version 1.3 2022-11-08 include serial buffer flush before/after "T" and after "F" command
// Version 1.4 2023-04-16 more info displayed to TFT screen, none displayed to serial monitor at start up
// Version 1.5 2023-12-05 increase GSM90F_timeout from 3500 to 4000 to work with older style magnetometers
// Version 2.0 2024-01    Incorporate DS1307 RT Clock on Mega R3
******************************************************************/
// Graphics library
#include <Adafruit_GFX.h>

// TFT touchscreen hardware library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

// touch screen library
#include <TouchScreen.h>

// DRFRobot R-T Clock library
#include <DFRobot_DS1307.h>

// EEPROM memory library
#include <EEPROM.h>

// Set software version number
#define _VERSION 2.0

// Touch screen pressure limits
#define MINPRESSURE 200
#define MAXPRESSURE 1000

// TFT screen colours
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// All Touch panels and wiring is DIFFERENT
// copy-paste results from MCUFriend/TouchScreen_Calibr_native.ino

//AML 2022-09-17 Screen fitted to Arduino UNO SN: 7513330333235150F071
    const int XP=8,XM=A2,YP=A3,YM=9; //240x320 ID=0x9595
    const int TS_LEFT=910,TS_RT=107,TS_TOP=84,TS_BOT=905;

// Define a TouchScreen objectl resisance measured at 351 Ohms, (default value of 300 works OK)
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 351);

// Touch buttons to display on the screen
// Main menu buttons
Adafruit_GFX_Button run_btn, baud_btn, tune_btn, obs_btn, clock_btn;

//Sub menu buttons
Adafruit_GFX_Button up_btn, down_btn, sel_btn;

// location of touch points
int pixel_x, pixel_y;     //Touch_getXY() updates global vars

// RTC time constructor
DFRobot_DS1307 DS1307;
uint16_t currentTime[7] = {0};

//Observatory specific parameters 
int baudSelect[5] = {300, 1200, 4800, 9600, 19200};

// If these default values are altered during run-time
// they are are stored in EEPROM and the updated values used
// for subsequent runs after reset/power cycle
int tune = 50;
int repeats = 8;
int baudIndex = 3;  // default baudrate = 9600
int i = 0;

// GSM90F_timeout must be at least 3.5 seconds; set the sum of the two GSM90* values to give desired sampling rate
// trial and error provided the values below for a 10 s sampling rate
// Increase GSM90F_timeout to 4000.  3500 times-out for "a" quality readings with older-style magnetometers (it works OK for newer mags) 2023-12-05
// Decrease GSM90_sampling_delay from 3500 to 3000 as a compromise between older-style and newer-style instruments to maintain close to 10 second samples
int GSM90F_sampling_delay = 3000, GSM90F_timeout = 4000, short_delay = 40;

//////////////////////////////////////////////////////////////////////////////////////////////
// get touch locations (AdaFruit)
bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
    }
    return pressed;
}
////////////////////////////////////////////////////////////////
// Read obs parameters from EEPROM if they are available
// address 0 = # (ascii 22)  indicates tune value in EEPROM(1) is valid data
//         1 = tune value (20 - 90)
//         2 = # indicates repeats value in EEPROM(3) is valid data
//         3 = repeats value (1 - 100)
//         4 = # indicates baud rate index in EEPROM(5) is valid data
//         5 = baud rate index (0 - 4) 
void readEEPROM() {
  byte eepromOK = 45; 
  byte eepromValue;
  eepromOK = EEPROM.read(0);
  if (eepromOK == 22) {
    eepromValue = EEPROM.read(1);
    tune = int(eepromValue);
    eepromOK = 45;
  }
  eepromOK = EEPROM.read(2);
  if (eepromOK == 22) {
    eepromValue = EEPROM.read(3);
    repeats =int(eepromValue);
    eepromOK = 45;
  }
  eepromOK = EEPROM.read(4);
  if (eepromOK == 22) {
    eepromValue = EEPROM.read(5);
    baudIndex = int(eepromValue);
    eepromOK = 45;
  }
}
//////////////////////////////////////////////////////////////
// Display the main menu selection buttons
void main_menu(void) {
  tft.fillScreen(BLUE);
  run_btn.drawButton(false);
  clock_btn.drawButton(false);
  baud_btn.drawButton(false);
  tune_btn.drawButton(false);
  obs_btn.drawButton(false);
}
///////////////////////////////////////////////////////////////
// Display sub menu selection buttons  
void sub_menu(void) {
  tft.fillScreen(CYAN);
  sel_btn.drawButton(false);
  up_btn.drawButton(false);
  down_btn.drawButton(false);
  tft.drawLine(0,205,320,205,BLACK);
  tft.drawLine(0,207,320,207,BLACK);
}  
//////////////////////////////////////////////////////////////
// Adjust serial comms baud rate from menu selection
void baudrate() {
  byte eepromValue;
  int baudIndex_old = 0;
  sub_menu();
  
// Roll-over for baud-rate array requires different treatment than other roll-overs to prevent index over-run
  while (1) {
// only update screen when value changes
    if (baudIndex_old != baudIndex) {
      tft.fillRect(00,250,240,70,CYAN);
      tft.setTextColor(BLACK);
      tft.setCursor(50, 250);  tft.print(F("Baud:"));
      tft.setCursor( 150,250); tft.print(baudSelect[baudIndex]);
      tft.setCursor( 80, 300); tft.print(F("n-8-1"));
      baudIndex_old = baudIndex;
    }
// check for screen press
    bool down = Touch_getXY();
    up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
    down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
    sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
    if (up_btn.justReleased())   up_btn.drawButton();
    if (down_btn.justReleased())   down_btn.drawButton();
    if (sel_btn.justReleased())   sel_btn.drawButton();
    
// Up Button pressed
    if ( up_btn.justPressed() ) {
        up_btn.drawButton(true);
        baudIndex++;
        if ( baudIndex > 4) baudIndex = 0;
    }      
// Down button pressed
     else if ( down_btn.justPressed() ) {
      down_btn.drawButton(true);
      baudIndex--;
      if ( baudIndex  < 0)  baudIndex = 4;
     }
//select button pushed  - save data to EEPROM
     else if ( sel_btn.justPressed() ) {
        sel_btn.drawButton(true);
        EEPROM.update(4,22);  
        eepromValue=byte(baudIndex);   
        EEPROM.update(5,eepromValue);   
// redraw the main menu
        main_menu();
        break;
     }
   delay(short_delay);
   }
}
//////////////////////////////////////////////////////////////
// Adjust the magnetometer tune value
void tuneMag() {
  byte eepromValue;
  int tune_old = 0; 
  sub_menu();
  
  while (1) {
    if ( tune < 20)  tune = 90;
    if ( tune > 90)  tune = 20;
// only update screen when value changes
    if (tune_old != tune) {
       tft.fillRect(00,250,240,70,CYAN);
       tft.setTextColor(BLACK);
       tft.setCursor(30, 250); tft.print(F("Tune:")); 
       tft.setCursor(150,250); tft.print(tune);
       tune_old = tune;
    }

// check for screen press
    bool down = Touch_getXY();
    up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
    down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
    sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
    if (up_btn.justReleased())   up_btn.drawButton();
    if (down_btn.justReleased())   down_btn.drawButton();
    if (sel_btn.justReleased())   sel_btn.drawButton();
    
// Up Button pressed
    if ( up_btn.justPressed() ) {
        up_btn.drawButton(true);
        tune++;
    }      
// Down button pressed
     else if ( down_btn.justPressed() ) {
      down_btn.drawButton(true);
      tune--;
     }
//select button pushed  - save data to EEPROM
     else if ( sel_btn.justPressed() ) {
        sel_btn.drawButton(true);
        EEPROM.update(0,22);  
        eepromValue=byte(tune);   
        EEPROM.update(1,eepromValue);   
// redraw the main menu
        main_menu();
        break;
     }
   delay(short_delay);
   }
}
//////////////////////////////////////////////////////////////
// Adjust the number of repeated observations
void obs() {
  byte eepromValue;
  int repeats_old = 0;
  sub_menu();
  
  while (1) {
    if ( repeats  < 1)  repeats = 100;
    if ( repeats  > 100)  repeats = 1;
// only update screen when value changes
    if (repeats_old != repeats) {
       tft.fillRect(00,250,240,70,CYAN);
       tft.setTextColor(BLACK);
       tft.setCursor(50, 250); tft.print(F("#Obs:"));
       tft.setCursor( 150,250);
       tft.print(repeats);
       repeats_old = repeats;
    }  

// check for screen press
    bool down = Touch_getXY();
    up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
    down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
    sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
    if (up_btn.justReleased())   up_btn.drawButton();
    if (down_btn.justReleased())   down_btn.drawButton();
    if (sel_btn.justReleased())   sel_btn.drawButton();
    
// Up Button pressed
    if ( up_btn.justPressed() ) {
        up_btn.drawButton(true);
        repeats++;
    }      
// Down button pressed
     else if ( down_btn.justPressed() ) {
      down_btn.drawButton(true);
      repeats--;
     }
//select button pushed  - save data to EEPROM
     else if ( sel_btn.justPressed() ) {
        sel_btn.drawButton(true);
        EEPROM.update(2,22);  
        eepromValue=byte(repeats);   
        EEPROM.update(3,eepromValue);   
// redraw the main menu
        main_menu();
        break;
     }
   delay(short_delay);
   }
}
//////////////////////////////////////////////////////////////////////////////
// Serial communications with GSM90 magnetometer - the main reason for this sketch
void comms() {
  int FLen,yloc,j,k;
  String TuneResponse, TotalField;
// serial port needs to be set up here in case baud rate has been changed from menu
// data bits, parity, stop bits cannot be changed interactively
  Serial.begin(baudSelect[baudIndex],SERIAL_8N1);
  Serial.setTimeout(1000);
// wait for serial port to connect.  
  while (!Serial) {
    delay(short_delay); 
  }
  
  tft.fillScreen(CYAN);
  tft.setTextSize(2);
  tft.setCursor(5,10); tft.print(F("T")); tft.print(tune);
  
// flush the serial buffer before tune command
  while (Serial.available() > 0) {
    k = Serial.read();
  }

// Initiate Tune command to the magnetometer 
  Serial.print(F("T")); Serial.println(tune,DEC);
  TuneResponse = Serial.readString();
  tft.setCursor(5,40);
  if ( TuneResponse.length() < 2) tft.print(F("Tuning timed out"));
  else  tft.print(TuneResponse);

// flush the serial buffer after tune command
  while (Serial.available() > 0) {
    k = Serial.read();
  }
  delay(4000);

// Initiate field readings
  Serial.setTimeout(GSM90F_timeout);
  for (i = 0; i < repeats; i++) {
// if there are more than 7 readings then clear the screen and re-start display from the top
    j = i % 7;
    if ( j == 0 ) tft.fillScreen(CYAN);
    yloc= 20 + j *40;
    tft.setTextSize(2);
    tft.setCursor(5,yloc);
    tft.print(i+1);  
    tft.setCursor(30,yloc);
    tft.print("F ");

// flush the serial buffer
  while (Serial.available() > 0) {
    k = Serial.read();
  }
// Send the command to the magnetometer
    Serial.print(F("F"));
    TotalField = Serial.readString();

// Get the time from the RTC
    DS1307.getTime(currentTime);

    FLen=TotalField.length();
    tft.setCursor(70,yloc);
    if ( FLen < 2) {
      tft.print(F("Field time-out"));
      tft.setCursor(70,yloc+18); print_time(currentTime,0);
    }
    else {
// dont display EOL chars at end of valid field readings    
      tft.print(TotalField.substring(0,FLen-1));
      tft.setCursor(70,yloc+18); print_time(currentTime,0);
    }
    delay(GSM90F_sampling_delay);
  
// flush the serial buffer after "F" command
    while (Serial.available() > 0) {
      k = Serial.read();
    }
  }
  tft.drawLine(0,yloc+32,320,yloc+32,BLACK);
  tft.drawLine(0,yloc+34,320,yloc+34,BLACK);
  delay(GSM90F_sampling_delay);
  delay(GSM90F_sampling_delay);
  main_menu();
}
///////////////////////////////////////////////////////////////////////////////////
// Set the time on the Real-time clock
// Global variable "currentTime" is unsigned int array, so values cannot go below 0.
// Hence cycling over 0 - 23/0 - 59 requires special treatment and use of a signed int variable
// hour, min, sec
void RTclock() {

  int hour = 0, min = 0, sec = 0;
  int year_old = 0, month_old = 0, day_old=0, hour_old = 0, min_old = 0, sec_old = 0;
// Get the current time  
  DS1307.getTime(currentTime);
// If clock not started then year = 2000 - There maybe a function to check clock status  
  if (currentTime[6] <= 2000) currentTime[6] = 2024;
  sub_menu();
   while (1) {
// First set the year    
      if ( currentTime[6]  < 2023)  currentTime[6] = 2050;
      if ( currentTime[6]  > 2050)  currentTime[6] = 2023;
// only update the screen if the value changes.      
      if (year_old != currentTime[6]) {
        tft.fillRect(00,250,240,70,CYAN);
        tft.setTextColor(BLACK);
        tft.setCursor(50, 250); tft.print(F("Year:"));
        tft.setCursor( 150,250); tft.print(currentTime[6]);
        tft.setTextSize(2); tft.setCursor(00, 300); print_time(currentTime,1);
        tft.setTextSize(3);
        year_old = currentTime[6];
      }
// check for screen press
      bool down = Touch_getXY();
      up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
      down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
      sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
      if (up_btn.justReleased())   up_btn.drawButton();
      if (down_btn.justReleased()) down_btn.drawButton();
      if (sel_btn.justReleased())  sel_btn.drawButton();
// Up Button pressed
      if ( up_btn.justPressed() ) {
        up_btn.drawButton(true);
        currentTime[6]++;
     }      
// Down button pressed
     else if ( down_btn.justPressed() ) {
      down_btn.drawButton(true);
      currentTime[6]--;
     }
//select button pushed
     else if ( sel_btn.justPressed() ) {
        sel_btn.drawButton(true);
        break;
     }
     delay(short_delay);
    } 
// Now set the month
  while (1) {
      if ( currentTime[5]  < 1)  currentTime[5] = 12;
      if ( currentTime[5]  > 12) currentTime[5] = 1;
      if (month_old != currentTime[5]) {
         tft.fillRect(00,250,240,70,CYAN);
         tft.setTextColor(BLACK);
         tft.setCursor(50, 250); tft.print(F("Month:"));
         tft.setCursor( 150,250); tft.print(currentTime[5]);
         tft.setTextSize(2); tft.setCursor(00, 300); print_time(currentTime,1);
         tft.setTextSize(3);
         month_old = currentTime[5];
      }
// check for screen press
       bool down = Touch_getXY();
       up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
       down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
       sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
       if (up_btn.justReleased())   up_btn.drawButton();
       if (down_btn.justReleased()) down_btn.drawButton();
       if (sel_btn.justReleased())  sel_btn.drawButton();
    
// Up Button pressed
       if ( up_btn.justPressed() ) {
         up_btn.drawButton(true);
         currentTime[5]++;
        }      
// Down button pressed
        else if ( down_btn.justPressed() ) {
          down_btn.drawButton(true);
          currentTime[5]--;
        }
//select button pushed
        else if ( sel_btn.justPressed() ) {
          sel_btn.drawButton(true);
          break;
       }
     delay(short_delay);
   }
// Now set the day
  while (1) {
      if ( currentTime[4]  < 1)  currentTime[4] = 31;
      if ( currentTime[4]  > 31) currentTime[4] = 1;
      if (day_old != currentTime[4]) {
         tft.fillRect(00,250,240,70,CYAN);
         tft.setTextColor(BLACK);
         tft.setCursor(50, 250); tft.print(F("Day:"));
         tft.setCursor(150,250); tft.print(currentTime[4]);
         tft.setTextSize(2); tft.setCursor(00, 300); print_time(currentTime,1);
         tft.setTextSize(3);
         day_old = currentTime[4];
      }
// check for screen press
       bool down = Touch_getXY();
       up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
       down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
       sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
       if (up_btn.justReleased())   up_btn.drawButton();
       if (down_btn.justReleased()) down_btn.drawButton();
       if (sel_btn.justReleased())  sel_btn.drawButton();
    
// Up Button pressed
       if ( up_btn.justPressed() ) {
         up_btn.drawButton(true);
         currentTime[4]++;
        }      
// Down button pressed
        else if ( down_btn.justPressed() ) {
          down_btn.drawButton(true);
          currentTime[4]--;
        }
//select button pushed
        else if ( sel_btn.justPressed() ) {
          sel_btn.drawButton(true);
          break;
       }
     delay(short_delay);
   }

// Now set the hour
// caste unsigned int into signed int.
  hour = currentTime[2];
  while (1) {
      if ( hour < 0)  hour = 23;
      if ( hour > 23) hour = 0;
// refresh the screen only when the value changes      
      if (hour_old != hour) {
// hour will always be a positive int.        
         currentTime[2] = hour;
         tft.fillRect(00,250,240,70,CYAN);
         tft.setTextColor(BLACK);
         tft.setCursor(50, 250); tft.print(F("Hour:"));
         tft.setCursor( 150,250); tft.print(currentTime[2]);
         tft.setTextSize(2); tft.setCursor(00, 300); print_time(currentTime,1);
         tft.setTextSize(3);
         hour_old = hour;
      }   
// check for screen press
       bool down = Touch_getXY();
       up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
       down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
       sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
       if (up_btn.justReleased())   up_btn.drawButton();
       if (down_btn.justReleased()) down_btn.drawButton();
       if (sel_btn.justReleased())  sel_btn.drawButton();
    
// Up Button pressed
       if ( up_btn.justPressed() ) {
         up_btn.drawButton(true);
         hour++;
        }      
// Down button pressed - "hour" can go negative here
        else if ( down_btn.justPressed() ) {
          down_btn.drawButton(true);
          hour--;
        }
//select button pushed
        else if ( sel_btn.justPressed() ) {
          sel_btn.drawButton(true);
          break;
       }
     delay(short_delay);
   }
// Now set the minute
// caste unsigned int into signed int.
  min = currentTime[1];
  while (1) {
      if ( min  < 0)  min = 59;
      if ( min  > 59) min = 0;
      if (min_old != min) {
// min will always be a positive int here       
         currentTime[1] = min;        
         tft.fillRect(00,250,240,70,CYAN);
         tft.setTextColor(BLACK);
         tft.setCursor(50, 250); tft.print(F("Min:"));
         tft.setCursor( 150,250); tft.print(currentTime[1]);
         tft.setTextSize(2); tft.setCursor(00, 300); print_time(currentTime,1);
         tft.setTextSize(3);
         min_old = min;
      }
// check for screen press
       bool down = Touch_getXY();
       up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
       down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
       sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
       if (up_btn.justReleased())   up_btn.drawButton();
       if (down_btn.justReleased()) down_btn.drawButton();
       if (sel_btn.justReleased())  sel_btn.drawButton();
    
// Up Button pressed
       if ( up_btn.justPressed() ) {
         up_btn.drawButton(true);
         min++;
        }      
// Down button pressed; "min" can go negative here
        else if ( down_btn.justPressed() ) {
          down_btn.drawButton(true);
          min--;
        }
//select button pushed
        else if ( sel_btn.justPressed() ) {
          sel_btn.drawButton(true);
          break;
       }
     delay(short_delay);
   }
// Now set the Second
// caste unsigned int into signed int.
  sec = currentTime[0];
  while (1) {
      if ( sec  < 0)  sec = 59;
      if ( sec  > 59) sec = 0;
// only update screen if value changes      
      if (sec_old != sec) {
// sec will always be a positive int.        
         currentTime[0] = sec;        
         tft.fillRect(00,250,240,70,CYAN);
         tft.setTextColor(BLACK);
         tft.setCursor(50, 250); tft.print(F("Sec:"));
         tft.setCursor( 150,250); tft.print(currentTime[0]);
         tft.setTextSize(2); tft.setCursor(00, 300); print_time(currentTime,1);
         tft.setTextSize(3);
         sec_old = sec;
      }   
// check for screen press
       bool down = Touch_getXY();
       up_btn.press(down && up_btn.contains(pixel_x, pixel_y));
       down_btn.press(down && down_btn.contains(pixel_x, pixel_y));
       sel_btn.press(down && sel_btn.contains(pixel_x, pixel_y));
    
       if (up_btn.justReleased())   up_btn.drawButton();
       if (down_btn.justReleased()) down_btn.drawButton();
       if (sel_btn.justReleased())  sel_btn.drawButton();
    
// Up Button pressed
       if ( up_btn.justPressed() ) {
         up_btn.drawButton(true);
         sec++;
        }      
// Down button pressed; sec can go negative here.
        else if ( down_btn.justPressed() ) {
          down_btn.drawButton(true);
          sec--;
        }
//select button pushed
        else if ( sel_btn.justPressed() ) {
          sel_btn.drawButton(true);
// Finally, now set the time on the RT clock
           DS1307.setTime(currentTime);
           break;
       }
     delay(short_delay);
   }
  main_menu();
}
////////////////////////////////////////////////////////////////////////////////
// Display date and/or time on tft screen
// print the year if flag = 1, otherwise dont print the year

void print_time(uint16_t time[7], bool flag) {
  if (flag) {
     tft.print(time[6]);tft.print(F("-"));
     if ( time[5] < 10 ) tft.print(F("0")); tft.print(time[5]); tft.print("-");
     if ( time[4] < 10 ) tft.print(F("0")); tft.print(time[4]); tft.print("T");
  }    
  if (time[2] < 10 )  tft.print(F("0")); tft.print(time[2]);  tft.print(":");
  if (time[1] < 10 )  tft.print(F("0")); tft.print(time[1]);  tft.print(":");
  if (time[0] < 10 )  tft.print(F("0")); tft.print(time[0]);
}
/////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
    Serial.begin(9600);
// start up the RT clock before doing anything with the screen
   while( !(DS1307.begin()) ){
    Serial.println("Communication with RTC failed, please check connection");
    delay(3000);
  }
// read TFT screen ID
  uint16_t ID = tft.readID();
// initialise the TFT screen object    
    tft.begin(ID);
    tft.setRotation(0);            // PORTRAIT
    
//  position(X,Y) , shape(X,Y) , colour(EDGE, BODY, FONT),  text, text size 
// "position" is the CENTRE of the button rectangle
     run_btn.initButton(&tft, 120, 80 , 150, 60, WHITE, GREEN, BLACK, "Run", 3);
    baud_btn.initButton(&tft,  60, 220, 100, 40, WHITE, CYAN, BLACK, "Baud",3);
    tune_btn.initButton(&tft, 180, 220, 100, 40, WHITE, CYAN, BLACK, "Tune",3);
     obs_btn.initButton(&tft, 120, 280, 100, 40, WHITE, CYAN, BLACK, "# Obs", 3);
   clock_btn.initButton(&tft, 120, 160, 100, 40, WHITE, CYAN, BLACK, "Clock", 3);

      up_btn.initButton(&tft, 60, 180, 100, 40, WHITE, YELLOW, BLACK, "Up",3);
    down_btn.initButton(&tft,180, 180, 100, 40, WHITE, YELLOW, BLACK, "Down",3);
     sel_btn.initButton(&tft,120,  80, 150, 60, WHITE, GREEN, BLACK, "Select", 3);

// Read obs data from EEPROM
  readEEPROM();

  tft.fillScreen(WHITE);   
  tft.setTextColor(BLACK);  
  tft.setTextSize(2);
  tft.setCursor(30, 30); tft.print(F("Screen ID 0x")); tft.println(ID, HEX);
  tft.setTextSize(3);
  tft.setCursor(30,50 ); tft.print(F("GA GEOMAG"));
  tft.setTextSize(2);
  tft.setCursor(30, 90); tft.print(F("GSM90 Interface"));
  tft.setCursor(30, 120);
  tft.print(F("Version ")); tft.println(_VERSION);    
  tft.setCursor(30, 150); tft.print(F("Tune: ")); tft.println(tune);
  tft.setCursor(30, 180); tft.print(F("Baud: ")); tft.println(baudSelect[baudIndex]);
  tft.setCursor(30, 210); tft.print(F("# Obs: ")); tft.println(repeats);
  delay(5000);
  main_menu();
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void loop(void)
{
// down = true if touch screen is presssed and returns the X, Y co-ordinates into global variables pixel_x, pixel_y 
    bool down = Touch_getXY();
    run_btn.press(down && run_btn.contains(pixel_x, pixel_y));
    baud_btn.press(down && baud_btn.contains(pixel_x, pixel_y));
    tune_btn.press(down && tune_btn.contains(pixel_x, pixel_y));
    obs_btn.press(down && obs_btn.contains(pixel_x, pixel_y));    
    clock_btn.press(down && clock_btn.contains(pixel_x, pixel_y));    
    if (run_btn.justReleased())   run_btn.drawButton();
    if (baud_btn.justReleased())  baud_btn.drawButton();
    if (tune_btn.justReleased())  tune_btn.drawButton();
    if (obs_btn.justReleased())   obs_btn.drawButton();
    if (clock_btn.justReleased()) clock_btn.drawButton();

    if (run_btn.justPressed())   comms();        
    if (baud_btn.justPressed())  baudrate();
    if (tune_btn.justPressed())  tuneMag();
    if (obs_btn.justPressed())   obs();
    if (clock_btn.justPressed()) RTclock();

// Display date and time at top of screen; the 250 mS delay
// slows response to "touch" buttons, but keeps
// time display "strobing" to an acceptable level
  tft.setTextSize(2);
  tft.setCursor(10, 5);
  tft.setTextColor(BLACK);
  DS1307.getTime(currentTime);
  print_time(currentTime,1);
  delay(250);
// Erase the time ready for overprint
  tft.setCursor(10, 5);
  tft.setTextColor(BLUE);
  print_time(currentTime,1);
}
 