// Current Version
#define VERSION "ArduiPRS v1.1 "
// config file to defines callsign, ssid, symbols and home location
//#define MYCALL "NOCALL"

// Needed this to prevent compile error for #defines
#if 1
__asm volatile ("nop");
#endif

#ifndef _CONFIGURATION_INCLUDED
#define _CONFIGURATION_INCLUDED
#include "config.h"
#endif

// GPS libraries
// Choose between GPS and BD
//#include <TinyGPS++BD.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;
// The packet decoding libs
#include <MicroAPRS.h>

MicroAPRS microaprs = MicroAPRS(&Serial);
// APRS Buffers
#define BUFLEN (260) //original 260
char packet[BUFLEN];
int buflen = 0;
bool showmsg, showstation;

float latitude = 0.0;
float longitude = 0.0;
float wayPointLatitude, wayPointLongitude;
float latitudeRadians, wayPointLatitudeRadians, longitudeRadians, wayPointLongitudeRadians;
float distanceToWaypoint, bearing, deltaLatitudeRadians, deltaLongitudeRadians;
const float pi = 3.14159265;
const int radiusOfEarth = 6371; // in km
int uur;
int minuut;
int seconde;
int headingDegrees;
int hoogte;
String hoogte2;
String CALL_SSID = "9";
// Refer to http://wa8lmf.net/aprs/APRS_symbols.htm
char SYMBOL_CHAR = '(';         // Sybol
char SYMBOL_TABLE = 's';        // Primary Table (s) or Alternate Table (a)

// Turn on/off debug, on by default on pin 2,3
#undef DEBUG
//#define DEBUG

// Turn on/off 16x2 I2C LCD
#define I2C16X2

// ifdef for I2C LCD
#ifdef I2C16X2
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif

// AltSoftSerial default on UNO is 8,9 (Rx,Tx)
#if defined (__AVR_ATmega328P__)
#include <AltSoftSerial.h>
AltSoftSerial ss(8, 9);
#else
// Map hw Serial2 to ss for gps port for other platform with hw serial
#define ss Serial2
#endif


#ifdef DEBUG
// Connect to GPS module on pin 9, 10 ( Rx, Tx )
#if defined (__AVR_ATmega328P__)
SoftwareSerial debug(2, 3);
#elif defined(__arm__) && defined(TEENSYDUINO)
#define debug Serial3
#endif
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Put All global defines here
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

const byte modePin = 3;
const byte buzzerPin = 4;
const byte ledPin = 6;

// Varables for Packet Decode
const unsigned int MAX_INPUT = 103;
static unsigned int packetDecoded = 0;

char *lastCall = "";
String rxCallsign = "";
unsigned int rxStation;

unsigned int mCounter = 0;
unsigned int txCounter = 0;
unsigned long txTimer = 0;
#ifdef I2C16X2
bool packetDisplay = 0;
unsigned long displayTime = 0;
#endif
unsigned long lastTx = 0;
unsigned long lastRx = 0;
unsigned long txInterval = 80000L;  // Initial 80 secs internal

int lastCourse = 0;
byte lastSpeed = 0;
byte buttonPressed = 0;

// Unused
//static unsigned int Hd,Ti,Di,Bn = 0;

int previousHeading, currentHeading = 0;
// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = HOME_LAT;
float lastTxLng = HOME_LON;
float lastTxdistance, homeDistance, base = 0.0;

// Used in the future for sending messages, commands to the tracker
const unsigned int MAX_DEBUG_INPUT = 30;


#ifdef I2C16X2
/// LCD Large fonts
// the 8 arrays that form each segment of the custom numbers

/*
  byte LT[8] =
  {
  B00011,
  B00111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111
  };*/

byte UB[8] =
{
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

byte RT[8] =
{
  B11100,
  B11110,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

byte LL[8] =
{
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B00111,
  B00011
};

byte LB[8] =
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111
};
/*
  byte LR[8] =
  {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11110,
  B11100
  };
*/
byte UMB[8] =
{
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111
};
byte LMB[8] =
{
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111
};

int x = 0;

/// LCD Large fonts
#endif

//////////////////////////////////////////////////////////////////////////////
// setup()
//////////////////////////////////////////////////////////////////////////////

void setup()
{
if (digitalRead(modePin) == HIGH) {CALL_SSID = "5"; SYMBOL_CHAR = '[';}

#ifdef I2C16X2
  lcd.begin(16, 2);
  lcd.createChar(1, RT);
  lcd.createChar(2, UB);
  //  lcd.createChar(3,LL);
  lcd.createChar(4, LB);
  //  lcd.createChar(5,LR);
  lcd.createChar(6, UMB);
  lcd.createChar(0, LMB);
  //  lcd.createChar(8,LT);

  lcd.backlight();
  lcd.begin(16, 2);
  lcd.print(VERSION);
  lcd.setCursor(0, 1);
  lcd.print("PD1DDK");
  if (CALL_SSID == "5"){ lcd.print(" HIKING");}
  else {lcd.print(" CAR");}
  delay(1000);
#endif

#if defined(__arm__) && defined(TEENSYDUINO)
  // This is for reading the internal reference voltage
  analogReference(EXTERNAL);
  analogReadResolution(12);
  analogReadAveraging(32);
#endif

  //Modeswitch
  pinMode(modePin, INPUT);
  // Buzzer uses pin 4
  pinMode(buzzerPin, OUTPUT);

  // LED pin on 13, only enable for non-SPI TFT
  pinMode(ledPin, OUTPUT);

  // Main serial talks to the MicroModem directly
  Serial.begin(9600);

  // ss talks to the GPS receiver at 9600
  ss.begin(9600);

#ifdef DEBUG
  debug.begin(9600);
#endif

#ifdef DEBUG
  debug.flush();
  debug.println();
  debug.print(F("DEBUG:- "));
  debug.println(F(VERSION));
  debug.println();
#endif

  // Set a delay for the MicroAPRS to boot up before configuring it
  delay(1000);
  configModem();
  Serial.flush();
  txTimer = millis();
      
} // end setup()

//////////////////////////////////////////////////////////////////////////////
// loop()
//////////////////////////////////////////////////////////////////////////////

void loop()
{
  // Speed in km/h
  const byte highSpeed = 80;       // High speed
  const byte lowSpeed = 30;        // Low speed
  char c;
  boolean inputComplete = false;
  int headingDelta = 0;

#ifdef DEBUG
  // Send commands from debug serial into hw Serial char by char

#if defined (__AVR_ATmega328P__)
  debug.listen();
#endif
#endif

  // Turn on listen() on GPS
#if defined (__AVR_ATmega328P__)
  ss.listen();
#endif
  while ( ss.available() > 0 ) {
    gps.encode(ss.read());
  }

  while (Serial.available() > 0)
  {
    char ch = microaprs.read();
    if (ch == '\n')
    {
      packet[buflen] = 0;
      show_packet();
      buflen = 0;
    }
    else if ((ch > 31 || ch == 0x1c || ch == 0x1d || ch == 0x27) && buflen < BUFLEN)
    {
      // Mic-E uses some non-printing characters
      packet[buflen++] = ch;
    }
  }

  ///////////////// Triggered by location updates ///////////////////////
  if ( gps.location.isUpdated() ) {

    homeDistance = TinyGPSPlus::distanceBetween(
                     gps.location.lat(),
                     gps.location.lng(),
                     HOME_LAT,
                     HOME_LON);

    lastTxdistance = TinyGPSPlus::distanceBetween(
                       gps.location.lat(),
                       gps.location.lng(),
                       lastTxLat,
                       lastTxLng);

    base = TinyGPSPlus::distanceBetween(
             gps.location.lat(),
             gps.location.lng(),
             HOME_LAT,
             HOME_LON) / 1000;

    latitude = gps.location.lat();
    longitude = gps.location.lng();

    // Get headings and heading delta
    currentHeading = (int) gps.course.deg();
    if ( currentHeading >= 180 ) {
      currentHeading = currentHeading - 180;
    }
    headingDelta = (int) ( previousHeading - currentHeading ) % 360;

  } // endof gps.location.isUpdated()

  ///////////////// Triggered by time updates ///////////////////////
  // Update LCD every second

  if ( gps.time.isUpdated() ) {

    if ( gps.satellites.value() > 3 ) {
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }

    if ( gps.time.second() % 10 == 0 ) {

    } else if ( gps.time.second() % 5 == 0 ) {

    }

    // I2C LCD addons
#ifdef I2C16X2
    if ( packetDisplay ) {
      // Do not overwrite the decoded packet
      // display packetDecoded for 10 secs
      if ( millis() - displayTime > 10000 ) {
        packetDisplay = 0;
      }
    } else if ( gps.time.second() > 30 && gps.time.second() < 36 ) {
      lcdScreen3(); // Screen1 is speed / sats
    } else if ( gps.time.second() > 50 && gps.time.second() < 56 ) {
      lcdScreen2(); // Screen2 is Altitide / bearing / distBase / Uptime / Rx / Tx / Msgs
    } else {
      lcdScreen1(); // Screen1 is speed / sats
    }
#endif

    // Change the Tx internal based on the current speed
    // This change will not affect the countdown timer
    // Based on HamHUB Smart Beaconing(tm) algorithm

    if ( gps.speed.kmph() < 5 ) {
      txInterval = 300000;         // Change Tx internal to 5 mins
    } else if ( gps.speed.kmph() < lowSpeed ) {
      txInterval = 70000;          // Change Tx interval to 60
    } else if ( gps.speed.kmph() > highSpeed ) {
      txInterval = 30000;          // Change Tx interval to 30 secs
    } else {
      // Interval inbetween low and high speed
      txInterval = (highSpeed / gps.speed.kmph()) * 30000;
    } // endif

  }  // endof gps.time.isUpdated()

  ////////////////////////////////////////////////////////////////////////////////////
  // Check for when to Tx packet
  ////////////////////////////////////////////////////////////////////////////////////

  lastTx = millis() - txTimer;

  // Only check the below if locked satellites < 3

  if ( gps.satellites.value() > 3 ) {
    if ( lastTx > 5000 ) {
      // Check for heading more than 25 degrees
      if ( (headingDelta < -25 || headingDelta >  25) && lastTxdistance > 5 ) {
        if (TxtoRadio(1)) {
          lastTxdistance = 0;   // Ensure this value is zero before the next Tx
          previousHeading = currentHeading;
        }
      } // endif headingDelta
    } // endif lastTx > 5000

    if ( lastTx > 10000 ) {
      // check of the last Tx distance is more than 600m
      if ( lastTxdistance > 600 ) {
        if ( TxtoRadio(2) ) {
          lastTxdistance = 0;   // Ensure this value is zero before the next Tx
        }
      } // endif lastTxdistance
    } // endif lastTx > 10000

    if ( lastTx >= txInterval ) {
      // Trigger Tx Tracker when Tx interval is reach
      // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
      if ( lastTxdistance > 20 ) {
        TxtoRadio(3);
      } // endif lastTxdistance > 20
    } // endif of check for lastTx > txInterval

    // Force a tx when 3 unique stations was received
    // Or 60 secs after last received station to prevent stale info on the distance
    if (  (rxStation > 2) || (( (rxStation > 0) && ( ((millis() - lastRx) / 1000) > 60) )) )  {
      TxtoRadio(4);
    }
  } // Endif check for satellites

  // Check if the analog0 is plugged into 5V and more than 10 secs

  // rev13 PCB will change to pullup circuit and read using digitalRead() instead of analogRead

  if ( analogRead(0) > 700 && (lastTx > 10000) ) {
    buttonPressed = 1;
    TxtoRadio(5);
  } // endif check analog0


} // end loop()

// New MicroAPRS function
void show_packet()
{
  char *posit, *pmsgTo, *call, *pcomment, *pmsg;
  char type, pmsgID;
  long lat, lon;

  // Only displasy if decode is true
  if ( microaprs.decode_posit(packet, &call, &type, &posit, &lon, &lat, &pcomment, &pmsgTo, &pmsg, &pmsgID) ) {

    if (type == 58) // 58 = "!" = Message
    {
      if (startsWith(MYCALL, pmsgTo))
      {
        mCounter++;

        // Beep 3 times
        beep(3);
      }
    }
    else // Not message, decode , calculate and display packets
    {
      wayPointLatitude = lat;
      wayPointLongitude = lon;

      wayPointLatitude = wayPointLatitude / 1000000;
      wayPointLongitude = wayPointLongitude / 1000000;

      distanceToWaypoint = calculateDistance();
      bearing = calculateBearing();

      // Beep twice is station is less than 500m
      if ( distanceToWaypoint < 0.5 ) {
        beep(2);
      }

      // Check for valid decoded packets
      if ( strlen(call) < 12 ) {
        lastRx = millis();
        packetDecoded++;

        // Append rx callsign into rxCallsign strings
        // Do not add own callsign
        if ( !startsWith(MYCALL, call) ) {
          // Do not add duplicated callsign from digipeater packets
          if ( !rxCallsign.startsWith(call) ) {
            rxCallsign.concat(" ");
            rxCallsign.concat(call);
            lastCall = call;
            // Only send distance if GPS is locked AND less than 500km away
            if ( gps.satellites.value() > 3 && distanceToWaypoint < 500 ) {
              rxCallsign.concat("/");
              rxCallsign.concat(distanceToWaypoint);
              rxCallsign.concat("km");
            }
            rxStation++;
          }
        }

#ifdef I2C16X2
        packetDisplay = 1;
        displayTime = millis();
        lcdScreen4(call, pcomment);
#endif

      } // endif check for valid packets
    }
  } // endif microaprs.decode_posit()
}


#ifdef I2C16X2
void lcdScreen1() {
  lcd.clear();
  displayLargeSpeed((unsigned int)gps.speed.kmph());
}

void lcdScreen2() {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("B:");
  lcd.print(base, 0);    // Max 999 afstanf tot thuis!

  lcd.setCursor(6, 0);
  lcd.print("T:");
  lcd.print(txCounter);  // Max 999

  // Max 2 digits10w

  lcd.setCursor(12, 0);
  lcd.print("R:");
  lcd.print(packetDecoded);  // Max 99


  lcd.setCursor(0,1);
  lcd.write("M:");
  lcd.print(mCounter);    // Messages received

  //lcd.setCursor(0,1);
  //lcd.write("C:");
  //lcd.write((unsigned int)gps.course.deg());

  //lcd.setCursor(0, 1);
  //lcd.print("U:");
  //lcd.print((float) millis() / 60000, 0); // Max 999

  lcd.setCursor(6, 1);
  lcd.print("A:");
  lcd.print((unsigned int)gps.altitude.meters()); // Max 9999
  
  //lcd.setCursor(12,1);
  //lcd.write("L:");
  //lcd.write((unsigned int)(millis()-lastRx)/1000);  // Print the seconds since last Rx

}

void lcdScreen3() {
  lcd.clear();
  lcd.setCursor(0, 0);

  // Convert to GMT+1
  byte hour = gps.time.hour() + 1;

  if ( hour > 23 ) {
    hour = 0;
  }
  if ( hour < 10 ) {
    lcd.write("0");
  }
  lcd.print(hour);
  lcd.print(":");
  if ( gps.time.minute() < 10 ) {
    lcd.print("0");
  }
  lcd.print(gps.time.minute());
  lcd.print(":");
  if ( gps.time.second() < 10 ) {
    lcd.print("0");
  }
  lcd.print(gps.time.second());


  lcd.print(" H:");
  lcd.print(gps.hdop.value());

  lcd.setCursor(0, 1);
  lcd.print(gps.location.lat(), 4);
  lcd.setCursor(8, 1);
  lcd.print(gps.location.lng(), 4);
}

void lcdScreen4(char *callsign, char *comment ) {

  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print(callsign);
  if (distanceToWaypoint < 500 ) {
    lcd.print(" ");
    lcd.print(distanceToWaypoint, 1);
    lcd.print("km");
  }
  lcd.setCursor(0, 1);
  comment[16] = 0;
  lcd.print(comment);
}

#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////

boolean TxtoRadio(int type) {

  char tmp[10];
  float latDegMin, lngDegMin = 0.0;
  String latOut, lngOut, cmtOut = "";
  unsigned int Mem = freeRam();
  float Volt = (float) readVcc() / 1000;

  lastTxLat = gps.location.lat();
  lastTxLng = gps.location.lng();
  
  if ( lastTx > 6000 ) { // This prevent ANY condition to Tx below 6 secs

    latDegMin = convertDegMin(lastTxLat);
    lngDegMin = convertDegMin(lastTxLng);

    // Convert Lat float to string
    dtostrf(fabs(latDegMin), 2, 2, tmp );
    cmtOut.concat("!@");      // set latitute command
    
    uur = (gps.time.hour());
    if (uur < 10) {
       cmtOut.concat("0");
       cmtOut.concat(uur);
    }
      else if (uur >= 10) {
         cmtOut.concat(uur);
       }

    minuut = (gps.time.minute());
    if (minuut < 10) {
       cmtOut.concat("0");
       cmtOut.concat(minuut);
    }
      else if (minuut >= 10) {
         cmtOut.concat(minuut);
       }

    seconde = (gps.time.second());
    if (seconde < 10) {
       cmtOut.concat("0");
       cmtOut.concat(seconde);
    }
      else if (seconde >= 10) {
         cmtOut.concat(seconde);
       }
  
    cmtOut.concat("h");

    
    // Append 0 if Lat less than 10
    if ( fabs(lastTxLat) < 10 ) {
      cmtOut.concat("/");
    }

    cmtOut.concat(tmp);      // Actual Lat in DDMM.MM

    // Determine E or W
    if (latDegMin >= 0) {
      cmtOut.concat("N");
    } else if (latDegMin < 0) {
      cmtOut.concat("S");
    }

    // Convert Lng float to string
    dtostrf(fabs(lngDegMin), 2, 2, tmp );
    cmtOut.concat("/");       // set longtitute command

    // Append 0 if Lng less than 100
    if ( ( fabs(lastTxLng) < 100) ) {
      cmtOut.concat("0");       // set longtitute command
    }

    // Append 0 if Lng less than 10
    if ( fabs(lastTxLng) < 10 ) {
      cmtOut.concat("0");      // Append 0 if Lng less than 10
    }

    cmtOut.concat(tmp);      // Actual Lng in DDDMM.MM

    // Determine E or W
    if (lngDegMin >= 0) {
      cmtOut.concat("E");
    } else if (latDegMin < 0) {
      cmtOut.concat("W");
    }
    
    cmtOut.concat("(");
    cmtOut.concat(padding((int) gps.course.deg(), 3));
    cmtOut.concat("/");
    cmtOut.concat(padding((int)gps.speed.mph(), 3));

    cmtOut.concat("/A=");
    hoogte = gps.altitude.feet();
    if (hoogte < 0) {
    hoogte2 = (padding((int) abs(hoogte), 5));
    cmtOut.concat("-");
    cmtOut.concat(hoogte2);  
      }
      else {
      hoogte2 = (padding((int) abs(hoogte), 6));
      cmtOut.concat(hoogte2);
            }  
    
    //cmtOut.concat(padding((int)gps.altitude.feet(), 6)); origineel hoogte
            
    cmtOut.concat(" Seq:");
    cmtOut.concat(txCounter);

    if ( rxCallsign.length() > 0 ) {
      cmtOut.concat(rxCallsign);     // Send out all the Rx callsign & dist
    }

    // Send out the type of Tx trigger
    switch (type) {
      case 1:   cmtOut.concat(" H"); break;
      case 2:   cmtOut.concat(" D"); break;
      case 3:   cmtOut.concat(" T"); break;
      case 4:   cmtOut.concat(" R"); break;
      case 5:   cmtOut.concat(" B"); break;
      default: break;
    }

    cmtOut.concat(" ");
    cmtOut.concat(COMMENT);



    // This condition is ONLY for button pressed ( do not sent out position if not locked )
    if ( gps.satellites.value() > 3 ) {
      //Serial.println(latOut);
      //delay(200);
      digitalWrite(buzzerPin, HIGH);
      //Serial.println(lngOut);
      delay(200);
      Serial.println(cmtOut);
      digitalWrite(buzzerPin, LOW);
      delay(50);

      // Clear the rxCallsign contents & counters
      rxCallsign = "";
      rxStation = 0;
      lastRx = 0;
    }

    // Only send status/version every 10 packets to save packet size
    // Sample status display
    // >SVTrackR v1.5a 5.14V S:10 B:0 U:135m Seq:20


    if ( ( txCounter % 10 == 0 ) || buttonPressed ) {

      digitalWrite(buzzerPin, HIGH); // Turn on buzzer
      cmtOut = "";
      cmtOut.concat("!>");
      cmtOut.concat(VERSION);
      cmtOut.concat(Volt);
      cmtOut.concat("V S:");
      cmtOut.concat(gps.satellites.value());
      cmtOut.concat(" B:");
      if ( gps.satellites.value() > 3 ) {
        cmtOut.concat((unsigned int)base);
      } else {
        cmtOut.concat("0");
      }
      // R - number of packets decoded
      cmtOut.concat(" R:");
      cmtOut.concat(packetDecoded);

      cmtOut.concat(" U:");
      cmtOut.concat(millis() / 60000);
      cmtOut.concat("m Seq:");
      cmtOut.concat(txCounter);

      delay(200);
      digitalWrite(buzzerPin, LOW);  // Turn off buzzer

      smartDelay(5000);
      Serial.println(cmtOut);
    } //endof txCounter / buttonPressed


    // Reset all tx timer & Tx variables
    txInterval = 80000;
    buttonPressed = 0;
    txCounter++;
    txTimer = millis();
    lastTx = 0;

    // Tx success, return TRUE
    return 1;
  } else {
    return 0;
  }// endif lastTX > 6000


} // endof TxtoRadio()

///////////////////////////////////////////////////////////////////////////////////////////////////////


void configModem() {
  // Functions to configure the callsign, ssid, path and other settings

#ifdef I2C16X2
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Config:");
  lcd.setCursor(0, 1);
  lcd.print(MYCALL);
  lcd.print("-");
  lcd.print(CALL_SSID);
#endif


  digitalWrite(ledPin, HIGH);
  Serial.println("1WIDE1");  // Set PATH1 callsign
  delay(100);

  digitalWrite(ledPin, LOW);
  Serial.println("2WIDE2");  // Set PATH2 callsign
  delay(100);

  digitalWrite(ledPin, HIGH);
  Serial.println("dAPZSVT");  // Set DST Callsign to APRSVTH
  delay(100);

  digitalWrite(ledPin, LOW);
  Serial.print("c");          // Set SRC Callsign
  Serial.println(MYCALL);     // Set SRC Callsign
  delay(100);

  digitalWrite(ledPin, HIGH);
  Serial.print("sc");         // Set SRC SSID
  Serial.println(CALL_SSID);      // Set SRC SSID
  delay(100);

  digitalWrite(ledPin, LOW);
  Serial.println("pd1");      // Ensable printing DST
  delay(100);

  digitalWrite(ledPin, HIGH);
  Serial.println("pp0");      // Disable printing PATH
  delay(100);

  digitalWrite(ledPin, LOW);
  Serial.print("ls");      // Set symbol n / Bluedot
  Serial.println(SYMBOL_CHAR);      // Set symbol n / Bluedot
  delay(100);

  digitalWrite(buzzerPin, HIGH); // Turn on buzzer
  digitalWrite(ledPin, HIGH);
  Serial.print("lt");      // Standard symbol
  Serial.println(SYMBOL_TABLE);      // Standard symbol
  delay(100);

  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);
  Serial.println("V1");
  delay(100);

#ifdef I2C16X2
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Done...");
  lcd.setCursor(0, 1);
  lcd.print(MYCALL);
  lcd.print("-");
  lcd.print(CALL_SSID);
  delay(1000);
#endif


}

///////////////////////////////////////////////////////////////////////////////////////////////////////

float convertDegMin(float decDeg) {

  float DegMin;

  int intDeg = decDeg;
  decDeg -= intDeg;
  decDeg *= 60;
  DegMin = ( intDeg * 100 ) + decDeg;

  return DegMin;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

long readVcc() {
  long result;
#if defined(__arm__) && defined(TEENSYDUINO)
  extern "C" char* sbrk(int incr);
  result = 1195 * 4096 / analogRead(39);
#else
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);                     // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
#endif

  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

String padding( int number, byte width ) {
  String result;

  // Prevent a log10(0) = infinity
  int temp = number;
  if (!temp) {
    temp++;
  }

  for ( int i = 0; i < width - (log10(temp)) - 1; i++) {
    result.concat('0');
  }
  result.concat(number);
  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

int freeRam() {
#if defined(__arm__) && defined(TEENSYDUINO)
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
#else  // non ARM, this is AVR
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// convert degrees to radians
void radianConversion()
{
  deltaLatitudeRadians = (wayPointLatitude - latitude) * pi / 180;
  deltaLongitudeRadians = (wayPointLongitude - longitude) * pi / 180;
  latitudeRadians = latitude * pi / 180;
  wayPointLatitudeRadians = wayPointLatitude * pi / 180;
  longitudeRadians = longitude * pi / 180;
  wayPointLongitudeRadians = wayPointLongitude * pi / 180;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// calculate distance from present location to next way point
float calculateDistance()
{
  radianConversion();
  float a = sin(deltaLatitudeRadians / 2) * sin(deltaLatitudeRadians / 2) +
            sin(deltaLongitudeRadians / 2) * sin(deltaLongitudeRadians / 2) *
            cos(latitudeRadians) * cos(wayPointLatitudeRadians);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = radiusOfEarth * c;
  return d * 1;                  // distance in kilometers
  //  return d * 0.621371192;        // distance in miles
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// calculate bearing from present location to next way point
float calculateBearing()
{
  radianConversion();
  float y = sin(deltaLongitudeRadians) * cos(wayPointLatitudeRadians);
  float x = cos(latitudeRadians) * sin(wayPointLatitudeRadians) -
            sin(latitudeRadians) * cos(wayPointLatitudeRadians) * cos(deltaLongitudeRadians);
  bearing = atan2(y, x) / pi * 180;
  if (bearing < 0)
  {
    bearing = 360 + bearing;
  }
  return bearing;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

bool startsWith(const char *pre, const char *str)
{
  size_t lenpre = strlen(pre),
         lenstr = strlen(str);
  return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial.available() > 0)
    {
      char ch = microaprs.read();
      if (ch == '\n')
      {
        packet[buflen] = 0;
        show_packet();
        buflen = 0;
      }
      else if ((ch > 31 || ch == 0x1c || ch == 0x1d || ch == 0x27) && buflen < BUFLEN)
      {
        // Mic-E uses some non-printing characters
        packet[buflen++] = ch;
      }
    } // end of Serial.available while loop
  } while (millis() - start < ms);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void beep(int i) {
  while (i > 0) {
    digitalWrite(buzzerPin, HIGH); // Buzz the user
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
    i--;
  }
}

#ifdef I2C16X2
///////////////////////////////////////////////////////////////////////////////////////////////////////
// LCD Large fonts
void displayLargeSpeed(int num) {

  int digit4 =  num / 1000;
  int digit3 = (num % 1000) / 100;
  int digit2 = (num % 100) / 10;
  int digit1 =  num % 10;


  x = 0;
  switch (digit3) {

    case 0:
      // Do not display first zero
      //custom00();
      break;
    case 1:
      custom1();
      break;
    case 2:
      custom2();
      break;
    case 3:
      custom3();
      break;
    case 4:
      custom4();
      break;
    case 5:
      custom5();
      break;
    case 6:
      custom6();
      break;
    case 7:
      custom7();
      break;
    case 8:
      custom8();
      break;
    case 9:
      custom9();
      break;
    default:
      break;
  }

  x = 4;
  switch (digit2) {

    case 0:
      custom00();
      break;
    case 1:
      custom1();
      break;
    case 2:
      custom2();
      break;
    case 3:
      custom3();
      break;
    case 4:
      custom4();
      break;
    case 5:
      custom5();
      break;
    case 6:
      custom6();
      break;
    case 7:
      custom7();
      break;
    case 8:
      custom8();
      break;
    case 9:
      custom9();
      break;
    default:
      break;
  }

  x = 8;
  switch (digit1) {

    case 0:
      custom00();
      break;
    case 1:
      custom1();
      break;
    case 2:
      custom2();
      break;
    case 3:
      custom3();
      break;
    case 4:
      custom4();
      break;
    case 5:
      custom5();
      break;
    case 6:
      custom6();
      break;
    case 7:
      custom7();
      break;
    case 8:
      custom8();
      break;
    case 9:
      custom9();
      break;
    default:
      break;
  }

  if ( gps.satellites.value() < 10 ) {
    lcd.setCursor(14, 0);
    lcd.print(gps.satellites.value());
    lcd.print("s");
  } else {
    lcd.setCursor(13, 0);
    lcd.print(gps.satellites.value());
    lcd.print("s");
  }

  //lcd.write("S");

  // Print degree or N/S/E/W headings
  lcd.setCursor(13, 1);
  /*
  if ( gps.course.deg() < 10 ) {
    lcd.print("00");
    lcd.print(gps.course.deg(), 0);
    lcd.print("d");
  } else if ( gps.course.deg() < 100 ) {
    lcd.print("0");
    lcd.print(gps.course.deg(), 0);
    lcd.print("d");
  } else {
    lcd.print(gps.course.deg(), 0);
    lcd.print("d");
  }
*/
  
headingDegrees = (unsigned int)gps.course.deg();

  if (headingDegrees < 0 || headingDegrees > 360) {
  }
  else if (headingDegrees >= 0 && headingDegrees <= 11)
  {
    lcd.print("  N");
  }
  else if (headingDegrees > 349 && headingDegrees <= 360)
  {
    lcd.print("  N");
  }
  else if (headingDegrees > 11 && headingDegrees <= 34)
  {
    lcd.print("NNE");
  }
  else if (headingDegrees > 34 && headingDegrees <= 56)
  {
    lcd.print(" NE");
  }
  else if (headingDegrees > 56 && headingDegrees <= 79)
  {
    lcd.print("ENE");
  }
  else if (headingDegrees > 79 && headingDegrees <= 101)
  {
    lcd.print("  E");
  }
  else if (headingDegrees > 101 && headingDegrees <= 124)
  {
    lcd.print("ESE");
  }
  else if (headingDegrees > 124 && headingDegrees <= 146)
  {
    lcd.print(" SE");
  }
  else if (headingDegrees > 146 && headingDegrees <= 169)
  {
    lcd.print("SSE");
  }
  else if (headingDegrees > 169 && headingDegrees <= 191)
  {
    lcd.print("  S");
  }
  else if (headingDegrees > 191 && headingDegrees <= 214)
  {
    lcd.print("SSW");
  }
  else if (headingDegrees > 214 && headingDegrees <= 236)
  {
    lcd.print(" SW");
  }
  else if (headingDegrees > 236 && headingDegrees <= 259)
  {
    lcd.print("WSW");
  }
  else if (headingDegrees > 259 && headingDegrees <= 281)
  {
    lcd.print("  W");
  }
  else if (headingDegrees > 281 && headingDegrees <= 304)
  {
    lcd.print("WNW");
  }
  else if (headingDegrees > 304 && headingDegrees <= 326)
  {
    lcd.print(" NW");
  }
  else if (headingDegrees > 326 && headingDegrees <= 349)
  {
    lcd.print("NNW");
  }
}

void custom00()
{ // uses segments to build the number 0
  lcd.setCursor(x, 0);
  lcd.write(255);
  lcd.write(2);
  lcd.write(255);
  lcd.setCursor(x, 1);
  lcd.write(255);
  lcd.write(4);
  lcd.write(255);
}

void custom1()
{
  lcd.setCursor(x, 0);
  lcd.write(2);
  lcd.write(255);
  lcd.setCursor(x + 1, 1);
  lcd.write(255);
}

void custom2()
{
  lcd.setCursor(x, 0);
  lcd.write(6);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x, 1);
  lcd.write(255);
  lcd.write(byte(0));
  lcd.write(byte(0));
}

void custom3()
{
  lcd.setCursor(x, 0);
  lcd.write(6);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x, 1);
  lcd.write(byte(0));
  lcd.write(byte(0));
  lcd.write(255);
}

void custom4()
{
  lcd.setCursor(x, 0);
  lcd.write(255);
  lcd.write(4);
  lcd.write(255);
  lcd.setCursor(x + 2, 1);
  lcd.write(255);
}

void custom5()
{
  lcd.setCursor(x, 0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(6);
  lcd.setCursor(x, 1);
  lcd.write(byte(0));
  lcd.write(byte(0));
  lcd.write(255);
}

void custom6()
{
  lcd.setCursor(x, 0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(6);
  lcd.setCursor(x, 1);
  lcd.write(255);
  lcd.write(byte(0));
  lcd.write(255);
}

void custom7()
{
  lcd.setCursor(x, 0);
  lcd.write(2);
  lcd.write(2);
  lcd.write(255);
  lcd.setCursor(x + 1, 1);
  lcd.write(255);
}

void custom8()
{
  lcd.setCursor(x, 0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x, 1);
  lcd.write(255);
  lcd.write(byte(0));
  lcd.write(255);
}

void custom9()
{
  lcd.setCursor(x, 0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x + 2, 1);
  lcd.write(255);
}

#endif


