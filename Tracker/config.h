
// Define your home lat & lon below
// http://andrew.hedges.name/experiments/convert_lat_long/
// Goto aprs.fi and find your deg mm.mm ( leave the secs blank )
// home
//const float HOME_LAT = 51.53;
//const float HOME_LON = 04.09;

const float HOME_LAT = 51.89235;
const float HOME_LON = 04.16303;

// Custom Comments here
#define COMMENT "QRV @ 145.475"

// config file to defines callsign, ssid, symbols and home location
#define MYCALL "PD1DDK"
#define DUALMODE //Kies indien je 2 verschillende sufix wil gebruiken, nl CALL-9 en CALL-5. Sluit schakelaar aan op pin 3.

bool richtingGraden = true; //Om de navigatie richting in graden te zien 
//bool richtingGraden = false; //Om de navigatie richting in graden te zien

bool richtingRoos = true;   //Om de navigatie richting in letters te zien 
//bool richtingRoos = false;   //Om de navigatie richting in letters te zien 

//bool gpsStatus = true;      //Om de gps nauwkeurigheid weer te geven in tekst 
bool gpsStatus = false;      //Om de gps nauwkeurigheid weer te geven in tekst 

// Current Version
#define VERSION "ArduiPRS v1.3 "
