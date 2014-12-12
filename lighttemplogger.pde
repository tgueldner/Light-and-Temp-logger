#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "DHT.h"
#include <RTCTimedEvent.h>
#include <EEPROM.h>  // Contains EEPROM.read() and EEPROM.write()

// Sound Sensor Pin
#define SOUNDSENSORPIN 3
#define SOUNDLIMIT 10

// START DHT
// URL: 
#define DHTPIN 2     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
// NOTE: For working with a faster chip, like an Arduino Due or Teensy, you
// might need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// Example to initialize DHT sensor for Arduino Due:
//DHT dht(DHTPIN, DHTTYPE, 30);
// END DHT

// A simple data logger for the Arduino analog pins
// https://learn.adafruit.com/adafruit-data-logger-shield/for-the-mega-and-leonardo

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
// #define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
// #define SYNC_INTERVAL 60000 // mills between calls to flush() - to write data to the card
// uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs
#define redLEDpin 13 // 2
#define greenLEDpin 13 // 3

#define BANDGAPREF 14            // special indicator that we want to measure the bandgap

#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
#define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off

RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

int soundSum = 0;
boolean soundOn = false;

int logfileNumber = 0;

//SETTINGS
// ID of the settings block
#define CONFIG_VERSION "TL1"

// Tell it where to store your config data in EEPROM
#define CONFIG_START 1


//  settings structure
struct StoreStruct {
  // This is for mere detection if they are your settings
  char version[4];
  // The variables of your settings
  int logfileNumber;
} storage = {
  CONFIG_VERSION,
  // The default values
  0
};

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t=0; t<sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
}

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Temp/RH logger V0.1 started.");
  
  // sound
  pinMode(SOUNDSENSORPIN, INPUT);
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    error("RTC failed");
  }
  
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }  
  Serial.println("done");
  
  Serial.print("Initializing DHT ...");  
  dht.begin();
  Serial.println("done");
  
  Serial.print("Loading config ...");  
  loadConfig();
  Serial.println("done");
  
  delay(1000);
  
  logfileNumber = storage.logfileNumber;
 
  // If you want to set the aref to something other than 5v
  analogReference(EXTERNAL);
  
  //initial buffer timers
  RTCTimedEvent.initialCapacity = sizeof(RTCTimerInformation)*4;

  RTCTimedEvent.addTimer(0,         //minute
                         TIMER_ANY,         //hour
                         TIMER_ANY, //day fo week
                         TIMER_ANY, //day
                         TIMER_ANY, //month
                         readAndSave);
  RTCTimedEvent.addTimer(15,         //minute
                         TIMER_ANY,         //hour
                         TIMER_ANY, //day fo week
                         TIMER_ANY, //day
                         TIMER_ANY, //month
                         readAndSave);
  RTCTimedEvent.addTimer(30,         //minute
                         TIMER_ANY,         //hour
                         TIMER_ANY, //day fo week
                         TIMER_ANY, //day
                         TIMER_ANY, //month
                         readAndSave);
  RTCTimedEvent.addTimer(45,         //minute
                         TIMER_ANY,         //hour
                         TIMER_ANY, //day fo week
                         TIMER_ANY, //day
                         TIMER_ANY, //month
                         readAndSave);
                         
   printBTHelp();
}

void loop(void)
{
  RTCTimedEvent.loop();
  //Narcoleptic.delay(8000);
  
  // sound handling
  handleSound();
  
  // handle bluetooth
  handleBT();
  
  // delay
  delay(1000);
}

void printBTHelp(void) {
  Serial.println("Willkommen beim Keller-Logger:");
  Serial.println("   a : aktuelle Werte anzeigen");
  Serial.println("   l : Liste anzeigen / download");
  Serial.println("   r : Reset, loescht letzte Liste");
  Serial.println("   h : diese Hilfe");
}

void handleBT(void) {
  String cmd = "";
  char character;
  while(Serial.available() > 0) {
      character = Serial.read();
      //if (character == 13) {
      //  break; // leave while
      //}
      cmd = cmd + character;
  }  
  
  if(cmd.length() > 0) {
    // enter key was hit
#if ECHO_TO_SERIAL
    Serial.print("Bluetooh received: ");
    Serial.println(cmd);
#endif //ECHO_TO_SERIAL
    
    if(cmd.equals("h")) {
      printBTHelp();
    } else if(cmd.equals("a")) {
      // actual values
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float rh = dht.readHumidity();
      // Read temperature as Celsius
      float t = dht.readTemperature();
      Serial.print("Temp: ");
      Serial.print(t);
      Serial.print("  RH: ");
      Serial.print(rh);
      Serial.println("%");
    } else if(cmd.equals("l")) {
      // download
      File logfile = getDataLogfile(logfileNumber, FILE_READ);
      Serial.print("Logfile opened for reading: ");
      Serial.print(logfile.name());
      Serial.print(" ");
      Serial.print(logfile.size());
      Serial.println("bytes");
      while(logfile.available() > 0) {
        Serial.write(logfile.read());
      }
      logfile.close();
      Serial.println("Download done!");
    } else if(cmd.equals("r")) {
      // reset
      logfileNumber++;
      saveConfig();
      Serial.println("Schreibe in neue Datei.");
    } else {
      // command not supported
      Serial.println("unbekannter Befehl");
    }
  }
}

File getDataLogfile(int logfileNumber, byte mode) {
  // create a new file
  char filename[] = "LOGGER00.CSV";
  filename[6] = logfileNumber/10 + '0';
  filename[7] = logfileNumber%10 + '0';
    
  boolean newFile = !SD.exists(filename);
#if ECHO_TO_SERIAL
  Serial.print("Try to open logfile: ");
  Serial.println(filename);
#endif
  File logfile = SD.open(filename, mode); 
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  if(newFile) {
    // create heading for new log file 
    logfile.println("millis,stamp,datetime,temp,rh,sound,vcc");    
    logfile.close();
#if ECHO_TO_SERIAL
    Serial.println("log file heading: millis,stamp,datetime,temp,rh,sound,vcc");
#endif //ECHO_TO_SERIAL
  }
  
  return logfile;
}

void handleSound(void) {
  int soundReading = 0;
  for(int i=0; i<50; i++) {
    // 1 = off, no sound; 0 = on, sound -> convert to 1 = on and 0 = off
    soundReading += ((digitalRead(SOUNDSENSORPIN) - 1 )* -1);
    delay(10);
  }
  if(soundReading == 0 && soundSum > 0) {
      soundSum--;
  } else if(soundReading >= 1 && soundSum < SOUNDLIMIT) {
    soundSum++;
  }
  // sound hysterese
  if(soundSum == SOUNDLIMIT && !soundOn) {
    soundOn = true;
  } else if(soundSum == 0 && soundOn) {
    soundOn = false;
  }
  
#if ECHO_TO_SERIAL
  Serial.print("soundSum:");
  Serial.print(soundSum);
  Serial.print(" soundOn:");
  Serial.println(soundOn);
#endif
}

void readAndSave(RTCTimerInformation* Sender) {
  digitalWrite(greenLEDpin, HIGH);
  
  // fetch the time
  DateTime now = RTC.now();
  
  File logfile = getDataLogfile(logfileNumber, FILE_WRITE);
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
#endif

  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float rh = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();  
  
  logfile.print(", ");    
  logfile.print(t);
  logfile.print(", ");    
  logfile.print(rh);
  logfile.print(", ");    
  logfile.print(soundOn);
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(t);
  Serial.print(", ");    
  Serial.print(rh);
  Serial.print(", ");    
  Serial.print(soundOn);
#endif //ECHO_TO_SERIAL

  // Log the estimated 'VCC' voltage by measuring the internal 1.1v ref
  analogRead(BANDGAPREF); 
  delay(10);
  int refReading = analogRead(BANDGAPREF); 
  float supplyvoltage = (bandgap_voltage * 1024) / refReading; 
  
  logfile.print(", ");
  logfile.print(supplyvoltage);
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(supplyvoltage);
#endif // ECHO_TO_SERIAL

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  //if ((millis() - syncTime) < SYNC_INTERVAL) return;
  //syncTime = millis();
  
 //-> jedes mal Speichern
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.close();
  digitalWrite(redLEDpin, LOW);
  
}
