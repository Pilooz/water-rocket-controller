/*****************************************************************************
* Logging Flight data on a SD Card
******************************************************************************/
#include <SPI.h>
#include <SD.h>

#define SD_LED 8  // SD led

// Set the pins used
#define cardSelect 4
File logfile;

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void prepare_logfile() {
  char filename[15];
  strcpy(filename, "/FLIGHT00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0' + i/10;
    filename[8] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  Serial.println("Ready!");
  // Header of file
  logfile.print("millis"); 
  logfile.print(";"); 
  logfile.print("x"); 
  logfile.print(";"); 
  logfile.print("y");
  logfile.print(";"); 
  logfile.print("z");
  logfile.print(";"); 
  logfile.print("ax");
  logfile.print(";"); 
  logfile.print("ay");
  logfile.print(";"); 
  logfile.print("az");
  logfile.print(";"); 
  logfile.println("parachute");
  logfile.flush();
}

void write_to_file(unsigned long currentTime, float x,  float y,  float z, double ax, double ay, double az, boolean p) {
    digitalWrite(SD_LED, HIGH);
    logfile.print(currentTime); 
    logfile.print(";"); 
    logfile.print(x); 
    logfile.print(";"); 
    logfile.print(y); 
    logfile.print(";"); 
    logfile.print(z); 
    logfile.print(";"); 
    logfile.print(ax);
    logfile.print(";"); 
    logfile.print(ay);
    logfile.print(";"); 
    logfile.print(az);
    logfile.print(";"); 
    int parachute = p ? 1 : 0;
    logfile.println(parachute);
    logfile.flush(); // This really writes data in file but causes a consumption peak of about 30mA instead of 10mA
}

void setup_logger() {
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }
}
