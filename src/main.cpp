#include <Arduino.h>
/***************************************************************
  auto-deploiement de parachute pour fusée
  - Accéléromètre
  - Servo-moteur
  - Bouton start/stop
  - Enrigistreur de paramètres de vol.
  - Micro-controleur Feather 32u4 AdaLogger avec carte SD.

  Déroulé du scénario :
  ---------------------
  A la fin du Setup, le système est en mode REPOS
  A l'appuie sur le bouton rouge, il passe en mode LANCEMENT
  Un autre appuie sur le bouton rouge, il repasse en mode REPOS

  Mode REPOS :
  ------------
  - La led pulse en vert avec une pulsation lente
  - pas de lecture de l'accéléro
  - servo moteur au repos.

  Mode LANCEMENT :
  ----------------
  Dans ce mode la fusée est prête pour le lancement
  - La led pulse rapidement
  - lecture de l'accéléro
  - armement du servo
  - Début de l'enregistrement de données

  Mode PARACHUTE :
  ----------------
  Activé si l'accéléro renvoie z<=0 après une valeur précédente
  positive (seuil à définir)
  - relachement du servo
  - la led continue à pulser (en cas d'atterissage dans les hautes herbes)
  - l'enregistrement de données continue jusqu'à activation du mode repos ou atteint de la limite max d'enregistrements

 ***************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <ADXL345.h>
#include "moyenne_glissante.h"
#include "logger.h"

#define LED_BUILTIN    13
#define START_BUT      12
#define SERVO_PIN      11
#define TIMEOUT_SERVO  480 // ms
#define WRITING_INTERVALL 1000 // in ms : writing flight data to SD
#define MAX_WRITE 100000 // max data to write.

// Mode du système
#define REPOS 0
#define LANCEMENT 1
#define PARACHUTE 2
int mode;

// Constantes pour le servo moteur
#define ANGLE_ARME 90
#define ANGLE_REPOS 170

// Pulse constants
const long BLINK60  = 1000;
const long BLINK120 = 500;
const long BLINK180 = 333;
const long BLINK240 = 250;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long lastWritingTime = 0;
long nb_writes = 0;

int ledBPM = 60;

// Variables et mémoire accéléro
int last_z = 0;
double last_az = 0;
double ax,ay,az;
int x,y,z; 

// Servomoteur pour le parachute
Servo servo;
int pos = 0;

// Accelerometre
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

boolean parachute_released = false;

String line = "--------------------------------------\n";

void accelero_calibration() {
  Serial.println("Calibration de l'accéléromètre");
  // Big positive int
  last_z = 10000;
  last_az = 10000;
}

// goes from 180 degrees to 0 degrees
void holdServo() {
  Serial.println("Armement du parachute.");
  servo.attach(SERVO_PIN);
  servo.write(ANGLE_ARME);
  delay(TIMEOUT_SERVO);
  digitalWrite(SERVO_PIN, LOW);
  servo.detach();
  parachute_released = false;
}

// goes from 0 degrees to 180 degrees
void releaseServo() {
  if (!parachute_released) {
    Serial.println("Libération du parachute.");
    servo.attach(SERVO_PIN);
    servo.write(ANGLE_REPOS);
    delay(TIMEOUT_SERVO);
    digitalWrite(SERVO_PIN, LOW);
    servo.detach();
    parachute_released = true;
  } else {
    Serial.println("Parachute déjà libéré.");
  }
}

void mode_repos() {
  Serial.println(line + "Mode REPOS");
  mode = REPOS;
  nb_writes = 0;
  releaseServo();
  ledBPM = BLINK60;
  Serial.print(line);
}

void mode_lancement() {
  Serial.println(line + "Mode LANCEMENT activé");
  mode = LANCEMENT;
  holdServo();
  ledBPM = BLINK240;
  // Init Accelero
  accelero_calibration();
  prepare_logfile();
  Serial.print(line);
}

void ouverture_parachute() {
  Serial.println(line + "Mode PARACHUTE");
  mode = PARACHUTE;
  // Liberer le parachute
  releaseServo();
  ledBPM = BLINK120;
  Serial.print(line);
}

void on_board_led_pulse(unsigned long blink_interval)
{
  if (currentMillis - previousMillis >= blink_interval) {
    previousMillis = currentMillis;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

/***************************************************************
   Setup
 ***************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println(line + "Mise en route du SETUP");
  // Init I/O
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  
  pinMode(START_BUT, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SD_LED, OUTPUT);

  // Init des états de sortie
  digitalWrite(SERVO_PIN, LOW);

  // Init Servo : position 0
  Serial.println(" . Activation état initial");
  mode_repos();

  // Init Accelerometre
  Serial.println(" . Initialisation de l'accéléromètre");
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(3); // how many seconds of no activity is inactive?

  //look of activity movement on this axes - 1 == on; 0 == off
  adxl.setActivityX(0);
  adxl.setActivityY(0);
  adxl.setActivityZ(1);

  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(0);
  adxl.setInactivityY(0);
  adxl.setInactivityZ(1);

  //set values for what is considered freefall (0-255)
  //adxl.setFreeFallThreshold(0.35); // Recommended 0.3 -0.6 g
  //adxl.setFreeFallDuration(0.1); // Recommended 0.1 s
  adxl.setFreeFallThreshold(8); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(20); //(20 - 70) recommended - 5ms per increment

  //setting all interrupts to take place on int pin 1
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );

  //register interrupt actions - 1 == on; 0 == off
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);

  // Preparing SD for Flight Logging
  setup_logger();
  
  Serial.print("Fin du SETUP.\n" + line);
}

/***************************************************************
   Loop
 ***************************************************************/
void loop() {

  // Changements de modes
  // de REPOS à LANCEMENT : Appuie sur le bouton start/stop
  // de LANCEMENT à REPOS : Re-Appuie sur le bouton start/stop
  if (digitalRead(START_BUT) == LOW ) {
    if ( mode == LANCEMENT || mode  == PARACHUTE) {
      mode_repos();
    } else {
      mode_lancement();
    }
    // Debounce du bouton et du changement de mode
    delay(1500);
  }

  currentMillis = millis();

  // Debug data
  if ( mode != REPOS) { // Ici mode LANCEMENT ou PARACHUTE  (pour continuer à logger des données durant tout le vol.)
    //Boring accelerometer stuff    
    adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
    // Output x,y,z values 
    storeValues(z);
    
    double xyz[3];
    adxl.getAcceleration(xyz);
    ax = xyz[0];
    ay = xyz[1];
    az = xyz[2];

    Serial.print(z);
    Serial.print("\t");
    Serial.print(get_moy_Z());
    Serial.print("\t");
    Serial.print(get_max_Z());
    Serial.print("\t");
    Serial.println(az);

    // When z becomes < 0 and acceleration too, let's trig the parachute
    if ( get_moy_Z() < 0 ) {
        ouverture_parachute();
       }
       last_z = z;
       last_az = az;

    // Writing fight data every WRITING_INTERVALL
    if ( currentMillis - lastWritingTime > WRITING_INTERVALL ) {
      if (mode != REPOS) {
        if (nb_writes < MAX_WRITE) {
          write_to_file(currentMillis, x, y, z, ax, ay, az, parachute_released);
          lastWritingTime = currentMillis;
          nb_writes++;
        }
      }
    }
  } // End mode != REPOS

  // Displaying LED
  on_board_led_pulse(ledBPM);
  // SD LED : not writing now, so LOW
  digitalWrite(SD_LED, LOW);
}
