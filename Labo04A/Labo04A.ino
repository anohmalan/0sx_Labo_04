#include <Wire.h>
#include <LCD_I2C.h>
#include <HCSR04.h>
#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 4

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define IN_1 53
#define IN_2 51
#define IN_3 49
#define IN_4 47

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

// États de la porte
enum State {FERME, OUVERTURE, OUVERT, FERMETURE};
State etatPorte = FERME;

// Variables globales
long distance;
unsigned long previousTime = 0;
unsigned long dernierSerial = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastStartupDisplay = 0;
bool startupDisplayed = false;

long int closed = 10, opened = 170;
int distanceOpen = 30, distanceClose = 60;
int nullValue = 0;
long uneTour = 2038;

void setup() {
    Serial.begin(9600);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    lcd.begin();
    lcd.backlight();
    
    myStepper.setMaxSpeed(255.0); // Ajustement pour 2 secondes
    myStepper.setAcceleration(127.5); // Ajustement pour 2 secondes
    myStepper.setSpeed(255); 
    myStepper.setCurrentPosition(closed);

    lastStartupDisplay = millis();
}

void updateStartupDisplay() {
  unsigned long rate = 2000;
  String da = "6334158     ";
    if (!startupDisplayed && millis() - lastStartupDisplay < rate) {
        lcd.setCursor(0, 0);
        lcd.print(da);
        lcd.setCursor(0, 1);
        lcd.print("Labo 4A           ");
    } else if (!startupDisplayed) {
        lcd.clear();
        startupDisplayed = true;
    }
}

void updateDistance() {
  unsigned long rate = 50;
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= rate) {
        distance = mesurerDistance();
        previousTime = currentTime;
    }

}

long mesurerDistance() {
    long duration;
    long distanceMesure = hc.dist();

    return distanceMesure ;
}

void gererPorte() {
    switch (etatPorte) {
        case FERME:
            if (distance < distanceOpen) {
                etatPorte = OUVERTURE;
                myStepper.enableOutputs();
                myStepper.moveTo(opened);
            }
            break;
        case OUVERTURE:
            if (myStepper.distanceToGo() == nullValue) {
                etatPorte = OUVERT;
                myStepper.disableOutputs();
            }
            break;
        case OUVERT:
            if (distance > distanceClose) {
                etatPorte = FERMETURE;
                myStepper.enableOutputs();
                myStepper.moveTo(closed);
            }
            break;
        case FERMETURE:
            if (myStepper.distanceToGo() == nullValue) {
                etatPorte = FERME;
                myStepper.disableOutputs();
            }
            break;
    }
    myStepper.run();
}

void afficherLCD() {
  unsigned long rate = 500;
  long mapIndex = map(myStepper.distanceToGo(), nullValue , uneTour,closed,opened);
    if (millis() - lastLCDUpdate >= rate) { // Mise à jour toutes les 500ms
        lcd.setCursor(0, 0);
        lcd.print("Dist: ");
        lcd.print(distance);
        lcd.print(" cm  ");
        
        lcd.setCursor(0, 1);
        lcd.print("Porte: ");
        if (etatPorte == FERME) {
            lcd.print("Fermee  ");
        } else if (etatPorte == OUVERT) {
            lcd.print("Ouverte  ");
        } else {
            lcd.print(mapIndex);
            lcd.print(" deg ");
        }
        lastLCDUpdate = millis();
    }
}

void envoyerSerial() {
  String da = "6334158";
   unsigned long rate = 100; 
    unsigned long currentTime = millis();
    if (currentTime - dernierSerial >= rate) {
        Serial.print("etd:" + da + ",dist:");
        Serial.print(distance);
        Serial.print(",etat:");
        Serial.println(etatPorte);
        dernierSerial = currentTime;
    }
}

void loop() {
    updateStartupDisplay();
    updateDistance();
    gererPorte();
    afficherLCD();
    envoyerSerial();
}
