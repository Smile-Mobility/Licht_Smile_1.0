/*
 * Erstellt Jakob Hölzel
 * Datum 04.11.2023  ( immer Aktualisieren )
 * -----------------------------------------------------------------------------------------
 * Arduino Mini Pro
 * Neopixel Blinker Rechts + LED + Taster
 * Neopixel Blinker Links + LED + Taster
 * Neopixel Rücklicht
 * Nextion Display
 * Neopixel farbeInnenLicht
 * 
 *
 *
 * -----------------------------------------------------------------------------------------
 *
 */

#include <Nextion.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include "Adafruit_MCP23X17.h"
#include <Servo.h>                //Neu

#define RL_PIN 2  // Rück
#define RL_Pixel 90

#define B_R_PIN 4  // Blinker Rechts
#define B_R_Pixel 64

#define B_L_PIN 5  // Blinker Links
#define B_L_Pixel 64

#define Innen_PIN 3  // farbeInnenLicht
#define Innen_Pixel 48

Adafruit_NeoPixel rueckLeuchte(RL_Pixel, RL_PIN, NEO_GRB + NEO_KHZ800);  // Rücklicht
Adafruit_NeoPixel blinkerRechts(B_R_Pixel, B_R_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel blinkerLinks(B_L_Pixel, B_L_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel innenLicht(Innen_Pixel, Innen_PIN, NEO_GRB + NEO_KHZ800);  // farbeInnenLicht
Adafruit_MCP23X17 Erweiterung;
Servo LichtNeigungVorne1; 
Servo LichtNeigungVorne2;

struct Farbe {
  uint8_t rot;
  uint8_t gruen;
  uint8_t blau;
};

struct Farbe farbeInnenLicht = {
  .rot = 30,
  .gruen = 40,
  .blau = 0
};

struct Farbe farbeRueckLicht = {
  .rot = 105,
  .gruen = 60,
  .blau = 0
};

struct Farbe farbeBlinker = {
  .rot = 120,
  .gruen = 150,
  .blau = 0
};

struct Eingaben {
  bool linkerTaster;
  bool rechterTaster;
  bool innenSchalter;
  bool schalterRuecklicht;
  bool warnBlinker;
  bool schalterBremslicht;
  bool schalterRueckwaertslicht;
  uint8_t neigungsWinkelLichtVorne;
};

struct Eingaben letzteEingaben = {
  .linkerTaster = false,
  .rechterTaster = false,
  .innenSchalter = false,
  .schalterRuecklicht = false,
  .warnBlinker = false,
  .schalterBremslicht = false,
  .schalterRueckwaertslicht = false,
  .neigungsWinkelLichtVorne = 10
};

const struct {
  uint8_t BLINKER_RECHTS = 3;
  uint8_t BLINKER_LINKS = 4;
  uint8_t SCHALTER_RUECKLICHT = 6;
  uint8_t LED_LICHT = 14; 
  uint8_t SCHALTER_WARNBLINK_ANLAGE = 7;
  uint8_t LED_WARNBLINK_ANLAGE = 15;
  uint8_t SCHALTER_RUECKWAERTS = 12;  
  uint8_t LED_RICHTUNG = 11;          
  uint8_t SCHALTER_BREMSLICHT = 5;    
} PINS_ERWEITERUNG;

const struct {
  uint8_t TASTER_LINKS = 6;
  uint8_t TASTER_RECHTS = 7;
  uint8_t SCHALTER_INNEN_LICHT = A3;
} PINS_ARDUINO;

const struct {
  uint8_t SCHILD_START = 19;
  uint8_t SCHILD_ANZAHL = 5;
  uint8_t OBEN_START = 24;
  uint8_t OBEN_ANZAHL = 65;
  uint8_t UNTEN_START = 0;
  uint8_t UNTEN_ANZAHL = 19;
  uint8_t RUECKWAERTSLICHT_START = 6;
  uint8_t RUECKWAERTSLICHT_ANZAHL = 6;
} RUECKLICHT_CONFIG;

struct BlinkerZustand {
  bool blinkt = false;
  uint8_t aktuelleLedVorne = 0;
  uint8_t aktuelleLedHinten = 51;
  uint8_t verzoegerung = 0;
} blinkerZustandLinks, blinkerZustandRechts;

void SetupPinsErweiterung() {
  Erweiterung.begin_I2C();
  Erweiterung.pinMode(PINS_ERWEITERUNG.BLINKER_LINKS, OUTPUT);
  Erweiterung.pinMode(PINS_ERWEITERUNG.BLINKER_RECHTS, OUTPUT);
  Erweiterung.pinMode(PINS_ERWEITERUNG.SCHALTER_RUECKLICHT, INPUT);
  Erweiterung.pinMode(PINS_ERWEITERUNG.LED_LICHT, OUTPUT); //Hier
  Erweiterung.pinMode(PINS_ERWEITERUNG.SCHALTER_WARNBLINK_ANLAGE, INPUT);
  Erweiterung.pinMode(PINS_ERWEITERUNG.LED_WARNBLINK_ANLAGE, OUTPUT);
  Erweiterung.pinMode(PINS_ERWEITERUNG.SCHALTER_RUECKWAERTS, INPUT);  
  Erweiterung.pinMode(PINS_ERWEITERUNG.SCHALTER_BREMSLICHT, INPUT); 
  Erweiterung.pinMode(PINS_ERWEITERUNG.LED_RICHTUNG, OUTPUT);         
}

void SetupPinsArduino() {
  pinMode(PINS_ARDUINO.TASTER_LINKS, INPUT);
  pinMode(PINS_ARDUINO.TASTER_RECHTS, INPUT);
  pinMode(PINS_ARDUINO.SCHALTER_INNEN_LICHT, INPUT);
  LichtNeigungVorne1.attach(A0);    
  LichtNeigungVorne2.attach(A1);   
}

void SetupPixels() {
  rueckLeuchte.begin();
  blinkerLinks.begin();
  blinkerRechts.begin();
  innenLicht.begin();
}

void setup() {
  Serial.begin(9600);
  SetupPinsErweiterung();
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  SetupPinsArduino();
  SetupPixels();
}

bool LeseSerielleDaten(char *buffer, unsigned int bufferSize) {
  if (Serial.available() < bufferSize) return false;

  unsigned int index = 0;
  while (Serial.available() > 0 && index < bufferSize) {
    buffer[index++] = Serial.read();
  }

  while(Serial.available() > 0) Serial.read(); // Buffer leeren

  return true;
}

struct Eingaben LeseEingaben() {
  const uint8_t serialBuffer[4] = {};
  const bool neueDatenVorhanden = LeseSerielleDaten(serialBuffer, 4);
  const uint8_t winkel = neueDatenVorhanden ? serialBuffer[0] : letzteEingaben.neigungsWinkelLichtVorne;

  return {
    .linkerTaster = digitalRead(PINS_ARDUINO.TASTER_LINKS) == HIGH,
    .rechterTaster = digitalRead(PINS_ARDUINO.TASTER_RECHTS) == HIGH,
    .innenSchalter = digitalRead(PINS_ARDUINO.SCHALTER_INNEN_LICHT) == HIGH,
    .schalterRuecklicht = Erweiterung.digitalRead(PINS_ERWEITERUNG.SCHALTER_RUECKLICHT) == HIGH,
    .warnBlinker = Erweiterung.digitalRead(PINS_ERWEITERUNG.SCHALTER_WARNBLINK_ANLAGE) == HIGH,
    .schalterBremslicht = Erweiterung.digitalRead(PINS_ERWEITERUNG.SCHALTER_BREMSLICHT) == HIGH,
    .schalterRueckwaertslicht = Erweiterung.digitalRead(PINS_ERWEITERUNG.SCHALTER_RUECKWAERTS) == HIGH,
    .neigungsWinkelLichtVorne = winkel
  };
}

void AktiviereInnenLicht(bool aktiv) {
  if (aktiv) {
    innenLicht.fill(innenLicht.Color(farbeInnenLicht.rot, farbeInnenLicht.gruen, farbeInnenLicht.blau), 0, 48);
  } else {
    innenLicht.clear();
  }
  innenLicht.show();
}

void ProzessiereRuecklicht(bool ruecklichtAktiv, bool bremslichtAktiv, bool rueckwaertslichtAktiv) {
  if (!(bremslichtAktiv || ruecklichtAktiv || rueckwaertslichtAktiv)) {
    Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_LICHT, LOW);  //Hier
    Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_RICHTUNG, LOW);
    rueckLeuchte.clear();
    Serial.print("Licht.val=0");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    rueckLeuchte.show();
    return;
  }

  if (ruecklichtAktiv) {
    rueckLeuchte.fill(rueckLeuchte.Color(100, 100, 20), RUECKLICHT_CONFIG.SCHILD_START, RUECKLICHT_CONFIG.SCHILD_ANZAHL);
  }

  if (bremslichtAktiv) {
    rueckLeuchte.fill(rueckLeuchte.Color(255, 30, 0), RUECKLICHT_CONFIG.OBEN_START, RUECKLICHT_CONFIG.OBEN_ANZAHL);
    rueckLeuchte.fill(rueckLeuchte.Color(255, 30, 0), RUECKLICHT_CONFIG.UNTEN_START, RUECKLICHT_CONFIG.UNTEN_ANZAHL);
  } else {
    rueckLeuchte.fill(rueckLeuchte.Color(40, 0, 0), RUECKLICHT_CONFIG.OBEN_START, RUECKLICHT_CONFIG.OBEN_ANZAHL);
    rueckLeuchte.fill(rueckLeuchte.Color(40, 0, 0), RUECKLICHT_CONFIG.UNTEN_START, RUECKLICHT_CONFIG.UNTEN_ANZAHL);
    Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_LICHT, HIGH);   //Hier geht es an 
    Serial.print("Licht.val=1");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }

  if (!bremslichtAktiv && !ruecklichtAktiv) {
    Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_LICHT, LOW); //Hier wird es abgeschalten verursacht flimmern. wenn ich es raus nehmen Leuchtet die LED beständig.
    rueckLeuchte.clear();
  }

  if (rueckwaertslichtAktiv) {
    rueckLeuchte.fill(rueckLeuchte.Color(100, 100, 20), RUECKLICHT_CONFIG.RUECKWAERTSLICHT_START, RUECKLICHT_CONFIG.RUECKWAERTSLICHT_ANZAHL);
    Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_RICHTUNG, HIGH);
  } else {
    Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_RICHTUNG, LOW);
  }

  rueckLeuchte.show();
}

void ProzessiereBlinker(struct BlinkerZustand &blinkerZustand, Adafruit_NeoPixel &blinker, bool blinkerRechts) {
  const uint8_t erweiterungPin = blinkerRechts ? PINS_ERWEITERUNG.BLINKER_RECHTS : PINS_ERWEITERUNG.BLINKER_LINKS;

  if (!blinkerZustand.blinkt || blinkerZustand.aktuelleLedVorne >= 40) {
    blinkerZustand.aktuelleLedVorne = 0;
    blinkerZustand.aktuelleLedHinten = 51;
    blinkerZustand.verzoegerung = 50;
    blinker.clear();
    blinker.show();

    Erweiterung.digitalWrite(erweiterungPin, LOW);
    Serial.print(blinkerRechts ? "B_R.val=0" : "B_L.val=0");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
    return;
  }

  if (blinkerZustand.verzoegerung > 0) {
    blinkerZustand.verzoegerung -= 1;
    return;
  }

  if (blinkerZustand.aktuelleLedVorne == 20) {
    Erweiterung.digitalWrite(erweiterungPin, HIGH);
    Serial.print(blinkerRechts ? "B_R.val=1" : "B_L.val=1");
    Serial.write(0xff);
    Serial.write(0xff);
    Serial.write(0xff);
  }

  blinker.setPixelColor(blinkerZustand.aktuelleLedVorne, blinker.Color(farbeBlinker.rot, farbeBlinker.gruen, farbeBlinker.blau));
  blinker.setPixelColor(blinkerZustand.aktuelleLedHinten, blinker.Color(farbeBlinker.rot, farbeBlinker.gruen, farbeBlinker.blau));
  blinker.show();

  blinkerZustand.aktuelleLedVorne += 1;
  blinkerZustand.aktuelleLedHinten -= blinkerZustand.aktuelleLedVorne % 3 == 0 ? 1 : 0;  // Bei 3 funktionieren alle hinteren Neos
}

void SetzeBlinkerZustaende(const struct Eingaben &eingaben) {
  if (eingaben.warnBlinker != letzteEingaben.warnBlinker) {
    if (eingaben.warnBlinker) {
      blinkerZustandLinks.blinkt = true;
      blinkerZustandRechts.blinkt = true;
      Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_WARNBLINK_ANLAGE, HIGH);
      Serial.print("R_L.val=1");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    } else {
      blinkerZustandLinks.blinkt = false;
      blinkerZustandRechts.blinkt = false;
      Erweiterung.digitalWrite(PINS_ERWEITERUNG.LED_WARNBLINK_ANLAGE, LOW);
      Serial.print("R_L.val=0");
      Serial.write(0xff);
      Serial.write(0xff);
      Serial.write(0xff);
    }
    blinkerZustandRechts.aktuelleLedVorne = 0;
    blinkerZustandRechts.aktuelleLedHinten = 51;
    blinkerZustandRechts.verzoegerung = 0;  
    blinkerZustandLinks.aktuelleLedVorne = 0;
    blinkerZustandLinks.aktuelleLedHinten = 51;
    blinkerZustandLinks.verzoegerung = 0;  

    return;
  }

  if ((eingaben.linkerTaster != letzteEingaben.linkerTaster) && eingaben.linkerTaster) {
    blinkerZustandLinks.blinkt = !blinkerZustandLinks.blinkt;
    blinkerZustandRechts.blinkt = false;
    blinkerZustandLinks.verzoegerung = 0;
    return;
  }

  if ((eingaben.rechterTaster != letzteEingaben.rechterTaster) && eingaben.rechterTaster) {
    blinkerZustandRechts.blinkt = !blinkerZustandRechts.blinkt;
    blinkerZustandLinks.blinkt = false;
    blinkerZustandRechts.verzoegerung = 0;
  }
}

void ProzessiereLichtServoVorne(const uint8_t neigungsWinkelLichtVorne){
  if(neigungsWinkelLichtVorne == letzteEingaben.neigungsWinkelLichtVorne){
    return;
  }
  LichtNeigungVorne1.write(neigungsWinkelLichtVorne);
  LichtNeigungVorne2.write(neigungsWinkelLichtVorne); // hier winkel anders einstellen
}


void loop() {
  const struct Eingaben eingaben = LeseEingaben();
  AktiviereInnenLicht(eingaben.innenSchalter);
  ProzessiereRuecklicht(eingaben.schalterRuecklicht, eingaben.schalterBremslicht, eingaben.schalterRueckwaertslicht);
  SetzeBlinkerZustaende(eingaben);
  ProzessiereBlinker(blinkerZustandRechts, blinkerRechts, true);
  ProzessiereBlinker(blinkerZustandLinks, blinkerLinks, false);
  ProzessiereLichtServoVorne(eingaben.neigungsWinkelLichtVorne);
  letzteEingaben = eingaben;
}