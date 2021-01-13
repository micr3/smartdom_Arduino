#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <MFRC522.h>
#include <Keypad.h>
#include <Servo.h>

//#define KONTAKTRON 21
#define CZUJ_PLOMIENI 2
#define ZMIERZCH 3
#define KRANCOWKA 6
#define PIR 10
#define ZAMEK 14
#define PRZYCISK_AUTOMAT 23
//#define PRZYCISK2 41
#define WLACZNIK_SWIATLA 22
#define SWIATLO 15

//RFID RC522 SPI//

#define RST_PIN         5          // Configurable, see typical pin layout above
#define SS_1_PIN        11        // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 2
#define SS_2_PIN        12          // Configurable, take a unused pin, only HIGH/LOW required, must be different to SS 1
#define SS_3_PIN        8
#define SS_4_PIN        9
#define NR_OF_READERS   4 //liczba RC522

byte ssPins[] = {SS_1_PIN, SS_2_PIN, SS_3_PIN, SS_4_PIN};

MFRC522 mfrc522[NR_OF_READERS];   // Create MFRC522 instance.

//BMP280 SPI//
/*
  #define BMP_SCK  (52)
  #define BMP_MISO (50)
  #define BMP_MOSI (51)
  #define BMP_CS   (53)

  Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
*/
//BMP280 I2C//
Adafruit_BMP280 bmp;

float temp1 = 0.0;
float cisnienie1 = 0.0;

// Servomechanizm
Servo roleta1;
//Servo brama1;

//liczenie czasu//
unsigned long czasCzujnika = 10000;
unsigned long czasZamka = 5000;
unsigned long czasKodu = 5000;
unsigned long czasUzbrojenia = 5000;
unsigned long czasWyswietlacza = 2000;
unsigned long czasAlarmu = 2000;

unsigned long aktualnyCzas = 0;
unsigned long zapamietanyCzas = 0;
unsigned long zapamietanyCzasCzujnika = 0;
unsigned long zapamietanyCzasZamka = 0;
unsigned long zapamietanyCzasWyswietlacza = 0;
unsigned long zapamietanyCzasAlarmu = 0;


//zapamietane kody RFID//
int code[] = {41, 165, 124, 194}; //znane UID
String codeMaster = "a9db5fc2";
String codeChild = "e96981c2";

String uidString;
//String uidString;
boolean match = true;
//serwomechanizm
int roleta = 0; //0-zatrzymaj 1- otworz 2- zamknij
//haslo master i haslo child
String hasloMaster = "1234";
String hasloChild = "9876";

char wpisaneHaslo[4];
String wpisaneHasloString = "";
String aName = " ";
int access = 0;
int poprawny = 0;
int proba = 0;
int stan = 2;
bool timerStart = 0;
bool otworz = 0;
int j;
int pos = 0;
bool automat = 0;
bool otworzZamek = 0;
bool uzbrajanie = 0;
bool flaga = 0;
bool roletaZamknieta = 1;

//Klawiatura//
const byte ROWS = 4; // il wierszy
const byte COLS = 3; //il kolumn

byte rowPins[ROWS] = {37, 35, 33, 31}; //piny wierszy
byte colPins[COLS] = {29, 27, 25}; //piny kolumn

char keys[ROWS][COLS] = { //mapowanie klawiatury
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
char key = "";
Keypad klawiatura = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS ); //inicjalizacja klawiatury
//https://forbot.pl/blog/kurs-arduino-ii-klawiatura-wlasny-system-alarmowy-id18341

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x20 for a 16 chars and 2 line display

void setup() {
  Serial.begin(115200);
  Serial.println("Hello!");

  //  setClockDivider();
  SPI.begin();
  ////BMP280 config////
  Serial.println(F("BMP280 test"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    // while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  ////RFID RC522 CONFIG////
  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN); // Init each MFRC522 card
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
  }
  roleta1.attach(4);
  roleta1.write(0);
  pinMode(SWIATLO, OUTPUT);
  digitalWrite(SWIATLO, HIGH);
  pinMode(ZAMEK, OUTPUT);
  digitalWrite(ZAMEK, HIGH);
  //pinMode(KONTAKTRON, INPUT_PULLUP); //Kontaktron jako wejście
  pinMode(ZMIERZCH, INPUT_PULLUP); //Czujnik Zmierzchu jako wejście
  pinMode(PIR, INPUT_PULLUP); //PIR jako wejście
  pinMode(PRZYCISK_AUTOMAT, INPUT_PULLUP); //przycisk wlacz automat
  //pos=roleta.read();
  //// LCD
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.print("Hello, world!");

  pinMode(CZUJ_PLOMIENI, INPUT);
 // attachInterrupt(digitalPinToInterrupt(CZUJ_PLOMIENI), alarm, LOW);
}

////FUNKCJE////
void alarm() //funkcja alarm od przerwania
{
  stan = 0;
  digitalWrite(ZAMEK, LOW);
}
void clearLCD()
{
  lcd.clear();
  lcd.setCursor(0, 0);
}
void stanBMP(int stan, int access)
{
  lcd.clear();
  temp1 = bmp.readTemperature();
  cisnienie1 = bmp.readPressure() / 100;
  switch (stan)
  {
    case 0:
      lcd.print("ALARM");
      break;
    case 1:
      lcd.print("ODBEZP ");
      if (access == 1)
      {
        lcd.print("Master");
      }
      else if (access == 2)
      {
        lcd.print("Child ");
      }
      break;
    case 2:
      lcd.print("UZBROJ");
      break;
  }

  lcd.setCursor(0, 1);
  lcd.print(temp1);
  lcd.print("*C");
  lcd.print(cisnienie1);
  lcd.print("kPa");
}

void wpisywanieHasla(int j)
{
  lcd.clear();
  for (int i = 0; i < j; i++)
  {
    lcd.setCursor(i, 0);
    lcd.print("*");
  }
}

//
String dump_byte_array(byte *buffer, byte bufferSize) {
  String uidString;
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
    uidString = uidString + String(buffer[i], HEX);
  }
  return uidString;
}

void odczytRFID()
{
  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {

    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial())
    {
      Serial.print(F("Reader "));
      Serial.print(reader);
      // Show some details of the PICC (that is: the tag/card)
      Serial.print(F(": Card UID:"));
      uidString = dump_byte_array(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.println();
      Serial.println(uidString);
      // lcd.setCursor(0, 1);
      // lcd.print(uidString);
      Serial.println();
      Serial.print(F("PICC type: "));
      MFRC522::PICC_Type piccType = mfrc522[reader].PICC_GetType(mfrc522[reader].uid.sak);
      Serial.println(mfrc522[reader].PICC_GetTypeName(piccType));
      mfrc522[reader].PICC_HaltA();
      mfrc522[reader].PCD_StopCrypto1();
    }
  }
}
//void odczytKlawiatury()

void sterowanieRoleta(int stan)
{
  while (roleta != 0)
  {
    switch (stan)
    {
      case 1: //otwieranie
        if (pos < 180)
        {
          roleta1.write(pos);
          pos++;
        }
        else
        {
          roleta1.write(180);
          roleta = 0;
          roletaZamknieta = 0;
        }

        break;

      case 2: //zamykanie
        if (pos > 0)
        {
          roleta1.write(pos);
          pos--;
        }
        else
        {
          roleta1.write(0);
          roleta = 0;
          roletaZamknieta = 1;
        }
    }
  }
}


//funkcja do kalibracji rolet
/*
   void kalibracjaRolet()
   {
   while(!digitalRead(KRANCOWKA))
    roleta.write(pos);
    pos--
   }
   pos=0;

*/

//main

void loop() {

  aktualnyCzas = millis();
  if (aktualnyCzas - zapamietanyCzasCzujnika >= czasCzujnika & !flaga)
  {
    stanBMP(stan, access);
    zapamietanyCzasCzujnika = aktualnyCzas;
  }

  if (digitalRead(PRZYCISK_AUTOMAT) == LOW & automat == 0) {
    automat = 1;
  }
  else if (digitalRead(PRZYCISK_AUTOMAT) == LOW & automat == 1) {
    automat = 0;
  }
  if(digitalRead(CZUJ_PLOMIENI)==LOW) alarm();
  sterowanieRoleta(roleta);

  
  switch (stan)
  {
    case 0: //ALARM
      if (!roletaZamknieta)
      {
        Serial.println("zamykanie rolety");
        roleta = 2;
      }
      ////miganie swiatlem////
      if (digitalRead(SWIATLO) == LOW & aktualnyCzas - zapamietanyCzasAlarmu >= czasAlarmu)
      {
        digitalWrite(SWIATLO, HIGH);
        zapamietanyCzasAlarmu = millis();
      }
      else if (digitalRead(SWIATLO) == HIGH & aktualnyCzas - zapamietanyCzasAlarmu >= czasAlarmu)
      {
        digitalWrite(SWIATLO, LOW);
        zapamietanyCzasAlarmu = millis();
      }
      ////
      odczytRFID();
      if (uidString == codeMaster)
      {
        stan = 1;
        access = 1;
        otworzZamek = 1;
        zapamietanyCzasZamka = millis();
        stanBMP(stan, access);
        Serial.print("Case 2");
      }
      if (uidString == codeChild)
      {
        stan = 1;
        access = 2;
        otworzZamek = 1;
        zapamietanyCzasZamka = millis();
        stanBMP(stan, access);
        Serial.print("Case 2");
      }
      uidString = "";

      key = klawiatura.getKey();
      if (key)
      {
        flaga = 1;
        wpisaneHasloString = wpisaneHasloString + String(key);
        Serial.print(key);
        j++;
        wpisywanieHasla(j);
        if (j == 4)
        {
          Serial.print(wpisaneHasloString);
          j = 0;
          if (wpisaneHasloString == hasloMaster)
          {
            stan = 1;
            access = 1;
            otworzZamek = 1;
            zapamietanyCzasZamka = millis();
            stanBMP(stan, access);
          }
          else if (wpisaneHasloString == hasloChild)
          {
            stan = 1;
            access = 2;
            otworzZamek = 1;
            zapamietanyCzasZamka = millis();
            stanBMP(stan, access);
          }
          else
          {
            lcd.clear();
            lcd.print("Bledne haslo");
          }
          flaga = 0;
          wpisaneHasloString = "";
        }
      }
      break;

    case 1:
      //odbezpieczony

      if ((digitalRead(PIR) & digitalRead(ZMIERZCH)) | digitalRead(WLACZNIK_SWIATLA))
      {
        digitalWrite(SWIATLO, LOW);
      }
      else digitalWrite(SWIATLO, HIGH);

      if (digitalRead(ZMIERZCH) == HIGH & !roletaZamknieta)
      {
        Serial.print("zamykanie rolety /n");
        roleta = 2;
      }
      if (digitalRead(ZMIERZCH) == LOW & roletaZamknieta)
      {
        Serial.print("otwieranie rolety /n");
        roleta = 1;
      }

      odczytRFID();
      if (uidString == codeMaster)
      {
        Serial.println("Master");
        access = 1;
        otworzZamek = 1;
        digitalWrite(ZAMEK, LOW);
        zapamietanyCzasZamka = aktualnyCzas;
        stanBMP(stan, access);
      }
      else if (uidString == codeChild)
      {
        Serial.println("Child");
        access = 2;
        otworzZamek = 1;
        digitalWrite(ZAMEK, LOW);
        zapamietanyCzasZamka = aktualnyCzas;
        stanBMP(stan, access);

      }
      uidString = "";
      //uzbrojenei systemu
      //klikniecie * z klawiatury

      if (klawiatura.getKey() == '*')
      {
        lcd.clear();
        lcd.print("Uzbrajanie");
        digitalWrite(ZAMEK, LOW);
        delay(5000); //opoznienie 5 sekund
        digitalWrite(ZAMEK, HIGH);
        roleta = 2;
        stan = 2;
      }


      break;

    case 2: //stan czuwania
      //  zabezpieczony (oczekiwanie na hasło lub RFID)

      if (digitalRead(PIR))
      {
        stan = 0;
      }
      odczytRFID();
      if (uidString == codeMaster)
      {
        otworzZamek = 1;
        Serial.print("Master");
        digitalWrite(ZAMEK, LOW);
        zapamietanyCzasZamka = aktualnyCzas;
        access = 1;
        stan = 1;
        proba=0;
        stanBMP(stan, access);
        break;
      }
      else if (uidString == codeChild)
      {
        otworzZamek = 1;
        Serial.print("Kid");
        digitalWrite(ZAMEK, LOW);
        zapamietanyCzasZamka = aktualnyCzas;
        access = 2;
        stan = 1;
        proba=0;
        stanBMP(stan, access);
        break;
      }
       else if (uidString != "")
       {
        proba++;
       }
      uidString = "";

      key = klawiatura.getKey();
      if ( timerStart & aktualnyCzas - zapamietanyCzas > czasKodu & !poprawny)
      {
        //koniec czasu
        Serial.print("KONIEC CZASU ");
        wpisaneHasloString = "";
        proba++;
        timerStart = 0;
        break;
      }
      if (proba >= 3)
      {
        //ALARM
        proba = 0;
        stan = 0;
        stanBMP(stan, access);
        break;
      }
      if (key)
      {
        flaga = 1;
        wpisaneHasloString = wpisaneHasloString + String(key);
        j++;
        wpisywanieHasla(j);
        if (j == 1)
        {
          Serial.print("Start timera");
          timerStart = 1;
          zapamietanyCzas = aktualnyCzas;
        }
        else if (j == 4)
        {
          j = 0;
          timerStart = 0;
          if (wpisaneHasloString == hasloChild)
          {
            Serial.print("Child");
            otworzZamek = 1;
            digitalWrite(ZAMEK, LOW);
            zapamietanyCzasZamka = aktualnyCzas;
            access = 2;
            stan = 1;
            proba = 0;
            stanBMP(stan, access);
          }
          else if (wpisaneHasloString == hasloMaster)
          {
            Serial.print("Master");
            otworzZamek = 1;
            digitalWrite(ZAMEK, LOW);
            zapamietanyCzasZamka = aktualnyCzas;
            access = 1;
            stan = 1;
            proba = 0;
            stanBMP(stan, access);
          }
          else
          {
            lcd.clear();
            lcd.print("Bledne haslo");
            proba++;
          }
          //zerowanie hasla
          flaga = 0;
          wpisaneHasloString = "";
        }
        key = "";
      }
      break;
  }
  //zamkniecie zamka po czasie

  if (otworzZamek & ( (aktualnyCzas - zapamietanyCzasZamka) >= czasZamka))
  {
    otworzZamek = 0;
    zapamietanyCzasZamka = aktualnyCzas;
    digitalWrite(ZAMEK, HIGH);
  }

  ///DOSTEPY////
  switch (access)
  {
    case 1: //Master
      //while (roleta >= 1) sterowanieRoleta(roleta);

      break;

    case 2: //Kid
      //while (roleta >= 1)  sterowanieRoleta(roleta);
      break;
  }
  delay(100);
}
