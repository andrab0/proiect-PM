#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "WiFiEsp.h"

#ifndef HAVE_SERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(7, 6); //RX, TX
#endif

Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiEspClient client;

// declarari pt modulul de WiFi:
char ssid[] = "AndraB"; // nume retea wifi
char pass[] = "yugy1803"; // parola wifi
int status = WL_IDLE_STATUS;
char server[] = "maker.ifttt.com";

// asocierea pinilor cu componentele:
unsigned const int SenzorNivelApa = A3;
unsigned const int Buton = 2;
unsigned const int Buzzer = 3;
unsigned const int ServoMotor = 4;
unsigned const int ledR = 11;
unsigned const int ledG = 10;
unsigned const int ledB = 9;
unsigned const int trig = 12;
unsigned const int echo = 13;

int nivelApa = 0;
int flagBuzzer = 1,
    flagLcd = 1,
    flagServo = 1,
    flagLed = 0,
    flagRed = 0,
    flagHTTP = 1,
    flagUltrasonic = 0;
int ts = 0;
int val1 = 90,
    val2 = 180,
    timer = 0;
int redVal = 0,
    greenVal = 255,
    blueVal = 0;

long durataUltrasonic;
int distantaMasurata;

void setup() {
  // put your setup code here, to run once:
  // pt afisarea nivelului apei:
  Serial.begin(115200);

  // pt initializarea LCD-ului:
  lcd.begin();
  lcd.backlight();

  // pt initializarea butonului:
  pinMode(Buton, INPUT_PULLUP);

  // pt initializareaa buzzer-ului:
  pinMode(Buzzer, OUTPUT);

  // pt ledRGB:
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  // pt servo motor:  
  myServo.attach(ServoMotor);
//  myServo.write(val2);

  // pt ultrasonic:
  pinMode(trig, OUTPUT);  
  pinMode(echo, INPUT);
  // intreruperea pentru apasarea butonului:
  EICRA |= (1 << ISC10);
  EIMSK |= (1 << INT0);

//  initializare seriala pentru ESP8266:
  Serial1.begin(115200);
//  initializare modul ESP8266:
  WiFi.init(&Serial1);

//  cautare raspuns wifi shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("Nu se detecteaza WiFi shield");
    // nu continua:
    while (true);
  }

//  incercare conectare retea WiFi:
  while ( status != WL_CONNECTED) {
    Serial.print("Incerc sa ma conectez la WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

//  afisare in caz de succes:
  Serial.println("Bucura-te de Wi-Fi!");
}

ISR(INT0_vect)
{
  if(millis() - ts >= 100)
  {
    ts = millis();
    timer = 0;
    // refac led-ul verde:
    flagRed = 1;

    // opresc sonorul buzzer-ului:
    noTone(Buzzer);
    flagBuzzer = 0;

    // scot afisarea de pe lcd:
    flagLcd = 0;

    // modific pozitia servo motorului:
    flagServo = 0; 
    flagUltrasonic = 1;
  }
}

void toggleLed()
{
  if (flagRed == 0) {
    if (flagLed == 0)
    {
      redVal = 255;
      greenVal = 0;
      analogWrite(ledR, redVal);
      analogWrite(ledG, greenVal);
      analogWrite(ledB, blueVal);
      flagLed = 1;
      delay(200);
    }
    else
    {
      redVal = 0;
      greenVal = 0;
      analogWrite(ledR, redVal);
      analogWrite(ledG, greenVal);
      analogWrite(ledB, blueVal);
      flagLed = 0;
      delay(200);
    }
  }
  else
  {
    greenVal = 255;
    redVal = 0;
    analogWrite(ledR, redVal);
    analogWrite(ledG, greenVal);
    analogWrite(ledB, blueVal);
    delay(200);
  }
  
}

void makeBipBip()
{
  if (flagBuzzer == 1)
  {
    tone(Buzzer, 2000);
    delay(200);
    noTone(Buzzer);
    delay(200);
  }
  else
  {
    noTone(Buzzer);
    delay(200); 
  }
}

void printLcd()
{
  if (flagLcd == 1)
  {
    lcd.clear();
    lcd.print("NIVEL APA CRITIC!");
    lcd.setCursor(0,1);
    lcd.print("APASATI BUTONUL!");
    lcd.setCursor(1,1);
    delay(500);
  }
  else {
    lcd.clear();
    lcd.print("");
    delay(500);
  }
}

void light()
{
    // pornesc led-ul pe verde:
  analogWrite(ledR, redVal);
  analogWrite(ledG, greenVal);
  analogWrite(ledB, blueVal);
}

void makeHttpRequest()
{
  Serial.println();
  client.stop();
//  Serial.println("Start conectare la server...");
  // daca se conecteaza cu succes, afisez in seriala:
  if(client.connect(server, 80))
  {
    Serial.println("Conectare reusita");
    // fac cererea http:
    client.println("POST http://maker.ifttt.com/trigger/alerta_baraj/json/with/key/nwjpA4wt5osMWZzg-upQSqptOXuRbfrgxSYdF1GRcGT HTTP/1.1");
    client.println("Host: maker.ifttt.com");
    client.println("Connection: close");
    client.println();
  }

}

void loop() {
  // citesc nivelul apei:
  nivelApa = analogRead(SenzorNivelApa);
  // afisez valoarea pe seriala:
  Serial.print("nivel apa: ");
  Serial.println(nivelApa);
  
  // daca nivelul apei este depasit se activeaza alarma si timer-ul
  // pentru timpul in care este necesara apasarea butonului pana
  // la trimiterea unei instiintari:
  if (nivelApa >= 300) {
    toggleLed();
    makeBipBip();
    printLcd();
    timer = millis();
    flagHTTP = 1;
  }
  else {
    light();
  }

  if((millis() - timer >= 10000) && (flagHTTP == 1))
  {
    //fac un request http:
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
    
    makeHttpRequest();

    // resetez timer-ul:
    timer = 0;
    flagHTTP = 0;

    Serial.print("Deconectare de la server ...");
  } 

  if(flagServo == 0)
  {
    myServo.write(val2);
  }
  else
  {
    myServo.write(val1);
  }
  
  if (flagUltrasonic = 1); 
  {
    // citeste distanta si modifica pozitia servo motorului cand e ok:
    digitalWrite(trig, LOW);
    delay(200);
    digitalWrite(trig, HIGH);
    delay(1000);
    digitalWrite(trig, LOW);

    durataUltrasonic = pulseIn(echo, HIGH);
    distantaMasurata = durataUltrasonic * 0.034 / 2;

    if (distantaMasurata <= 4)
    {
      flagServo = 1;
    }
  }

  delay(1000);
}
