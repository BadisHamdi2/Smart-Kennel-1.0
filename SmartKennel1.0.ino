#include "Servo.h"
#include <Wire.h>
#include <RTClib.h>
#include "TM1637Display.h"

// Déclaration du bouton de démarrage
const int START = 43;

// Déclaration du pin de redémarrage
const int RST = 33;

// Déclaration de l'afficheur (TM1637) et ses broches
const int CLC_TM = 23;
const int DIO_TM = 24;
TM1637Display display(23,24);

// Déclaration du l'horloge temps réel (RTC DS1307)
RTC_DS1307 rtc;

// Déclaration du servo moteur et sa broche
Servo myServo;
const int SERVO_PIN = 2;

// Déclaration des broches du capteur Ultrasonic (HC-SR04)
const int TRIG = 25;
const int ECHO = 45;

// Déclaration des broches des leds pour timingMatrix
const int T_LED_1 = 26;
const int T_LED_2 = 27;
const int T_LED_3 = 28;
const int T_LED_4 = 29;
const int T_LED_5 = 30;

// Déclaration des broches des boutons poussoirs des configurations
const int BR_PIN = 54;
const int BRIGHT_CONFIG = 53;
const int H_PLUS = 52;
const int H_MOINS = 51;
const int M_PLUS = 50;
const int M_MOINS = 49;
const int NEXT = 48;
const int T_CONFIG = 47;
const int D_SET = 46;
const int SHOW_LVL = 41;

// Déclaration de broche pour led de distribution
const int DIST_LED = 31;

// Déclaration de broche pour le buzzeur
const int BUZ = 32;

// Déclaration de broche de verification de la fermeture du réservoir
const int CLOSE_PIN = 44;

// Déclaration des variables globeaux
int fps=30,z=0,bright=4,bci=0,dose=3,nxt=0;
int timingMatrix[5][4];
float refDistance = 0;
bool b=true;

void timeShow(TM1637Display display, int hh, int mm, int fps,int e) {
    static int z = e; // Static variable to persist the state across function calls
    int time = hh * 100 + mm; // Calculate the time in a format suitable for display
    if (z >= 0 && z < 500) {
        display.showNumberDecEx(time, 0b01000000); // Display time with colon
        z += fps;
    } else if (z >= 500) {
        display.showNumberDecEx(time, 0); // Display time without colon
        z -= 1000; // Reset z to handle the blinking cycle
    } else { // In case of any unexpected z value, reset it and display time without colon
        display.showNumberDecEx(time, 0);
        z += fps;
    }
}

void brightnessConfig(int bright,int brpin,int brightConf,int bPlus,int bMoins,int bci,bool b,TM1637Display display){
  int* br=&bright;
  int* pbci=&bci;
  bool* pb=&b;
  int d=0;
  if ((digitalRead(brightConf)==HIGH)&&(bci==0)) {
    while (d<180000) {
      if(!b){
        if (digitalRead(bPlus)==HIGH) {
          if (bright < 7) {
            (*br)++;
          };
          d=0;*pb=true;
          brightnessConfig(bright,brpin,42,42,42,1,true,display);
        } else if (digitalRead(bMoins)==HIGH) {
          if (bright > 0) {
            (*br)--;
          };
          d=0;*pb=true;
          brightnessConfig(bright,brpin,42,42,42,1,true,display);
        } else if (digitalRead(brightConf)==LOW) {
        d=d+50;*pb=false;
        };
      };
      *pbci=1;
      if ((digitalRead(brightConf)==HIGH)&&(!b)) {
        d=180000;
      };
      delay(50);
    };
  };
  if (bci==1) {
    analogWrite(brpin,(7-bright)*36);
    display.setBrightness(bright);
  };
}

void doseSet(int dose,int dS,int dPlus,int dMoins,TM1637Display display,int bright,int brpin){
  brightnessConfig(7,brpin,42,42,42,1,true,display);
  bool b=false;
  int* ptrdose=&dose;
  int d=0;
  while (digitalRead(dS)==HIGH) {
    display.showNumberDecEx(dose, 0b00000000);
    b=true;
    delay(10);
  };
  while (b) {
    if (digitalRead(dPlus)==HIGH){
      (*ptrdose)++;
      if ((*ptrdose)>10){
        *ptrdose=10;
      };
      d=0;
      display.showNumberDecEx(dose, 0b00000000);
    } else if (digitalRead(dMoins)==HIGH){
      (*ptrdose)--;
      if ((*ptrdose)<1){
        *ptrdose=1;
      };
      d=0;
      display.showNumberDecEx(dose, 0b00000000);
    } else if ((digitalRead(dS)==HIGH)||(d>=180000)){
      b=false;
    } else {
      d=d+10;
    };
  };
  brightnessConfig(bright,brpin,42,42,42,1,true,display);
}

float croqLevel(int trig,int echo,float refDistance,int closePin,int res,TM1637Display display,int z){
  int d=0;
  if (digitalRead(closePin)==HIGH){
    unsigned long MEASURE_TIMEOUT = 25000UL;
    float SOUND_SPEED = 0.340;
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long measure = pulseIn(echo, HIGH, MEASURE_TIMEOUT);
    float distance_mm = measure / 2.0 * SOUND_SPEED;
    return (refDistance-distance_mm);
  } else {
    if (d>180000){
      digitalWrite(res,HIGH);
    };
    d=d+1000;
    timeShow(display,(2-(d/60000)),((60000-(d%60000))/1000),100,z);
    delay(100);
  };
  return 0.0;
}

void timingConfig(int timingMatrix[5][4],int hPlus,int hMoins,int mPlus,int mMoins,int nextButton,int doneConfigButton,TM1637Display display,int bright,int brpin) {
  while (digitalRead(doneConfigButton)==HIGH) {
    delay(100);
  };
  int recentTMatrix[5][4];
  for (int i=0;i<5;i++) {
    for (int j=0;j<4;j++) {
      recentTMatrix[i][j]=timingMatrix[i][j];
    };
  };
  brightnessConfig(7,brpin,42,42,42,1,true,display);
  for (int i = 0; i < 5; i++){
    timingMatrix[i][1]=0;
    timingMatrix[i][2]=0;
    timingMatrix[i][3]=0; //La 4ème coulonne c'est pour indiquer si cette ligne sera prise en considération ou non donc elle ne prend que 0 ou 1
  };
  int d=0,c=0,h=14400,m=14400;
  while ((c<5)&&(d<180000)){
    digitalWrite(timingMatrix[c][0],HIGH);
    if (digitalRead(doneConfigButton)==HIGH){
      c=5;
    } else if (digitalRead(nextButton)==HIGH) {
      timingMatrix[c][3]=1;
      d=0;c++;h=14400;m=14400;
    } else if (digitalRead(hPlus)==HIGH) {
      h++;d=0;
      timingMatrix[c][1]=h%24;
    } else if (digitalRead(hMoins)==HIGH) {
      h--;d=0;
      timingMatrix[c][1]=h%24;
    } else if (digitalRead(mPlus)==HIGH) {
      m++;d=0;
      timingMatrix[c][2]=m%60;
    } else if (digitalRead(hMoins)==HIGH) {
      m--;d=0;
      timingMatrix[c][2]=m%60;
    } else {
      d=d+250;
    };
    timeShow(display,h%24,m%60,0,500);
    delay(250);
    digitalWrite(timingMatrix[c,0],LOW);
    if (digitalRead(doneConfigButton)==HIGH){
      c=5;
    } else if (digitalRead(nextButton)==HIGH) {
      timingMatrix[c][3]=1;
      digitalWrite(timingMatrix[c,0],HIGH);
      c++;h=14400;m=14400;
    } else if (digitalRead(hPlus)==HIGH) {
      h++;
      timingMatrix[c][1]=h%24;
    } else if (digitalRead(hMoins)==HIGH) {
      h--;
      timingMatrix[c][1]=h%24;
    } else if (digitalRead(mPlus)==HIGH) {
      m++;
      timingMatrix[c][2]=m%60;
    } else if (digitalRead(hMoins)==HIGH) {
      m--;
      timingMatrix[c][2]=m%60;
    } else {
      d=d+250;
    };
    timeShow(display,h%24,m%60,0,500);
    delay(250);
  };
  if(d>=180000) {
    for (int i=0;i<5;i++) {
      for (int j=1;j<4;j++) {
        timingMatrix[i][j]=recentTMatrix[i][j];
      };
    };
  };
  for (int i=0;i<4;i++) {
    if(timingMatrix[i][3]==1){
      for (int j=i+1;j<5;i++) {
        if((timingMatrix[i][1]==timingMatrix[j][1])&&(timingMatrix[i][2]==timingMatrix[j][2])){
          timingMatrix[i][3]=0;
        };
      };
    };
  };
  int checkT = 0;
  for(int i=0;i<5;i++) {
    checkT += timingMatrix[i][3];
  };
  if (checkT == 0) {
    for (int i=0;i<5;i++) {
      for (int j=1;j<4;j++) {
        timingMatrix[i][j]=recentTMatrix[i][j];
      };
    };
  };
  brightnessConfig(bright,brpin,42,42,42,1,true,display);
}

void preWork(Servo myServo,int servoPin,int distLed,int start) {
  int d=14400000-10000;
  myServo.write(0);
  while (digitalRead(start)==LOW){
    if (!myServo.attached()) {
      myServo.attach(servoPin);
    };
    if (d > (14400000-1)) {
      digitalWrite(distLed ,HIGH);
      myServo.write(45);
      delay(2000);
      myServo.write(0);
      d=0;
    };
    delay(100);
    d=d+100;
  };

}

void setup() {
  for (int i=23;i<34;i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i,LOW);
  };
  for (int i=54;i>40;i--) {
    pinMode(i, INPUT);
  };
  preWork(myServo,SERVO_PIN,DIST_LED,START);
  timingMatrix[0][0] = T_LED_1;
  timingMatrix[1][0] = T_LED_2;
  timingMatrix[2][0] = T_LED_3;
  timingMatrix[3][0] = T_LED_4;
  timingMatrix[4][0] = T_LED_5;
  if (!myServo.attached()) {
    myServo.attach(SERVO_PIN);
  };
  myServo.write(45);
  float testingLevel1 = 10.0, testingLevel2 = croqLevel(TRIG,ECHO,0.0,CLOSE_PIN,RST,display,z);
  while (testingLevel1 > testingLevel2) {
    testingLevel1 = croqLevel(TRIG,ECHO,0.0,CLOSE_PIN,RST,display,z);
    delay(1000);
    testingLevel2 = croqLevel(TRIG,ECHO,0.0,CLOSE_PIN,RST,display,z);
  };
  myServo.write(0);
  refDistance = croqLevel(TRIG,ECHO,0.0,CLOSE_PIN,RST,display,z);
  int d=0;
  Serial.begin(9600);
  while (!rtc.begin()) {
    Serial.println("Probleme de trouver RTC");
    delay(100);
    d+=100;
    if (d>60000) {
      digitalWrite(RST,HIGH);
    };
  };
  d=0;
  while (!rtc.isrunning()) {
    Serial.println("RTC n'est pas fonctionnel !");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(100);
    d+=100;
    if (d>60000) {
      digitalWrite(RST,HIGH);
    };
  };
  for (int i=0;i<5;i++) {
    for (int j=1;j<4;i++) {
      timingMatrix[i][j] = 0;
    };
  };
  timingConfig(timingMatrix,H_PLUS,H_MOINS,M_PLUS,M_MOINS,NEXT,T_CONFIG,display,bright,BR_PIN);
  int checkT = 0;
  for(int i=0;i<5;i++) {
    checkT += timingMatrix[i][3];
  };
  if (checkT == 0) {
    digitalWrite(RST,HIGH);
  };
}

void loop() {
  DateTime now = rtc.now();
  timeShow(display,now.hour(),now.minute(),fps,z);
  int* ptrnxt=&nxt;
  if (digitalRead(T_CONFIG)==HIGH) {
    timingConfig(timingMatrix,H_PLUS,H_MOINS,M_PLUS,M_MOINS,NEXT,T_CONFIG,display,bright,BR_PIN);
    timeShow(display,now.hour(),now.minute(),fps,z);
    *ptrnxt=0;
  };
  if (digitalRead(BRIGHT_CONFIG)==HIGH) {
    brightnessConfig(bright,BR_PIN,BRIGHT_CONFIG,H_PLUS,H_MOINS,bci,b,display);
  };
  while (digitalRead(SHOW_LVL)==HIGH) {
    int l = (int)croqLevel(TRIG,ECHO,refDistance,CLOSE_PIN,RST,display,z) / (dose*10);
    display.showNumberDecEx(l, 0b00000000);
    timeShow(display,now.hour(),now.minute(),fps,z);
    delay(2000);
  };
  if (digitalRead(D_SET)==HIGH) {
    doseSet(dose,D_SET,H_PLUS,H_MOINS,display,bright,BR_PIN);
    timeShow(display,now.hour(),now.minute(),fps,z);
  };
  if ((timingMatrix[nxt][1]*100 + timingMatrix[nxt][2]) < (now.hour()*100 + now.minute())) {
    float nf=0;
    nf = croqLevel(TRIG,ECHO,refDistance,CLOSE_PIN,RST,display,z) - dose*10;
    if (!myServo.attached()) {
      myServo.attach(SERVO_PIN);
    };
    digitalWrite(DIST_LED,HIGH);
    myServo.write(45);
    while(croqLevel(TRIG,ECHO,refDistance,CLOSE_PIN,RST,display,z) > nf){
      brightnessConfig(7,BR_PIN,42,42,42,1,true,display);
      delay(50);
      display.setBrightness(0);
      delay(50);
    };
    myServo.write(0);
    digitalWrite(DIST_LED,LOW);
    tone(BUZ,500,50);
    brightnessConfig(bright,BR_PIN,42,42,42,1,true,display);
    myServo.detach();
  };
  for (int i=0;i<5;i++) {
     if ((2*(timingMatrix[i][1]*100 + timingMatrix[i][2]) * (timingMatrix[i][3] - 1)) > (2*(now.hour()*100 + now.minute()))) {
      if ((timingMatrix[i][1]*100 + timingMatrix[i][2]) < (timingMatrix[nxt][1]*100 + timingMatrix[nxt][2])) {
        *ptrnxt = i;
        if (timingMatrix[nxt][3] == 0) {
        (*ptrnxt)++;
        };
      };
     };
  };
  delay(fps);
}
