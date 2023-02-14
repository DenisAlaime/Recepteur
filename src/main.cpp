//dernière version
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <L298N.h> //driver moteur
#include <Wire.h> //***=sur base de ce qui fut fait avec Badel
#include <Adafruit_PWMServoDriver.h>//***
//module son
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <FireTimer.h>

/*
#define RF_CS 0  // Pin 0 de l'arduino
#define RF_RST 16 // Pin 21 de l'arduino //CE
*/
// #ifdef ARDUINO_AVR_MEGA2560
// #define RF_CS 7
// #define RF_RST 6
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();//***

SoftwareSerial mySoftwareSerial(21, 17); // RX, TX (j'ai mis les vraies PIN de l'arduino)
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

#define RF_CS 0
#define RF_RST 16

#define LEDcabineAv 9
#define LEDcabineArr 15
#define LEDgauche  6
#define LEDdroite 7
#define LEDfrontLight 8 
// #ifdef
// //#elif 
// ARDUINO_AVR_NANO_EVERY
// #define RF_CS 0
// #define RF_RST 16

// // #elif ARDUINO_AVR_NANO
// // #define RF_CS 7  // CS de l'émetteur à la pin GPA4 sur le MCP23017 (CSN)
// // #define RF_RST 6 // RST de l'émetteur à la pin GPA5 sur le MCP23017 (CE)

//  #endif
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define tunnel "PIPE1"

RF24 radio(RF_RST, RF_CS); // Instanciation du NRF24L01

const byte adresse[6] = tunnel; // Mise au format "byte array" du nom du tunnel
char message[32];               // Avec cette librairie, on est "limité" à 32 caractères par message
String msg_saisie;
uint8_t num_channel;
uint8_t cond_start;
char StatLED=0;

uint8_t servonum = 0;//**** Servo-Levage connecté à la PIN LED0 soit n°0 (il faut mettre un nombre entre 0 et 15 et sur l'extender, on a LED0-LED15)


//moteur
const unsigned int IN1=2;
const unsigned int IN2=20;
const unsigned int EnA=10;
L298N motor(EnA, IN1, IN2);
 

 

//float ValeurJoyStk1VertAnalogRecue=0.0;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Nouveau!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
struct PackageData{
    int JoyGaucheX;
    int JoyGaucheY;
    int JoyDroitX;
    int JoyDroitY;
    bool LEDcabAv;
    bool LEDcabArr;
    bool Warning;
    bool FrontLight;
    bool ClignoDroite;
    bool ClignoGauche;
    bool Klaxon;
    byte JoyGauche;
    byte LEDcab;
};
PackageData DataToSend;
float value;
float valuePWM;

void setup()
{
  
  //Je pense ces lignes inutiles pour le Bull
  // Initialisation du port série (pour afficher les infos reçues, sur le "Moniteur Série" de l'IDE Arduino)
  Serial.begin(9600);
  Serial.println("Récepteur NRF24");
  Serial.println("");
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(LEDcabineAv,OUTPUT);
  digitalWrite(LEDcabineAv, LOW);

  pinMode(LEDcabineArr, OUTPUT);
  digitalWrite(LEDcabineArr, LOW);


  // Partie NRF24

  if (!radio.begin())
  {
    Serial.println("Radio error");
  }
  else
  {
    Serial.println("Radio OK");
  }

  radio.openReadingPipe(0, adresse); // Ouverture du tunnel en LECTURE, avec le "nom" qu'on lui a donné
  //Serial.println("52");
  //radio.setDataRate(RF24_2MBPS);
  //Serial.println("54");
  //radio.setChannel(0);
  //Serial.println("56");
  radio.setPALevel(RF24_PA_MIN); // Sélection d'un niveau "MINIMAL" pour communiquer (pas besoin d'une forte puissance, pour nos essais)
  //Serial.println("58");
  radio.setDataRate(RF24_250KBPS);
  //Serial.println("60");
  radio.startListening(); // Démarrage de l'écoute du NRF24 (signifiant qu'on va recevoir, et non émettre quoi que ce soit, ici)
  //Serial.println("62");

  motor.stop(); //arreter le moteur lorsque l'on recharge

  pwm.begin();//***
  pwm.setOscillatorFrequency(27000000);//***
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates ***

  mySoftwareSerial.begin(9600); // dialogue dfplayer
Serial.println("138");
  if (!myDFPlayer.begin(mySoftwareSerial))
  { //Use softwareSerial to communicate with mp3. //(canal)
    Serial.println(F("DF Player ne va pas!"));
    
  }
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms sablier sinon il met un message erreur par ex
  Serial.println("144");
  //----Set volume----
  myDFPlayer.volume(30);   //Set volume value (0~30).
  myDFPlayer.volumeUp();   //Volume Up
  myDFPlayer.volumeDown(); //Volume Down
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);

// Serial.println();
// Serial.println(F("DFRobot DFPlayer Mini Demo"));
// Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
// if (!myDFPlayer.begin(mySoftwareSerial)) { //Use softwareSerial to communicate with mp3
// Serial.println(F("Unable to begin:"));
// Serial.println(F("1.Please recheck the connection!"));
// Serial.println(F("2.Please insert the SD card!"));
// }
  myDFPlayer.play(2);
  delay(2000);
}

void loop()
{
  /************************************************************
   ATTENTION. VÉRIFIER LE COM DANS "platformio.ini" 
  ************************************************************/
  
  while (1)
  {
    if (radio.available()) {
    bool goodSignal = radio.testRPD(); //cette ligne là, je ne la comprends pas vraiment ?
      // Serial.println(goodSignal ? "Strong signal > 64dBm" : "Weak signal < 64dBm");
      // Serial.println("Récepteur ");
    
    //radio.read(&ValeurJoyStk1VertAnalogRecue, sizeof(ValeurJoyStk1VertAnalogRecue)); 
    radio.read(&DataToSend,sizeof(DataToSend)); //lire la totalisté des données reçues 
    //Serial.print("Message reçu : "); 
    //Serial.println(message);     // … et on l'affiche sur le port série !
    // … toutes les secondes !
    //Serial.println(DataToSend);
    
    digitalWrite(13,!(digitalRead(13)));
    // Serial.print("Déplacement x: ");
    // Serial.println(DataToSend.JoyGaucheX);
    Serial.print("Déplacement y: ");
    Serial.println(DataToSend.JoyGaucheY);

    //horlogique=marche avant (à vérifier tout de meme )
    //se baser sur la valeur y reçue 
    if (DataToSend.JoyGaucheY >= 580){
      
      value = map(DataToSend.JoyGaucheY, 530, 1023, 0, 255);
      motor.setSpeed(value);
      motor.forward();
      Serial.println("av");
      // //valeurs de 530 à 1023
      // if ((DataToSend.JoyGaucheY < 860)&&(DataToSend.JoyGaucheY > 694)){
      //   //motor.setSpeed(90);

      //   Serial.println("v2 av");
      // }else 
      // if (DataToSend.JoyGaucheY >= 860){
      //   motor.setSpeed(140);
      //   Serial.println("v3 av");
      // }else{
      //   motor.setSpeed(50);
      //   Serial.println("v1 av");
      // }
    }else
    if (DataToSend.JoyGaucheY <= 430){
      
      value = map(DataToSend.JoyGaucheY, 480, 0, 0, 255);
      motor.setSpeed(value);
      motor.backward();
      Serial.println("arr");
      // if ((DataToSend.JoyGaucheY >160)&&(DataToSend.JoyGaucheY <= 320)){
      //   motor.setSpeed(90);
      //   Serial.println("v2 arr");
      // }else 
      // if (DataToSend.JoyGaucheY <= 160){
      //   motor.setSpeed(140);
      //   Serial.println("v3 arr");
      // }else{
      //   motor.setSpeed(50);
      //   Serial.println("v1 arr");
      // } 
    }else
    {
     // motor.setSpeed(0);
      motor.stop();
      Serial.println("Stop ");
    }
  //Serial.println("pas bloque");
  digitalWrite(LEDcabineAv,DataToSend.LEDcabAv);
  Serial.println("led");
  digitalWrite(LEDcabineArr,DataToSend.LEDcabArr);

  //digitalWrite(LEDfrontLight,digitalRead(DataToSend.FrontLight));

  //while(DataToSend.Warning==1){
  //   if (DataToSend.Warning==1){
  //   digitalWrite(LEDgauche,digitalRead(DataToSend.Warning));
  //   digitalWrite(LEDdroite,digitalRead(DataToSend.Warning));
   
  //   delay(200);
  //   digitalWrite(LEDgauche,digitalRead(!DataToSend.Warning));
  //   digitalWrite(LEDdroite,digitalRead(!DataToSend.Warning));
  //   delay(200);
  //   }
  // //}
  // digitalWrite(LEDgauche,digitalRead(DataToSend.ClignoGauche));
  // digitalWrite(LEDdroite,digitalRead(DataToSend.ClignoDroite));
  // if (DataToSend.ClignoDroite==1){
  //   delay(200);
  //   digitalWrite(LEDdroite,digitalRead(!DataToSend.ClignoDroite));
  //   delay(200);
  // }
  // if (DataToSend.ClignoGauche==1){
  //   delay(200);
  //   digitalWrite(LEDgauche,digitalRead(!DataToSend.ClignoGauche));
  //   delay(200);
  // }



if (DataToSend.JoyDroitY >= 580){
      Serial.println("PWM sens 2");
      valuePWM = map(DataToSend.JoyDroitY, 530, 1023, 0, 180);
      pwm.setPWM(servonum,0,valuePWM);//***
    }else
    if (DataToSend.JoyDroitY <= 430){
      Serial.println("PWM sens 1");
      valuePWM = map(DataToSend.JoyDroitY, 480, 0, 180, 0);
      pwm.setPWM(servonum,0,valuePWM);//***
    }else
    {
      Serial.println("PWM Stop");
      pwm.setPWM(servonum,0,0);//***
    }

  if (DataToSend.Klaxon==1){
    myDFPlayer.play(1); //tester et voir quel son est le klaxon
    //delay(1200);//essayé en M1 et son delay ca va trop vite
   // myDFPlayer.pause();
  }
  
  //mettre des sons quand on fait circuler le BUll
 

  // if (DataToSend.JoyGaucheY<530 && DataToSend.JoyGaucheY>480){
  //  myDFPlayer.loop(3);
  // //delay(5000);
  // }//else if (DataToSend.JoyGaucheY <580 && DataToSend.JoyGaucheY>530){
//    myDFPlayer.play(4);
//   //delay(2000);
//   }else if (DataToSend.JoyGaucheY <650 && DataToSend.JoyGaucheY>580){
//   myDFPlayer.play(5);
//   //delay(5000);
//   }else if (DataToSend.JoyGaucheY <720 && DataToSend.JoyGaucheY>650){
//   myDFPlayer.play(6);
//   //delay(2000);
//  }else if (DataToSend.JoyGaucheY <800 && DataToSend.JoyGaucheY>720){
//   myDFPlayer.play(7);
//   //delay(5000);
//  }else if (DataToSend.JoyGaucheY <850 && DataToSend.JoyGaucheY>800){
//   myDFPlayer.play(8);
//   //delay(2000);
//  }else if (DataToSend.JoyGaucheY <1023 && DataToSend.JoyGaucheY>850){
//   myDFPlayer.play(9);
//   //delay(5000);
//   // }else if (DataToSend.JoyGaucheY <){
//   // myDFPlayer.play(10);
//   // delay(2000);
//   // }else if (DataToSend.JoyGaucheY <){
//   // myDFPlayer.play(11);
//   // delay(3000);
//   }else if (DataToSend.JoyGaucheY <480 && DataToSend.JoyGaucheY>430){
//   myDFPlayer.play(12);
//   //delay(1000);
//   }else if (DataToSend.JoyGaucheY >430 && DataToSend.JoyGaucheY<300){
//   myDFPlayer.play(13);
//   //delay(3000);
//   }else if (DataToSend.JoyGaucheY <300 && DataToSend.JoyGaucheY<0){
//   myDFPlayer.play(14);
//   //delay(2000);
//   // }else if (DataToSend.JoyGaucheY <){
//   // myDFPlayer.play(15);
//   // delay(5000);
//   // }else if (DataToSend.JoyGaucheY <){
//   // myDFPlayer.play(1);
//   // delay(5000);
//   // }else {
//   //   myDFPlayer.pause(); 
//   }

  }                             
 }
  
  msg_saisie = Serial.readString();

  while (!Serial.available())
  {
    Serial.println("74");
    //Afficher le numéro du cannal n'est pas "obligé"
    num_channel = msg_saisie.toInt(); // convertir un string en un integer
    radio.setChannel(num_channel);

     Serial.print("Le numero du canal est: ");
     Serial.println(num_channel);
    delay(1000);
    if (radio.available())
    {
      Serial.println("83");
      bool goodSignal = radio.testRPD(); //cette ligne là, je ne la comprends pas vraiment ?
      Serial.println(goodSignal ? "Strong signal > 64dBm" : "Weak signal < 64dBm");
      Serial.println("Récepteur ");
      radio.read(&message, sizeof(message)); // Si un message vient d'arriver, on le charge dans la variable "message"
      Serial.print("Message reçu : ");
      Serial.println(message); // … et on l'affiche sur le port série !
    }
  }
  // On vérifie à chaque boucle si un message est arrivé
}
