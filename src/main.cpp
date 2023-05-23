//dernière version
#include <Arduino.h>
#include "main.h"
#include "user.h"
#include "menu.h"
#include <PROGMEM_readAnything.h>
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
//sdfsdf
//sfsfdsf

// #ifdef ARDUINO_AVR_MEGA2560
// #define RF_CS 7
// #define RF_RST 6


#define ENABLE_DEBUG_OUTPUT

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();//***

SoftwareSerial mySoftwareSerial(21, 17); // RX, TX (j'ai mis les vraies PIN de l'arduino)
DFRobotDFPlayerMini myDFPlayer;

//void printDetail(uint8_t type, int value);

unsigned char state;  // holds the current state, according to "menu.h"
unsigned char nextstate;
PGM_P statetext;
char (*pStateFunc)(char);
unsigned char input; //Attention pour une impression correcte avec serial.print, il faut un "unsigned char"
unsigned char i, j; // char i;
char bufferStr[30]; // buffer utiliser pour copier des chaines de caractères en mémoire programme vers la Ram pour pouvoir être imprimées.
char Transition= 0; 
bool ReadIO= TRUE;
void ReadKey(void);

unsigned int global_counter=0;//permet de générer des tempo dans la routine d'interruption.

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

 // #elif ARDUINO_AVR_NANO
// // #define RF_CS 7  // CS de l'émetteur à la pin GPA4 sur le MCP23017 (CSN)
// // #define RF_RST 6 // RST de l'émetteur à la pin GPA5 sur le MCP23017 (CE)

//  #endif
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define ESCOFF     1000
#define ESCON       1400

#define tunnel "PIPE1"

RF24 radio(RF_RST, RF_CS); // Instanciation du NRF24L01

const byte adresse[6] = tunnel; // Mise au format "byte array" du nom du tunnel
char message[32];               // Avec cette librairie, on est "limité" à 32 caractères par message
String msg_saisie;
uint8_t num_channel;
uint8_t cond_start;
char StatLED=0;


#define servonum_0 0  //**** Servo-Levage connecté à la PIN LED0 soit n°0 (il faut mettre un nombre entre 0 et 15 et sur l'extender, on a LED0-LED15)
#define servonum_1 1
#define servonum_2 2
#define servonum_3 3
#define servonum_4 4
#define ESC 5

//moteur
#define IN1 2
#define IN2 20
#define EnA 10
L298N motor(EnA, IN1, IN2);
 
//float ValeurJoyStk1VertAnalogRecue=0.0;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Nouveau!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
struct PackageData{
    int JoyStk1HorValue=513;
    int JoyStk1VertValue=513;
    int JoyStk2HorValue=513;
    int JoyStk2VertValue=513;
    bool LEDcabAv=0;
    bool LEDcabArr=0;
    bool Warning=0;
    bool ClignoDroite=0;
    bool ClignoGauche=0;
    bool FrontLight=0;
    bool Klaxon=0;
    byte JoyGauche=0;
    byte LEDcab=0;
    bool ESCpump=0;
};
PackageData DataToReceive;
float value;
float valuePWM;
float valESC;

void setup()
{
  
  
    state = ST_S1;
    nextstate = ST_S1;
    statetext = MT_S0Start;//NULL;//(menu_state[0].pText);//MT_TIME_CLOCK;//MT_HELLO;
    pStateFunc = NULL;  
  //Je pense ces lignes inutiles pour le Bull
  // Initialisation du port série (pour afficher les infos reçues, sur le "Moniteur Série" de l'IDE Arduino)
  Serial.begin(9600);
  Serial.println("Récepteur NRF24");
  Serial.println("");
  pinMode(13, OUTPUT);
  //pinMode(5,INPUT_PULLDOWN);
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
  
  //radio.setChannel(0);
  
  radio.setPALevel(RF24_PA_MIN); // Sélection d'un niveau "MINIMAL" pour communiquer (pas besoin d'une forte puissance, pour nos essais)
  radio.setDataRate(RF24_250KBPS);
  radio.startListening(); // Démarrage de l'écoute du NRF24 (signifiant qu'on va recevoir, et non émettre quoi que ce soit, ici)
  
  motor.stop(); //arreter le moteur lorsque l'on recharge

  pwm.begin();//***
  pwm.setOscillatorFrequency(27000000);//***
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates ***

  mySoftwareSerial.begin(9600); // dialogue dfplayer

  if (!myDFPlayer.begin(mySoftwareSerial))
  { //Use softwareSerial to communicate with mp3. //(canal)
    Serial.println(F("DF Player ne va pas!"));
    
  }
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms sablier sinon il met un message erreur par ex

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
    
    for (;;)            // Main loop
    {

        //Serial.println("207"); // Exemple affichage sur le terminal
        if (ReadIO) //"ReadIO" est placée à true dans une fonction d'interruption (Timer) pour avoir la main sur la période d'appel de la 
                    //fonction "ReadIOFuncion"
        {
            ReadKey();
        }

        if (statetext)
        {   
            strcpy_P(bufferStr, statetext);//nécessaire pour pouvoir récupérer la string en mémoire programme.
            //https://www.arduino.cc/reference/en/language/variables/utilities/progmem/ les explication de site son à réfléchir, ça ne correspond pas tout à fait aux observations.
            
            Serial.println(bufferStr); // Exemple affichage sur le terminal
            statetext = NULL;//Important pour ne pas réafficher chaque fois le message, surcharge du LCD, pas utile.
        }

        //input = getkey();           // Read buttons
        input = GetTransition();
        if (input != KEY_NULL)// Serial.println(input);

        if (pStateFunc)  //Si une fonction est en court, le pointeur de foncion est différent de 0 et la fonction pointée par ce pointeur
                        //est appelée en boucle. 
        {
            nextstate = pStateFunc(input); // le changement d'état se fait à l'intérieur de la fonction appelée et est retourné par celle-ci.
        }
        
        else if (input != KEY_NULL)
        {
            nextstate = StateMachine(state, input); // S'il n'y a pas de fonction en cours, le changement d'état est recalculé dans la "stateMachine" en fonction des touches activées.
            //Voir tableau "const   MENU_NEXTSTATE menu_nextstate[]" dans menu.h
            //Serial.println(nextstate);
        }

        if (nextstate != state)
        {
            state = nextstate;//s'il y a eu un changement d'état, la machine prend ce nouvel état.
            for (i=0; 1; i++)
            {
                MENU_STATE menu_state_ram;// obligation de passer par une variable tempon pour copier la structure en mpr pour l'utiliser dans l'application.
                PROGMEM_readAnything (&menu_state[i],menu_state_ram); // Code repris du Web pour effectuer la copie d'une ligne du tableau
                                                                        // dans la variable en Ram qui est égale au type "MENU_STATE" du tableau
                                                                        //Une seule ligne du tableau est copiée à la fois.
                                
                if (menu_state_ram.state == state)
                {
                    statetext = menu_state_ram.pText;// permettra d'afficher le nouveau texte en fonction de l'état dans lequel on se trouve
                    pStateFunc = menu_state_ram.pFunc;   //permettra d'appeler la fonction liée à l'état dans lequel on se trouve.                                     
                    break; //Lorsque le nouvel état est trouvé, on quitte la boucle.
                }
            }
        }
        
    SetTransition(KEY_NULL);
    //fin de la machine d'état.
    
    } //End Main loop 
  }
}     
/*****************************************************************************
*
*   Function name : StateMachine
*
*   Returns :       nextstate
*
*   Parameters :    state, stimuli
*
*   Purpose :       Shifts between the different states
*
*****************************************************************************/
unsigned char StateMachine(char state, unsigned char stimuli)
{
    unsigned char nextstate = state;    // Default stay in same state
    unsigned char i, j;

    // mt: for (i=0; menu_nextstate[i].state; i++)
    for (i=0; ( j=menu_nextstate[i].state); i++ )
    {   //Tout le tableau est parcouru, on affecte "j" à chaque ligne du tableau et on teste si "j" est égale au "state" avec le "stimuli"
        //corestpondant pour changer d'état.
        // mt: if (menu_nextstate[i].state == state && menu_nextstate[i].input == stimuli)
        if (( j == state) && (menu_nextstate[i].input == stimuli))
        {
            nextstate = menu_nextstate[i].nextstate;//Si l'on trouve un nouvel état en fonction du stimuli, "nextstate" est modifié
                                                    //si non, on sort de la fonction en gardant l'état au moment de l'appel
                                                    // grâce à la ligne "unsigned char nextstate = state" en début de cette fonction.
            break;
        }
    }
    return nextstate;
}

char GetTransition (void)
    {

        return Transition;

    }

void SetTransition (char _transition)
      {

      Transition=_transition;
      } 
    


void ReadKey(void)
{
     if (radio.available()){
      SetTransition(KEY_RadioDataAvailable);
     // Serial.println("ReadKey");
      // while (1)
      //   {
      //     /* code */
      //   }
     }

    

}    
    
 char AnalyzeData(char input)//ST_S2_ReceiveDataFunc
{
   static char enter = 1;
   uint16_t valuePWM;
   //int valESC;

    if (enter)
    {
        enter = 0;        
        
        //Serial.println("ST_S2_ReceiveDataFunc");
        // while (1)
        // {
        //   /* code */
        // }
        
    } 

    radio.read(&DataToReceive,sizeof(DataToReceive)); //lire la totalisté des données reçues 
//moteur dc
    if (DataToReceive.JoyStk1VertValue >= 580){
      
      value = map(DataToReceive.JoyStk1VertValue, 530, 1023, 0, 255);
      motor.setSpeed(value);
      motor.forward();
      //Serial.println("av");
      
    }else
    if (DataToReceive.JoyStk1VertValue <= 430){
      
      value = map(DataToReceive.JoyStk1VertValue, 480, 0, 0, 255);
      motor.setSpeed(value);
      motor.backward();
      //Serial.println("arr");
    }else
    {
     // motor.setSpeed(0);
      motor.stop();
      //Serial.println("Stop ");
    }
    //fin moteur dc

  //Serial.println("pas bloque");

  //Leds cabine
   digitalWrite(LEDcabineAv,DataToReceive.LEDcabAv);
   //Serial.println("led");
   digitalWrite(LEDcabineArr,DataToReceive.LEDcabArr);
  // fin Leds cabine

//Servo

   valuePWM = map(DataToReceive.JoyStk2VertValue, 0, 1023, USMIN, USMAX);
    Serial.print("DataToReceive.JoyStk2VertValue:");
    Serial.println(DataToReceive.JoyStk2VertValue);
    Serial.print("DataToReceive.JoyStk2hor:");
    Serial.println(DataToReceive.JoyStk2HorValue);
   //pwm.writeMicroseconds(0,valuePWM);
   //pwm.setPWM(servonum_0,0,valuePWM);//***
   //valuePWM = map(DataToReceive.JoyStk2HorValue, 0, 1023, SERVOMIN, SERVOMAX);
   valuePWM = map(DataToReceive.JoyStk1HorValue, 0, 1023, USMIN, USMAX);
   pwm.writeMicroseconds(servonum_0,valuePWM);//***
   //valuePWM = map(DataToReceive.JoyStk1HorValue, 0, 1023, 0, 180);
   pwm.writeMicroseconds(servonum_1,valuePWM);//***
   //Serial.println(DataToReceive.JoyStk1HorValue);
   Serial.println(valuePWM);
   
// fin servo

// ESC pompe
if (DataToReceive.ESCpump == 1)
{
  //valESC = 1300;
  Serial.print("pump : ");
  Serial.println(DataToReceive.ESCpump);
  Serial.println(ESCON);
   pwm.writeMicroseconds(ESC,ESCON);//***
}else
{
  //valESC = 1000;
   Serial.print("pump : ");
  Serial.println(DataToReceive.ESCpump);
  pwm.writeMicroseconds(ESC,ESCOFF);
}

 // fin ESC pompe 
   
  
  // if (DataToReceive.JoyStk2VertValue >= 580){
  //     Serial.println("PWM sens 2");
  //     valuePWM = map(DataToReceive.JoyStk2VertValue, 530, 1023, 0, 180);
  //     pwm.setPWM(servonum,0,valuePWM);//***
  //   }else
  //   if (DataToReceive.JoyStk2VertValue <= 430){
  //     Serial.println("PWM sens 1");
  //     valuePWM = map(DataToReceive.JoyStk2VertValue, 480, 0, 180, 0);
  //     pwm.setPWM(servonum,0,valuePWM);//***
  //   }else
  //   {
  //     Serial.println("PWM Stop");
  //     pwm.setPWM(servonum,0,0);//***
  //   }

  if (DataToReceive.Klaxon==1){
    myDFPlayer.play(1); //tester et voir quel son est le klaxon
    //delay(1200);//essayé en M1 et son delay ca va trop vite
   // myDFPlayer.pause();
  }
  
    
    
    
    // if (input == SwJoyStk2)
    // {
     
    //     enter = 1;
    //    // return ST_S1;

    // } 
    return ST_S1;

}



    



    // if (radio.available()) {
    // bool goodSignal = radio.testRPD(); //cette ligne là, je ne la comprends pas vraiment ?
    //   // Serial.println(goodSignal ? "Strong signal > 64dBm" : "Weak signal < 64dBm");
    //   // Serial.println("Récepteur ");
    
    // //radio.read(&ValeurJoyStk1VertAnalogRecue, sizeof(ValeurJoyStk1VertAnalogRecue)); 
    // radio.read(&DataToReceive,sizeof(DataToReceive)); //lire la totalisté des données reçues 
    // //Serial.print("Message reçu : "); 
    // //Serial.println(message);     // … et on l'affiche sur le port série !
    // // … toutes les secondes !
    // //Serial.println(DataToReceive);
    
    // digitalWrite(13,!(digitalRead(13)));
    // // Serial.print("Déplacement x: ");
    // // Serial.println(DataToReceive.JoyStk1HorValue);
    // Serial.print("Déplacement y: ");
    // Serial.println(DataToReceive.JoyStk1VertValue);

    // //horlogique=marche avant (à vérifier tout de meme )
    // //se baser sur la valeur y reçue 
    

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

//   }                             
//  }
  
  // msg_saisie = Serial.readString();

  // while (!Serial.available())
  // {
  //   Serial.println("74");
  //   //Afficher le numéro du cannal n'est pas "obligé"
  //   num_channel = msg_saisie.toInt(); // convertir un string en un integer
  //   radio.setChannel(num_channel);

  //    Serial.print("Le numero du canal est: ");
  //    Serial.println(num_channel);
  //   delay(1000);
  //   if (radio.available())
  //   {
  //     Serial.println("83");
  //     bool goodSignal = radio.testRPD(); //cette ligne là, je ne la comprends pas vraiment ?
  //     Serial.println(goodSignal ? "Strong signal > 64dBm" : "Weak signal < 64dBm");
  //     Serial.println("Récepteur ");
  //     radio.read(&message, sizeof(message)); // Si un message vient d'arriver, on le charge dans la variable "message"
  //     Serial.print("Message reçu : ");
  //     Serial.println(message); // … et on l'affiche sur le port série !
  //   }
  // }
  // On vérifie à chaque boucle si un message est arrivé

