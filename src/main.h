#ifndef Main_
#define Main_

#include <avr/pgmspace.h>
//#include "AnalogEvent.h"
// #include "ButtonEvent.h"

//Revisions number
#define SWHIGH  0
#define SWLOW   7
// mt
#define SWLOWLOW 2

// main.h

void Initialization(void);
unsigned char StateMachine(char state, unsigned char stimuli);


void SetTransition (char _tranition);
char GetTransition (void);


char SetSound(char input);
char ViewPourcentage(char input);
// char PositionRoue(char input);
// char VitesseMise(char input);
// char EtatMoteur(char input);
//char Arret(char input);

//envoie vers le Bull
char AnalyzeData(char input);
char Klaxon(char input);
char ClignoDroite(char input);
char ClignoGauche(char input);

void Delay(unsigned int millisec);
char Revision(char input);
// char BlinkLED(char input);
// char ReadADC_Channel(char input);


#define BOOL    char

#define FALSE   0
#define TRUE    (!FALSE)
//#define NULL    0

#define AUTO    3

// Macro definitions
//mtA - 
// sbi and cbi are not longer supported by the avr-libc
// to avoid version-conflicts the macro-names have been 
// changed to sbiBF/cbiBF "everywhere"
#define sbiBF(port,bit)  (port |= (1<<bit))   //set bit in port
#define cbiBF(port,bit)  (port &= ~(1<<bit))  //clear bit in port
//mtE


#endif //Main_




