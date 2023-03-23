//menu.h

#include "user.h"

#ifndef PGM_P
#define PGM_P const __rom char *  
#endif

// mt __flash typedef struct
typedef struct //PROGMEM
{
    unsigned char state;
    unsigned char input;
    unsigned char nextstate;
} MENU_NEXTSTATE;

typedef struct //PROGMEM
{
    unsigned char state;
    // char __flash *pText;       
    PGM_P pText;	
    char (*pFunc)(char input);
} MENU_STATE;

// Menu state machine states
#define ST_S1                             1
#define ST_S2_Bienvenu                    2
#define ST_S3_SetSound                    3 //volume du bull

// Menu text

const  char PROGMEM  MT_S0Start[]                       = "Volvo L180H";
const  char PROGMEM  MT_S2_Bienvenu[]                   = "Bienvenu:";

const   MENU_NEXTSTATE menu_nextstate[]  = {
//  STATE                       INPUT                   NEXT STATE
    {ST_S1,                       KEY_SwJoyStk1,            ST_S2_Bienvenu},
    {ST_S1,                       SwJoyStk2,                ST_S3_SetSound},


    {0,                         0,          0}
};

   
  const MENU_STATE menu_state[] PROGMEM    = {  
//  STATE                               STATE TEXT                  STATE_FUNC
    
    {ST_S1,                             MT_S0Start,                   NULL}, //quand on entre dans le menu, on ne fait rien 
    {ST_S2_Bienvenu,                   MT_S2_Bienvenu,           SetSound}, 
    
    {0,                                 NULL,                       NULL},

};
