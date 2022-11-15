/*
 * File:   oscilador.c
 * Author: pablo
 *
 * Created on October 21, 2022, 9:02 AM
 */

#include "oscilador.h"


//******************************************************************************
// Función para configurar PWM
//******************************************************************************
void setupINTOSC(uint8_t IRCF){
    
    
    if(IRCF == 7){
        OSCCONbits.IRCF = 0b111;       // 8 MHz
    }
    if(IRCF == 6){
        OSCCONbits.IRCF = 0b110;       // 4 MHz
    }
    if(IRCF == 3){
        OSCCONbits.IRCF = 0b011;       // 500 KHz
    }
    
    OSCCONbits.SCS = 1;
}

