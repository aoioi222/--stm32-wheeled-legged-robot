#ifndef CUP23_FSM_RND1_H_
#define CUP23_FSM_RND1_H_

#include "rcs.h"

typedef struct{
    uint8_t blink_freq;
    uint8_t status;
}context_var_t;


void LED1_Blink(void* ipt);
void LED2_Blink(void* ipt);
void LED3_Blink(void* ipt);
void LED4_Blink(void* ipt);
void LED_Change_Freq(void* ipt);
void Reset_Context_StatusFlag(void* ipt);
void FSM_Blink_Main(void);
void FSM_Blink_Init(void);

#endif