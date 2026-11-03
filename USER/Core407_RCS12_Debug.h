#ifndef CORE407_RCS12_DEBUG_H_
#define CORE407_RCS12_DEBUG_H_

#include "RCS_Debug.h"

extern RCS_Shell_Outs Core407_RCSLIB_Debug;
void RCS_Core407_Debug_Init(RCS_PIN_USART USARTx_MAP);
RCS_Shell_Outs* Get_Core407_Debug_View(void);
#endif