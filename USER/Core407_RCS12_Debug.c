#include "Core407_RCS12_Debug.h"

RCS_Shell_Outs Core407_RCSLIB_Debug;

void RCS_Core407_Debug_Init(RCS_PIN_USART USARTx_MAP)
{
	RCS_Shell_Init(&Core407_RCSLIB_Debug,USARTx_MAP,SHELL_TYPE_LOG);
}

RCS_Shell_Outs* Get_Core407_Debug_View(void)
{
	return &Core407_RCSLIB_Debug;
}