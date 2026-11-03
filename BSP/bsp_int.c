/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                            (c) Copyright 2007-2008; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                     Micrium uC-Eval-STM32F107
*                                        Evaluation Board
*
* Filename      : bsp_int.c
* Version       : V1.00
* Programmer(s) : EHS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_INT_MODULE
#include <bsp.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  BSP_INT_SRC_NBR                                 82


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/

static CPU_FNCT_VOID  BSP_IntVectTbl[BSP_INT_SRC_NBR];


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  BSP_IntHandler     (CPU_DATA  int_id);
static  void  BSP_IntHandlerDummy(void);


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              BSP_IntClr()
*
* Description : Clear interrupt.
*
* Argument(s) : int_id      Interrupt to clear.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) An interrupt does not need to be cleared within the interrupt controller.
*********************************************************************************************************
*/

void  BSP_IntClr (CPU_DATA  int_id)
{

}


/*
*********************************************************************************************************
*                                              BSP_IntDis()
*
* Description : Disable interrupt.
*
* Argument(s) : int_id      Interrupt to disable.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntDis (CPU_DATA  int_id)
{
    if (int_id < BSP_INT_SRC_NBR) {
        CPU_IntSrcDis(int_id + 16);
    }
}


/*
*********************************************************************************************************
*                                           BSP_IntDisAll()
*
* Description : Disable ALL interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntDisAll (void)
{
    CPU_IntDis();
}


/*
*********************************************************************************************************
*                                               BSP_IntEn()
*
* Description : Enable interrupt.
*
* Argument(s) : int_id      Interrupt to enable.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntEn (CPU_DATA  int_id)
{
    if (int_id < BSP_INT_SRC_NBR) {
        CPU_IntSrcEn(int_id + 16);
    }
}


/*
*********************************************************************************************************
*                                            BSP_IntVectSet()
*
* Description : Assign ISR handler.
*
* Argument(s) : int_id      Interrupt for which vector will be set.
*
*               isr         Handler to assign
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntVectSet (CPU_DATA       int_id,
                      CPU_FNCT_VOID  isr)
{
    CPU_SR_ALLOC();


    if (int_id < BSP_INT_SRC_NBR) {
        CPU_CRITICAL_ENTER();
        BSP_IntVectTbl[int_id] = isr;
        CPU_CRITICAL_EXIT();
    }
}


/*
*********************************************************************************************************
*                                            BSP_IntPrioSet()
*
* Description : Assign ISR priority.
*
* Argument(s) : int_id      Interrupt for which vector will be set.
*
*               prio        Priority to assign
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntPrioSet (CPU_DATA    int_id,
                      CPU_INT08U  prio)
{
    CPU_SR_ALLOC();


    if (int_id < BSP_INT_SRC_NBR) {
        CPU_CRITICAL_ENTER();
        CPU_IntSrcPrioSet(int_id + 16, prio);
        CPU_CRITICAL_EXIT();
    }
}


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           INTERNAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              BSP_IntInit()
*
* Description : Initialize interrupts:
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntInit (void)
{
    CPU_DATA  int_id;
    for (int_id = 0; int_id < BSP_INT_SRC_NBR; int_id++) {
        BSP_IntVectSet(int_id, BSP_IntHandlerDummy);
    }
}

/*
*********************************************************************************************************
*                                        BSP_IntHandler****()
*
* Description : Handle an interrupt.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/



void   WWDG_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_WWDG);                }// Window WatchDog
void   PVD_IRQHandler              (void){ BSP_IntHandler(BSP_INT_ID_PVD);                 }// PVD through EXTI Line detection
void   TAMP_STAMP_IRQHandler       (void){ BSP_IntHandler(BSP_INT_ID_TAMP_STAMP);          }// Tamper and TimeStamps through the EXTI line
void   RTC_WKUP_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_RTC_WKUP);            }// RTC Wakeup through the EXTI line
void   FLASH_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_FLASH);               }// FLASH
void   RCC_IRQHandler              (void){ BSP_IntHandler(BSP_INT_ID_RCC);                 }// RCC
void   EXTI0_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_EXTI0);               }// EXTI Line0
void   EXTI1_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_EXTI1);               }// EXTI Line1
void   EXTI2_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_EXTI2);               }// EXTI Line2
void   EXTI3_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_EXTI3);               }// EXTI Line3
void   EXTI4_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_EXTI4);               }// EXTI Line4
void   DMA1_Stream0_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream0);        }// DMA1 Stream 0
void   DMA1_Stream1_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream1);        }// DMA1 Stream 1
void   DMA1_Stream2_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream1);        }// DMA1 Stream 2
void   DMA1_Stream3_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream3);        }// DMA1 Stream 3
void   DMA1_Stream4_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream4);        }// DMA1 Stream 4
//void   DMA1_Stream5_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream5);        }// DMA1 Stream 5
void   DMA1_Stream6_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream6);        }// DMA1 Stream 6
void   ADC_IRQHandler              (void){ BSP_IntHandler(BSP_INT_ID_ADC);                 }// ADC1, ADC2 and ADC3s
void   CAN1_TX_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_CAN1_TX);             }// CAN1 TX
void   CAN1_RX0_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_CAN1_RX0);            }// CAN1 RX0
void   CAN1_RX1_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_CAN1_RX1);            }// CAN1 RX1
void   CAN1_SCE_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_CAN1_SCE);            }// CAN1 SCE
void   EXTI9_5_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_EXTI9_5);             }// External Line[9:5]s
void   TIM1_BRK_TIM9_IRQHandler    (void){ BSP_IntHandler(BSP_INT_ID_TIM1_BRK_TIM9);       }// TIM1 Break and TIM9
void   TIM1_UP_TIM10_IRQHandler    (void){ BSP_IntHandler(BSP_INT_ID_TIM1_UP_TIM10);       }// TIM1 Update and TIM10
void   TIM1_TRG_COM_TIM11_IRQHandler(void){ BSP_IntHandler(BSP_INT_ID_TIM1_TRG_COM_TIM11); }// TIM1 Trigger and Commutation and TIM11
void   TIM1_CC_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_TIM1_CC);             }// TIM1 Capture Compare
void   TIM2_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_TIM2);                }// TIM2
void   TIM3_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_TIM3);                }// TIM3
void   TIM4_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_TIM4);                }// TIM4
void   I2C1_EV_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_I2C1_EV);             }// I2C1 Event
void   I2C1_ER_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_I2C1_ER);             }// I2C1 Error
void   I2C2_EV_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_I2C2_EV);             }// I2C2 Event
void   I2C2_ER_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_I2C2_ER);             }// I2C2 Error
void   SPI1_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_SPI1);                }// SPI1
void   SPI2_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_SPI2);                }// SPI2
void   USART1_IRQHandler           (void){ BSP_IntHandler(BSP_INT_ID_USART1);              }// USART1
void   USART2_IRQHandler           (void){ BSP_IntHandler(BSP_INT_ID_USART2);              }// USART2
void   USART3_IRQHandler           (void){ BSP_IntHandler(BSP_INT_ID_USART3);              }// USART3
void   EXTI15_10_IRQHandler        (void){ BSP_IntHandler(BSP_INT_ID_EXTI15_10);           }// External Line[15:10]s
void   RTC_Alarm_IRQHandler        (void){ BSP_IntHandler(BSP_INT_ID_RTC_Alarm);           }// RTC Alarm (A and B) through EXTI Line
void   OTG_FS_WKUP_IRQHandler      (void){ BSP_IntHandler(BSP_INT_ID_OTG_FS_WKUP);         }// USB OTG FS Wakeup through EXTI line
void   TIM8_BRK_TIM12_IRQHandler   (void){ BSP_IntHandler(BSP_INT_ID_TIM8_BRK_TIM12);      }// TIM8 Break and TIM12
void   TIM8_UP_TIM13_IRQHandler    (void){ BSP_IntHandler(BSP_INT_ID_TIM8_UP_TIM13);       }// TIM8 Update and TIM13
void   TIM8_TRG_COM_TIM14_IRQHandler(void){ BSP_IntHandler(BSP_INT_ID_TIM8_TRG_COM_TIM14); }// TIM8 Trigger and Commutation and TIM14
void   TIM8_CC_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_TIM8_CC);             }// TIM8 Capture Compare
void   DMA1_Stream7_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA1_Stream7);        }// DMA1 Stream7
void   FSMC_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_FSMC);                }// FSMC
void   SDIO_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_SDIO);                }// SDIO
void   TIM5_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_TIM5);                }// TIM5
void   SPI3_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_SPI3);                }// SPI3
void   UART4_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_UART4);               }// UART4
void   UART5_IRQHandler            (void){ BSP_IntHandler(BSP_INT_ID_UART5);               }// UART5
void   TIM6_DAC_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_TIM6_DAC);            }// TIM6 and DAC1&2 underrun errors
void   TIM7_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_TIM7);                }// TIM7
void   DMA2_Stream0_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream0);        }// DMA2 Stream 0
void   DMA2_Stream1_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream1);        }// DMA2 Stream 1
void   DMA2_Stream2_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream2);        }// DMA2 Stream 2
void   DMA2_Stream3_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream3);        }// DMA2 Stream 3
void   DMA2_Stream4_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream4);        }// DMA2 Stream 4
void   ETH_IRQHandler              (void){ BSP_IntHandler(BSP_INT_ID_ETH);                 }// Ethernet
void   ETH_WKUP_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_ETH_WKUP);            }// Ethernet Wakeup through EXTI line
void   CAN2_TX_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_CAN2_TX);             }// CAN2 TX
void   CAN2_RX0_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_CAN2_RX0);            }// CAN2 RX0
void   CAN2_RX1_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_CAN2_RX1);            }// CAN2 RX1
void   CAN2_SCE_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_CAN2_SCE);            }// CAN2 SCE
void   OTG_FS_IRQHandler           (void){ BSP_IntHandler(BSP_INT_ID_OTG_FS);              }// USB OTG FS
void   DMA2_Stream5_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream5);        }// DMA2 Stream 5
void   DMA2_Stream6_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream6);        }// DMA2 Stream 6
void   DMA2_Stream7_IRQHandler     (void){ BSP_IntHandler(BSP_INT_ID_DMA2_Stream7);        }// DMA2 Stream 7
void   USART6_IRQHandler           (void){ BSP_IntHandler(BSP_INT_ID_USART6);              }// USART6
void   I2C3_EV_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_I2C3_EV);             }// I2C3 event
void   I2C3_ER_IRQHandler          (void){ BSP_IntHandler(BSP_INT_ID_I2C3_ER);             }// I2C3 error
void   OTG_HS_EP1_OUT_IRQHandler   (void){ BSP_IntHandler(BSP_INT_ID_OTG_HS_EP1_OUT);      }// USB OTG HS End Point 1 Out
void   OTG_HS_EP1_IN_IRQHandler    (void){ BSP_IntHandler(BSP_INT_ID_OTG_HS_EP1_IN);       }// USB OTG HS End Point 1 In
void   OTG_HS_WKUP_IRQHandler      (void){ BSP_IntHandler(BSP_INT_ID_OTG_HS_WKUP);         }// USB OTG HS Wakeup through EXTI
void   OTG_HS_IRQHandler           (void){ BSP_IntHandler(BSP_INT_ID_OTG_HS);              }// USB OTG HS
void   DCMI_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_DCMI);                }// DCMI
void   CRYP_IRQHandler             (void){ BSP_IntHandler(BSP_INT_ID_CRYP);                }// CRYP crypto
void   HASH_RNG_IRQHandler         (void){ BSP_IntHandler(BSP_INT_ID_HASH_RNG);            }// Hash and Rng
void   FPU_IRQHandler              (void){ BSP_IntHandler(BSP_INT_ID_FPU);                 }// FPU

/**********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                          BSP_IntHandler()
*
* Description : Central interrupt handler.
*
* Argument(s) : int_id          Interrupt that will be handled.
*
* Return(s)   : none.
*
* Caller(s)   : ISR handlers.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void BSP_IntHandler (CPU_DATA int_id)
{
    CPU_FNCT_VOID  isr;
    CPU_SR_ALLOC();
    CPU_CRITICAL_ENTER();                         /* Tell the OS that we are starting an ISR            */

    OSIntEnter();

    CPU_CRITICAL_EXIT();

    if (int_id < BSP_INT_SRC_NBR)
	{
        isr = BSP_IntVectTbl[int_id];
        if (isr != (CPU_FNCT_VOID)0)
		{
            isr();
        }
    }

    OSIntExit();                                                /* Tell the OS that we are leaving the ISR            */
}



/*
*********************************************************************************************************
*                                        BSP_IntHandlerDummy()
*
* Description : Dummy interrupt handler.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_IntHandler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  BSP_IntHandlerDummy (void)
{

}
