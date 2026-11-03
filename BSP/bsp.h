#ifndef _BSP_H_
#define	_BSP_H_

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <cpu.h>
#include <cpu_core.h>

#include <lib_ascii.h>
#include <lib_def.h>
#include <lib_mem.h>
#include <lib_str.h>

#include <stm32f4xx.h>
#include <ucos_ii.h>
#include <app_cfg.h>

/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/


#define  BSP_INT_ID_WWDG                     0       /*!< Window WatchDog Interrupt                                         */
#define  BSP_INT_ID_PVD                      1      /*!< PVD through EXTI Line detection Interrupt */
#define  BSP_INT_ID_TAMP_STAMP               2       /*!< Tamper and TimeStamp interrupts through the EXTI line             */
#define  BSP_INT_ID_RTC_WKUP                 3       /*!< RTC Wakeup interrupt through the EXTI line                        */
#define  BSP_INT_ID_FLASH                    4       /*!< FLASH global Interrupt                                            */
#define  BSP_INT_ID_RCC                      5       /*!< RCC global Interrupt                                              */
#define  BSP_INT_ID_EXTI0                    6       /*!< EXTI Line0 Interrupt                                              */
#define  BSP_INT_ID_EXTI1                    7       /*!< EXTI Line1 Interrupt                                              */
#define  BSP_INT_ID_EXTI2                    8       /*!< EXTI Line2 Interrupt                                              */
#define  BSP_INT_ID_EXTI3                    9       /*!< EXTI Line3 Interrupt                                              */
#define  BSP_INT_ID_EXTI4                    10      /*!< EXTI Line4 Interrupt                                              */
#define  BSP_INT_ID_DMA1_Stream0             11      /*!< DMA1 Stream 0 global Interrupt                                    */
#define  BSP_INT_ID_DMA1_Stream1             12      /*!< DMA1 Stream 1 global Interrupt                                    */
#define  BSP_INT_ID_DMA1_Stream2             13      /*!< DMA1 Stream 2 global Interrupt                                    */
#define  BSP_INT_ID_DMA1_Stream3             14      /*!< DMA1 Stream 3 global Interrupt                                    */
#define  BSP_INT_ID_DMA1_Stream4             15      /*!< DMA1 Stream 4 global Interrupt                                    */
#define  BSP_INT_ID_DMA1_Stream5             16      /*!< DMA1 Stream 5 global Interrupt                                    */
#define  BSP_INT_ID_DMA1_Stream6             17      /*!< DMA1 Stream 6 global Interrupt                                    */
#define  BSP_INT_ID_ADC                      18      /*!< ADC1  ADC2 and ADC3 global Interrupts                             */
#define  BSP_INT_ID_CAN1_TX                  19      /*!< CAN1 TX Interrupt                                                 */
#define  BSP_INT_ID_CAN1_RX0                 20      /*!< CAN1 RX0 Interrupt                                                */
#define  BSP_INT_ID_CAN1_RX1                 21      /*!< CAN1 RX1 Interrupt                                                */
#define  BSP_INT_ID_CAN1_SCE                 22      /*!< CAN1 SCE Interrupt                                                */
#define  BSP_INT_ID_EXTI9_5                  23      /*!< External Line[9:5] Interrupts                                     */
#define  BSP_INT_ID_TIM1_BRK_TIM9            24      /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
#define  BSP_INT_ID_TIM1_UP_TIM10            25      /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
#define  BSP_INT_ID_TIM1_TRG_COM_TIM11       26      /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
#define  BSP_INT_ID_TIM1_CC                  27      /*!< TIM1 Capture Compare Interrupt                                    */
#define  BSP_INT_ID_TIM2                     28      /*!< TIM2 global Interrupt                                             */
#define  BSP_INT_ID_TIM3                     29      /*!< TIM3 global Interrupt                                             */
#define  BSP_INT_ID_TIM4                     30      /*!< TIM4 global Interrupt                                             */
#define  BSP_INT_ID_I2C1_EV                  31      /*!< I2C1 Event Interrupt                                              */
#define  BSP_INT_ID_I2C1_ER                  32      /*!< I2C1 Error Interrupt                                              */
#define  BSP_INT_ID_I2C2_EV                  33      /*!< I2C2 Event Interrupt                                              */
#define  BSP_INT_ID_I2C2_ER                  34      /*!< I2C2 Error Interrupt                                              */
#define  BSP_INT_ID_SPI1                     35      /*!< SPI1 global Interrupt                                             */
#define  BSP_INT_ID_SPI2                     36      /*!< SPI2 global Interrupt                                             */
#define  BSP_INT_ID_USART1                   37      /*!< USART1 global Interrupt                                           */
#define  BSP_INT_ID_USART2                   38      /*!< USART2 global Interrupt                                           */
#define  BSP_INT_ID_USART3                   39      /*!< USART3 global Interrupt                                           */
#define  BSP_INT_ID_EXTI15_10                40      /*!< External Line[15:10] Interrupts                                   */
#define  BSP_INT_ID_RTC_Alarm                41      /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
#define  BSP_INT_ID_OTG_FS_WKUP              42      /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
#define  BSP_INT_ID_TIM8_BRK_TIM12           43      /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
#define  BSP_INT_ID_TIM8_UP_TIM13            44      /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
#define  BSP_INT_ID_TIM8_TRG_COM_TIM14       45      /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
#define  BSP_INT_ID_TIM8_CC                  46      /*!< TIM8 Capture Compare Interrupt                                    */
#define  BSP_INT_ID_DMA1_Stream7             47      /*!< DMA1 Stream7 Interrupt                                            */
#define  BSP_INT_ID_FSMC                     48      /*!< FSMC global Interrupt                                             */
#define  BSP_INT_ID_SDIO                     49      /*!< SDIO global Interrupt                                             */
#define  BSP_INT_ID_TIM5                     50      /*!< TIM5 global Interrupt                                             */
#define  BSP_INT_ID_SPI3                     51      /*!< SPI3 global Interrupt                                             */
#define  BSP_INT_ID_UART4                    52      /*!< UART4 global Interrupt                                            */
#define  BSP_INT_ID_UART5                    53      /*!< UART5 global Interrupt                                            */
#define  BSP_INT_ID_TIM6_DAC                 54      /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
#define  BSP_INT_ID_TIM7                     55      /*!< TIM7 global interrupt                                             */
#define  BSP_INT_ID_DMA2_Stream0             56      /*!< DMA2 Stream 0 global Interrupt                                    */
#define  BSP_INT_ID_DMA2_Stream1             57      /*!< DMA2 Stream 1 global Interrupt                                    */
#define  BSP_INT_ID_DMA2_Stream2             58      /*!< DMA2 Stream 2 global Interrupt                                    */
#define  BSP_INT_ID_DMA2_Stream3             59      /*!< DMA2 Stream 3 global Interrupt                                    */
#define  BSP_INT_ID_DMA2_Stream4             60      /*!< DMA2 Stream 4 global Interrupt                                    */
#define  BSP_INT_ID_ETH                      61      /*!< Ethernet global Interrupt                                         */
#define  BSP_INT_ID_ETH_WKUP                 62      /*!< Ethernet Wakeup through EXTI line Interrupt                       */
#define  BSP_INT_ID_CAN2_TX                  63      /*!< CAN2 TX Interrupt                                                 */
#define  BSP_INT_ID_CAN2_RX0                 64      /*!< CAN2 RX0 Interrupt                                                */
#define  BSP_INT_ID_CAN2_RX1                 65      /*!< CAN2 RX1 Interrupt                                                */
#define  BSP_INT_ID_CAN2_SCE                 66      /*!< CAN2 SCE Interrupt                                                */
#define  BSP_INT_ID_OTG_FS                   67      /*!< USB OTG FS global Interrupt                                       */
#define  BSP_INT_ID_DMA2_Stream5             68      /*!< DMA2 Stream 5 global interrupt                                    */
#define  BSP_INT_ID_DMA2_Stream6             69      /*!< DMA2 Stream 6 global interrupt                                    */
#define  BSP_INT_ID_DMA2_Stream7             70      /*!< DMA2 Stream 7 global interrupt                                    */
#define  BSP_INT_ID_USART6                   71      /*!< USART6 global interrupt                                           */
#define  BSP_INT_ID_I2C3_EV                  72      /*!< I2C3 event interrupt                                              */
#define  BSP_INT_ID_I2C3_ER                  73      /*!< I2C3 error interrupt                                              */
#define  BSP_INT_ID_OTG_HS_EP1_OUT           74      /*!< USB OTG HS End Point 1 Out global interrupt                       */
#define  BSP_INT_ID_OTG_HS_EP1_IN            75      /*!< USB OTG HS End Point 1 In global interrupt                        */
#define  BSP_INT_ID_OTG_HS_WKUP              76      /*!< USB OTG HS Wakeup through EXTI interrupt                          */
#define  BSP_INT_ID_OTG_HS                   77      /*!< USB OTG HS global interrupt                                       */
#define  BSP_INT_ID_DCMI                     78      /*!< DCMI global interrupt                                             */
#define  BSP_INT_ID_CRYP                     79      /*!< CRYP crypto global interrupt                                      */
#define  BSP_INT_ID_HASH_RNG                 80       /*!< Hash and Rng global interrupt                                     */
#define  BSP_INT_ID_FPU                      81      /*!< FPU global interrupt        */
/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void         BSP_Init                    (void);

void 		 SysTick_init(void);

void         BSP_IntDisAll               (void);

CPU_INT32U   BSP_CPU_ClkFreq             (void);

/*
*********************************************************************************************************
*                                     PERIPHERAL POWER/CLOCK SERVICES
*********************************************************************************************************
*/

CPU_INT32U   BSP_PeriphClkFreqGet        (CPU_DATA       pwr_clk_id);

void         BSP_PeriphEn                (CPU_DATA       pwr_clk_id);

void         BSP_PeriphDis               (CPU_DATA       pwr_clk_id);
/*
*********************************************************************************************************
*                                           INTERRUPT SERVICES
*********************************************************************************************************
*/

void         BSP_IntInit                 (void);

void         BSP_IntEn                   (CPU_DATA       int_id);

void         BSP_IntDis                  (CPU_DATA       int_id);

void         BSP_IntClr                  (CPU_DATA       int_id);

void         BSP_IntVectSet              (CPU_DATA       int_id,
                                          CPU_FNCT_VOID  isr);

void         BSP_IntPrioSet              (CPU_DATA       int_id,
                                          CPU_INT08U     prio);

/*
*********************************************************************************************************
*                                              STATUS INPUTS
*********************************************************************************************************
*/

CPU_BOOLEAN  BSP_StatusRd                (CPU_INT08U  id);


void   WWDG_IRQHandler(void) ;                  // Window WatchDog
void   PVD_IRQHandler(void) ;                   // PVD through EXTI Line detection
void   TAMP_STAMP_IRQHandler(void) ;            // Tamper and TimeStamps through the EXTI line
void   RTC_WKUP_IRQHandler(void) ;              // RTC Wakeup through the EXTI line
void   FLASH_IRQHandler(void) ;                 // FLASH
void   RCC_IRQHandler(void) ;                   // RCC
void   EXTI0_IRQHandler(void) ;                 // EXTI Line0
void   EXTI1_IRQHandler(void) ;                 // EXTI Line1
void   EXTI2_IRQHandler(void) ;                 // EXTI Line2
void   EXTI3_IRQHandler(void) ;                 // EXTI Line3
void   EXTI4_IRQHandler(void) ;                 // EXTI Line4
void   DMA1_Stream0_IRQHandler(void) ;          // DMA1 Stream 0
void   DMA1_Stream1_IRQHandler(void) ;          // DMA1 Stream 1
void   DMA1_Stream2_IRQHandler(void) ;          // DMA1 Stream 2
void   DMA1_Stream3_IRQHandler(void) ;          // DMA1 Stream 3
void   DMA1_Stream4_IRQHandler(void) ;          // DMA1 Stream 4
void   DMA1_Stream5_IRQHandler(void) ;          // DMA1 Stream 5
void   DMA1_Stream6_IRQHandler(void) ;          // DMA1 Stream 6
void   ADC_IRQHandler(void) ;                   // ADC1, ADC2 and ADC3s
void   CAN1_TX_IRQHandler(void) ;               // CAN1 TX
void   CAN1_RX0_IRQHandler(void) ;              // CAN1 RX0
void   CAN1_RX1_IRQHandler(void) ;              // CAN1 RX1
void   CAN1_SCE_IRQHandler(void) ;              // CAN1 SCE
void   EXTI9_5_IRQHandler(void) ;               // External Line[9:5]s
void   TIM1_BRK_TIM9_IRQHandler(void) ;         // TIM1 Break and TIM9
void   TIM1_UP_TIM10_IRQHandler(void) ;         // TIM1 Update and TIM10
void   TIM1_TRG_COM_TIM11_IRQHandler(void) ;    // TIM1 Trigger and Commutation and TIM11
void   TIM1_CC_IRQHandler(void) ;               // TIM1 Capture Compare
void   TIM2_IRQHandler(void) ;                  // TIM2
void   TIM3_IRQHandler(void) ;                  // TIM3
void   TIM4_IRQHandler(void) ;                  // TIM4
void   I2C1_EV_IRQHandler(void) ;               // I2C1 Event
void   I2C1_ER_IRQHandler(void) ;               // I2C1 Error
void   I2C2_EV_IRQHandler(void) ;               // I2C2 Event
void   I2C2_ER_IRQHandler(void) ;               // I2C2 Error
void   SPI1_IRQHandler(void) ;                  // SPI1
void   SPI2_IRQHandler(void) ;                  // SPI2
void   USART1_IRQHandler(void) ;                // USART1
void   USART2_IRQHandler(void) ;                // USART2
void   USART3_IRQHandler(void) ;                // USART3
void   EXTI15_10_IRQHandler(void) ;             // External Line[15:10]s
void   RTC_Alarm_IRQHandler(void) ;             // RTC Alarm (A and B) through EXTI Line
void   OTG_FS_WKUP_IRQHandler(void) ;           // USB OTG FS Wakeup through EXTI line
void   TIM8_BRK_TIM12_IRQHandler(void) ;        // TIM8 Break and TIM12
void   TIM8_UP_TIM13_IRQHandler(void) ;         // TIM8 Update and TIM13
void   TIM8_TRG_COM_TIM14_IRQHandler(void) ;    // TIM8 Trigger and Commutation and TIM14
void   TIM8_CC_IRQHandler(void) ;               // TIM8 Capture Compare
void   DMA1_Stream7_IRQHandler(void) ;          // DMA1 Stream7
void   FSMC_IRQHandler(void) ;                  // FSMC
void   SDIO_IRQHandler(void) ;                  // SDIO
void   TIM5_IRQHandler(void) ;                  // TIM5
void   SPI3_IRQHandler(void) ;                  // SPI3
void   UART4_IRQHandler(void) ;                 // UART4
void   UART5_IRQHandler(void) ;                 // UART5
void   TIM6_DAC_IRQHandler(void) ;              // TIM6 and DAC1&2 underrun errors
void   TIM7_IRQHandler(void) ;                  // TIM7
void   DMA2_Stream0_IRQHandler(void) ;          // DMA2 Stream 0
void   DMA2_Stream1_IRQHandler(void) ;          // DMA2 Stream 1
void   DMA2_Stream2_IRQHandler(void) ;          // DMA2 Stream 2
void   DMA2_Stream3_IRQHandler(void) ;          // DMA2 Stream 3
void   DMA2_Stream4_IRQHandler(void) ;          // DMA2 Stream 4
void   ETH_IRQHandler(void) ;                   // Ethernet
void   ETH_WKUP_IRQHandler(void) ;              // Ethernet Wakeup through EXTI line
void   CAN2_TX_IRQHandler(void) ;               // CAN2 TX
void   CAN2_RX0_IRQHandler(void) ;              // CAN2 RX0
void   CAN2_RX1_IRQHandler(void) ;              // CAN2 RX1
void   CAN2_SCE_IRQHandler(void) ;              // CAN2 SCE
void   OTG_FS_IRQHandler(void) ;                // USB OTG FS
void   DMA2_Stream5_IRQHandler(void) ;          // DMA2 Stream 5
void   DMA2_Stream6_IRQHandler(void) ;          // DMA2 Stream 6
void   DMA2_Stream7_IRQHandler(void) ;          // DMA2 Stream 7
void   USART6_IRQHandler(void) ;                // USART6
void   I2C3_EV_IRQHandler(void) ;               // I2C3 event
void   I2C3_ER_IRQHandler(void) ;               // I2C3 error
void   OTG_HS_EP1_OUT_IRQHandler(void) ;        // USB OTG HS End Point 1 Out
void   OTG_HS_EP1_IN_IRQHandler(void) ;         // USB OTG HS End Point 1 In
void   OTG_HS_WKUP_IRQHandler(void) ;           // USB OTG HS Wakeup through EXTI
void   OTG_HS_IRQHandler(void) ;                // USB OTG HS
void   DCMI_IRQHandler(void) ;                  // DCMI
void   CRYP_IRQHandler(void) ;                  // CRYP crypto
void   HASH_RNG_IRQHandler(void) ;              // Hash and Rng
void   FPU_IRQHandler(void) ;                   // FPU


#endif
