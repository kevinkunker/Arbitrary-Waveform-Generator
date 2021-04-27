/* KL25Z_NVIC.h
 * 
 * 2021-01-03, sac, written
 */

#ifndef KL25Z_NVIC_H_
#define KL25Z_NVIC_H_


/* Interrupt Number Definitions
 */
typedef enum IRQn_Assignents
{
  NMI_IRQn                      = -14,             /* Non Maskable Interrupt */
  HardFault_IRQn                = -13,             /* HardFault Interrupt */
  SVCall_IRQn                   =  -5,             /* SV Call Interrupt */
  PendSV_IRQn                   =  -2,             /* Pend SV Interrupt */
  SysTick_IRQn                  =  -1,              /* System Tick Interrupt */
  DMA0_IRQn                     = 0,               /* DMA channel 0 transfer complete/error interrupt */
  DMA1_IRQn                     = 1,               /* DMA channel 1 transfer complete/error interrupt */
  DMA2_IRQn                     = 2,               /* DMA channel 2 transfer complete/error interrupt */
  DMA3_IRQn                     = 3,               /* DMA channel 3 transfer complete/error interrupt */
  Reserved20_IRQn               = 4,               /* Reserved interrupt 20 */
  FTFA_IRQn                     = 5,               /* FTFA command complete/read collision interrupt */
  LVD_LVW_IRQn                  = 6,               /* Low Voltage Detect, Low Voltage Warning */
  LLW_IRQn                      = 7,               /* Low Leakage Wakeup */
  I2C0_IRQn                     = 8,               /* I2C0 interrupt */
  I2C1_IRQn                     = 9,               /* I2C0 interrupt 25 */
  SPI0_IRQn                     = 10,              /* SPI0 interrupt */
  SPI1_IRQn                     = 11,              /* SPI1 interrupt */
  UART0_IRQn                    = 12,              /* UART0 status/error interrupt */
  UART1_IRQn                    = 13,              /* UART1 status/error interrupt */
  UART2_IRQn                    = 14,              /* UART2 status/error interrupt */
  ADC0_IRQn                     = 15,              /* ADC0 interrupt */
  CMP0_IRQn                     = 16,              /* CMP0 interrupt */
  TPM0_IRQn                     = 17,              /* TPM0 fault, overflow and channels interrupt */
  TPM1_IRQn                     = 18,              /* TPM1 fault, overflow and channels interrupt */
  TPM2_IRQn                     = 19,              /* TPM2 fault, overflow and channels interrupt */
  RTC_IRQn                      = 20,              /* RTC interrupt */
  RTC_Seconds_IRQn              = 21,              /* RTC seconds interrupt */
  PIT_IRQn                      = 22,              /* PIT timer interrupt */
  Reserved39_IRQn               = 23,              /* Reserved interrupt 39 */
  USB0_IRQn                     = 24,              /* USB0 interrupt */
  INT_DAC0_IRQn                 = 25,              /* DAC0 interrupt */
  TSI0_IRQn                     = 26,              /* TSI0 interrupt */
  MCG_IRQn                      = 27,              /* MCG interrupt */
  LPTimer_IRQn                  = 28,              /* LPTimer interrupt */
  Reserved45_IRQn               = 29,              /* Reserved interrupt 45 */
  PORTA_IRQn                    = 30,              /* Port A interrupt */
  PORTD_IRQn                    = 31               /* Port D interrupt */
} IRQn_Type;

/* Interrupt Priorities are WORD accessible only under Armv6-M                  */
/* The following MACROS handle generation of the register offset and byte masks */
#define BIT_SHIFT(IRQn)         (  ((((uint32_t)(int32_t)(IRQn))         )      &  0x03UL) * 8UL)
#define SHP_IDX(IRQn)           ( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >>    2UL)      )
#define IP_IDX(IRQn)            (   (((uint32_t)(int32_t)(IRQn))                >>    2UL)      )
#define NVIC_PRIO_BITS          2

void NVIC_EnableIRQ(IRQn_Type IRQn);
uint32_t NVIC_GetEnableIRQ(IRQn_Type IRQn);
void NVIC_DisableIRQ(IRQn_Type IRQn);
uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn);
void NVIC_SetPendingIRQ(IRQn_Type IRQn);
void NVIC_ClearPendingIRQ(IRQn_Type IRQn);
void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);
uint32_t NVIC_GetPriority(IRQn_Type IRQn);


#endif // KL25Z_NVIC_H_

