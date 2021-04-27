/* KL25Z_NVIC.c
 * 
 * 2021-01-03, sac, adapted from the CMSIS-CORE NVIC functions
 */

#include <stdint.h>
#include "derivative.h"
#include "KL25Z_NVIC.h"

/*
 Enable Interrupt
 Enables a device specific interrupt in the NVIC interrupt controller.
 IRQn  Device specific interrupt number.
       IRQn must not be negative.
 */
void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    NVIC_ISER = (uint32_t)(1UL << (((uint32_t)(IRQn)) & 0x1FUL));
}

/*
  Get Interrupt Enable status
  Returns a device specific interrupt enable status from the NVIC interrupt controller.
  IRQn  Device specific interrupt number.
  Returns      0  Interrupt is not enabled.
               1  Interrupt is enabled.
  RQn must not be negative.
 */
uint32_t NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
    return((uint32_t)(((NVIC_ISER & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}


/*
  Disable Interrupt
  Disables a device specific interrupt in the NVIC interrupt controller.
  IRQn  Device specific interrupt number.
        IRQn must not be negative.
 */
void NVIC_DisableIRQ(IRQn_Type IRQn)
{
    NVIC_ICER = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
 }


/**
  Get Pending Interrupt
  Reads the NVIC pending register and returns the pending bit for the specified device specific interrupt.
  IRQn  Device specific interrupt number.
  Return             0  Interrupt status is not pending.
                     1  Interrupt status is pending.
  IRQn must not be negative.
 */
uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
    return((uint32_t)(((NVIC_ISPR & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}


/*
  Set Pending Interrupt
  Sets the pending bit of a device specific interrupt in the NVIC pending register.
  IRQn  Device specific interrupt number.
  IRQn must not be negative.
 */
void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
    NVIC_ISPR = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
}


/*
  Clear Pending Interrupt
  Clears the pending bit of a device specific interrupt in the NVIC pending register.
  IRQn  Device specific interrupt number.
  IRQn must not be negative.
 */
void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
    NVIC_ICPR = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
}


/*
  Set Interrupt Priority
  Sets the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  IRQn  Interrupt number.
  priority  Priority to set.
  The priority cannot be set for every processor exception.
 */
void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC_IP(IP_IDX(IRQn))  = ((uint32_t)(NVIC_IP(IP_IDX(IRQn))  & ~(0xFFUL << BIT_SHIFT(IRQn))) |
       (((priority << (8U - NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) << BIT_SHIFT(IRQn)));
  }
  else if (  ((int32_t)(IRQn) >= -4)  &&  ((int32_t)(IRQn) <= -1)  )
  {
	    SCB_SHPR3 =	(  ((uint32_t)(SCB_SHPR3)) & ~(0xFFUL << (BIT_SHIFT(IRQn)))  )
	    		|
	         ((priority << (8U - NVIC_PRIO_BITS)) & ((uint32_t)(0xFFUL) << (BIT_SHIFT(IRQn))));
  }
  else
  {
    SCB_SHPR2 = (  ((uint32_t)(SCB_SHPR2)) & ~(0xFFUL << (BIT_SHIFT(IRQn))) ) |
       ((priority << (8U - NVIC_PRIO_BITS)) & ((uint32_t)(0xFFUL) << (BIT_SHIFT(IRQn))));
  }
}

/*
  Get Interrupt Priority
  Reads the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  IRQn  Interrupt number.
  Returns    Interrupt Priority.
             Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((NVIC_IP(IP_IDX(IRQn)) >> BIT_SHIFT(IRQn) ) & (uint32_t)0xFFUL) >> (8U - NVIC_PRIO_BITS)));
  }
  else if (  ((int32_t)(IRQn) >= -4)  &&  ((int32_t)(IRQn) <= -1)  )
  {
    return    ((uint32_t)SCB_SHPR3 >> (BIT_SHIFT(IRQn))) &  ((uint32_t)0xFFUL >> (8U - NVIC_PRIO_BITS));
  }
  else
  {
    return    ((uint32_t)SCB_SHPR2 >> (BIT_SHIFT(IRQn))) &  ((uint32_t)0xFFUL >> (8U - NVIC_PRIO_BITS));
  }
}
