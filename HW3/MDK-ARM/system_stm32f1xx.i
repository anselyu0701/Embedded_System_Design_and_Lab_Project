#line 1 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Source\\Templates\\system_stm32f1xx.c"






























































 



 



   
  


 

#line 1 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"













































 



 



 
    






  


 



 






 

#line 96 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"



 
  
#line 109 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"



 
#line 121 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"



 



 

#line 1 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"











































 




 



 
    









 


 







 



 




 

  
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      
  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  SPI1_IRQn                   = 35,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  USBWakeUp_IRQn              = 42,      
} IRQn_Type;




 

#line 1 "../Drivers/CMSIS/Include/core_cm3.h"
 




 

























 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "../Drivers/CMSIS/Include/core_cm3.h"

















 




 



 

 













#line 120 "../Drivers/CMSIS/Include/core_cm3.h"



 







#line 162 "../Drivers/CMSIS/Include/core_cm3.h"

#line 1 "../Drivers/CMSIS/Include/core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "../Drivers/CMSIS/Include/cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 297 "../Drivers/CMSIS/Include/cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "../Drivers/CMSIS/Include/cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 

#line 731 "../Drivers/CMSIS/Include/cmsis_armcc.h"
 


#line 54 "../Drivers/CMSIS/Include/core_cmInstr.h"

 
#line 84 "../Drivers/CMSIS/Include/core_cmInstr.h"

   

#line 164 "../Drivers/CMSIS/Include/core_cm3.h"
#line 1 "../Drivers/CMSIS/Include/core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "../Drivers/CMSIS/Include/core_cmFunc.h"

 
#line 84 "../Drivers/CMSIS/Include/core_cmFunc.h"

 

#line 165 "../Drivers/CMSIS/Include/core_cm3.h"
















 
#line 203 "../Drivers/CMSIS/Include/core_cm3.h"

 






 
#line 219 "../Drivers/CMSIS/Include/core_cm3.h"

 




 












 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:27;               
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 


















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 



























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 




#line 500 "../Drivers/CMSIS/Include/core_cm3.h"

 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    

  volatile uint32_t ACTLR;                   



} SCnSCB_Type;

 



 










 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   


#line 1223 "../Drivers/CMSIS/Include/core_cm3.h"







 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1372 "../Drivers/CMSIS/Include/core_cm3.h"

#line 1381 "../Drivers/CMSIS/Include/core_cm3.h"






 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 147 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
#line 1 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\system_stm32f1xx.h"



































 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           
extern const uint8_t  AHBPrescTable[16];   
extern const uint8_t  APBPrescTable[8];    



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 148 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
#line 149 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t SR;                
  volatile uint32_t CR1;               
  volatile uint32_t CR2;               
  uint32_t  RESERVED[16];
  volatile uint32_t DR;                
} ADC_Common_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint32_t DR1;
  volatile uint32_t DR2;
  volatile uint32_t DR3;
  volatile uint32_t DR4;
  volatile uint32_t DR5;
  volatile uint32_t DR6;
  volatile uint32_t DR7;
  volatile uint32_t DR8;
  volatile uint32_t DR9;
  volatile uint32_t DR10;
  volatile uint32_t RTCCR;
  volatile uint32_t CR;
  volatile uint32_t CSR;
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;            
  volatile uint8_t  IDR;           
  uint8_t       RESERVED0;     
  uint16_t      RESERVED1;       
  volatile uint32_t CR;             
} CRC_TypeDef;




 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;





 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t OAR1;
  volatile uint32_t OAR2;
  volatile uint32_t DR;
  volatile uint32_t SR1;
  volatile uint32_t SR2;
  volatile uint32_t CCR;
  volatile uint32_t TRISE;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;            
  volatile uint32_t PR;            
  volatile uint32_t RLR;           
  volatile uint32_t SR;            
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;


} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t CRH;
  volatile uint32_t CRL;
  volatile uint32_t PRLH;
  volatile uint32_t PRLL;
  volatile uint32_t DIVH;
  volatile uint32_t DIVL;
  volatile uint32_t CNTH;
  volatile uint32_t CNTL;
  volatile uint32_t ALRH;
  volatile uint32_t ALRL;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
} SPI_TypeDef;



 
typedef struct
{
  volatile uint32_t CR1;              
  volatile uint32_t CR2;              
  volatile uint32_t SMCR;             
  volatile uint32_t DIER;             
  volatile uint32_t SR;               
  volatile uint32_t EGR;              
  volatile uint32_t CCMR1;            
  volatile uint32_t CCMR2;            
  volatile uint32_t CCER;             
  volatile uint32_t CNT;              
  volatile uint32_t PSC;              
  volatile uint32_t ARR;              
  volatile uint32_t RCR;              
  volatile uint32_t CCR1;             
  volatile uint32_t CCR2;             
  volatile uint32_t CCR3;             
  volatile uint32_t CCR4;             
  volatile uint32_t BDTR;             
  volatile uint32_t DCR;              
  volatile uint32_t DMAR;             
  volatile uint32_t OR;               
}TIM_TypeDef;




 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 
  
typedef struct
{
  volatile uint16_t EP0R;                   
  volatile uint16_t RESERVED0;                  
  volatile uint16_t EP1R;                  
  volatile uint16_t RESERVED1;                    
  volatile uint16_t EP2R;                  
  volatile uint16_t RESERVED2;                    
  volatile uint16_t EP3R;                   
  volatile uint16_t RESERVED3;                    
  volatile uint16_t EP4R;                  
  volatile uint16_t RESERVED4;                    
  volatile uint16_t EP5R;                  
  volatile uint16_t RESERVED5;                    
  volatile uint16_t EP6R;                  
  volatile uint16_t RESERVED6;                    
  volatile uint16_t EP7R;                  
  volatile uint16_t RESERVED7[17];              
  volatile uint16_t CNTR;                  
  volatile uint16_t RESERVED8;                    
  volatile uint16_t ISTR;                  
  volatile uint16_t RESERVED9;                    
  volatile uint16_t FNR;                   
  volatile uint16_t RESERVEDA;                    
  volatile uint16_t DADDR;                 
  volatile uint16_t RESERVEDB;                    
  volatile uint16_t BTABLE;                
  volatile uint16_t RESERVEDC;                    
} USB_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 
  


 











 




#line 650 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



#line 663 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"










 






 
  


   

#line 723 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"




 



 
  
  

 
    
 
 
 

 
 
 
 
 

 




 




 




 
 
 
 
 

 
#line 784 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 791 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 801 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"






 
#line 820 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
 
 
 

 




 




 




 




 




 




 




 




 




 






 
#line 892 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 900 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 917 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
 
 
 

 
#line 955 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"


 
 










 










 
#line 988 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 998 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1006 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







 
#line 1020 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







 



















 
#line 1055 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"




#line 1105 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1113 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







  
#line 1130 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1183 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"


 
#line 1204 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





#line 1218 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"






 
#line 1240 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





#line 1251 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"











 
#line 1275 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"




 
#line 1298 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





#line 1312 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"






 
#line 1334 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





#line 1345 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"











 
#line 1366 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







 





#line 1385 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

   
#line 1414 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"


 
 
 
 
 
 

 








































































































 








































































































 
#line 1682 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1732 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1782 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 1831 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1881 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1934 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 

 
#line 1945 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 1993 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 2000 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2015 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 
#line 2033 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"








 
#line 2049 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







 
#line 2067 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







 
#line 2082 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"








 
#line 2098 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 
#line 2110 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 2121 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"


 
#line 2136 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2157 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2178 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

   
#line 2199 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2220 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2234 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2255 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2276 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

   
#line 2297 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2318 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2332 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2353 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2374 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

   
#line 2395 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2416 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2430 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2451 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2472 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

   
#line 2493 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2514 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 



 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2584 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2621 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2658 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2695 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2732 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 





 





 





 





 





 





 





 





 






 
#line 2799 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 



 









 
#line 2823 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"




 




 
#line 2839 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2853 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 2869 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
#line 2887 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
 
#line 2906 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
 
#line 2925 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 






 




 




 




 
 
 
 
 

 
#line 3020 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3042 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3104 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3126 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3188 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3210 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3272 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3294 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3356 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3378 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3440 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3462 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
 
 
 

 
#line 3554 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3640 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3666 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"























 




 




 




 
 
 
 
 



 


 
#line 3731 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 



 
#line 3745 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3770 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3777 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3785 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3792 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 



 
#line 3816 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3823 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





#line 3834 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3847 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3855 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3862 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3869 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3876 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3883 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3890 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3897 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3904 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 3912 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3919 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3926 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3933 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3940 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3947 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3954 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3961 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3968 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 3975 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 




 




 




 




 




 
#line 4015 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4024 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4033 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4042 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4050 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4060 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4069 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4078 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4087 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4096 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4105 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4115 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4124 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4133 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4142 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4151 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4160 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4170 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4179 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4188 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4197 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







 




 




 




 




 
#line 4231 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"


 
 
 
 
 
 
#line 4254 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

















 
#line 4281 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4288 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4313 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4321 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





#line 4332 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





#line 4344 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







#line 4357 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4404 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4442 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4468 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 






#line 4482 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4489 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"











#line 4506 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4513 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 







#line 4533 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







#line 4547 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 






#line 4561 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4568 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"











#line 4585 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4592 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 







#line 4612 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







#line 4626 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4673 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 




 




 




 




 




 




 




 
#line 4726 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







#line 4751 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4761 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 4770 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 

 
 
 
 
 

 
#line 4794 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4814 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 




 




 




 




 




 




 




 
 
 
 
 

 




 
#line 4873 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 
#line 4886 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
 
 
 

 
#line 4904 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4913 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 
#line 4929 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 4938 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







 







 





 
 
 
 
 

 






 
#line 4985 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







#line 4998 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 










#line 5036 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 




 




 




 




 




 




 




 
#line 5090 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 5098 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 5111 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 
#line 5190 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 5231 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 5305 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 




 
 
 
 
 

 
#line 5331 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

  
#line 5363 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 

                                                                            
#line 5375 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
                                                                 

                                                                            
#line 5385 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
                                                                            
#line 5393 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 










#line 5414 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
                                                                           
















#line 5437 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 



                                                                          






#line 5458 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"










                                                                           






#line 5481 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 










#line 5502 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

















#line 5525 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 










#line 5546 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

















#line 5569 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 










#line 5590 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

















#line 5613 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 










#line 5634 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

















#line 5657 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 










#line 5678 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

















#line 5701 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 










#line 5722 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

















#line 5745 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
#line 5787 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 5819 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 5836 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 5862 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





     




 
 




 




 




 




 




 




 




 




 

 




 




 




 




 




 




 




 




 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 




 




 




 




 




 




 




 




 

 




#line 6062 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 




#line 6080 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 




#line 6098 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 




#line 6116 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 




#line 6134 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 




#line 6152 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 




#line 6170 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 




#line 6188 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 

 


#line 6204 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6216 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6228 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6240 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6252 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6264 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6276 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6288 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6300 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6312 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6324 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6336 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6348 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6360 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6372 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 


#line 6384 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"



 
 
 
 
 

 
 
#line 6425 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6454 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6504 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 6517 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 6530 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6544 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6558 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6602 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6613 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 6620 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 6627 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6656 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
#line 6674 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6685 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6699 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6713 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6730 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6741 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6755 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6769 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6786 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

   
#line 6797 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6811 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6825 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6839 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6850 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6864 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6878 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6892 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6903 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6917 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6931 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
#line 6940 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 6987 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7034 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7081 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7128 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7226 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7324 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7422 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7520 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7618 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7716 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7814 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 7912 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8010 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8108 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8206 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8304 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8402 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8500 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8598 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8696 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8794 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8892 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 8990 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9088 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9186 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9284 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9382 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9480 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9578 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9676 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9774 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9872 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
 
 
 

 
#line 9889 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 9896 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 9927 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9947 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 9973 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 




 




 




 





 
 
 
 
 

 
#line 10049 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10060 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 10076 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 



#line 10111 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 
#line 10123 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10167 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10193 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10204 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 
 
 
 
 

 
#line 10247 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 
#line 10260 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10304 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10327 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"











 
#line 10372 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10385 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 
 
 
 
 

 




#line 10420 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10434 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"







#line 10462 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
 
 
 
 
 
#line 10475 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 10485 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




#line 10500 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 







 
#line 10522 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10554 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 
#line 10567 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

#line 10586 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 




 

 
#line 10601 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10609 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10617 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10625 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"

 
#line 10633 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"





 



  



 

 









     


 


 

 
#line 10674 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
  
 





 


 


 


 


 


 
 














































































#line 10794 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"
























    




    



 


                                            



 



 



 



                                     



 



 



 



 



 


 


 







  
 
 
 
 
  
 
 

 
#line 10906 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"


 
#line 10925 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f103x6.h"




 



 





  

  
  
  
   
#line 148 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"
#line 161 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"



 



   
typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum 
{
  ERROR = 0, 
  SUCCESS = !ERROR
} ErrorStatus;



 




 



















 

#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"




































  

 







 
#line 1 "..\\Inc\\stm32f1xx_hal_conf.h"
































  

 







#line 1 "..\\Inc\\mxconstants.h"































 
 


   

 

 

 

 

 



  



  


 
#line 44 "..\\Inc\\stm32f1xx_hal_conf.h"

 
 

 


 

 
 
 
 

 

 
 

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

 


 
 

#line 91 "..\\Inc\\stm32f1xx_hal_conf.h"

 




 












 







 









 

 


      





 



 
 

 

 

 
#line 157 "..\\Inc\\stm32f1xx_hal_conf.h"

    





 

  

  

 





 



 
#line 191 "..\\Inc\\stm32f1xx_hal_conf.h"




  
 




 










 


 

#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



































 

 







 
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"




































 

 







 
#line 1 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"













































 



 



 
    
#line 227 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"


 



 
  



 
#line 49 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




































 

 







 
 
 



 








 
  


 
#line 107 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 115 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


  
  




    
   


 
#line 146 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 202 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 
  





 



 

#line 238 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 261 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
  



  
  
  


 



 
  
#line 347 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 
  
#line 364 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  



 
#line 383 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
  




 



 






















#line 428 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 435 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"










 



 
#line 459 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
   
#line 468 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 491 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 
   


 










 



 





                                              















                                                                      



                                                        


 



 






 



 

 
#line 636 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 












 
  


 
  









#line 673 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
















 

  


 















 

  


 
#line 727 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 











 
  


 


  
#line 780 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 790 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 809 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 



 























 

  


 








 



 




 



 
#line 887 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 

#line 904 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 916 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 
#line 947 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 











   
  
#line 994 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 

 



 



   



  
#line 1022 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 













 
  


 
#line 1057 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 1071 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 

 



 






 

 



 
#line 1108 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1116 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




#line 1130 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
 

   
  


 





 



 



   



 






 
   


  



 
  


  



   
   
  


 
  


 

 



 





   
  


 
#line 1219 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"









 

   


 
#line 1247 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1268 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1279 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1288 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1302 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1311 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 







 
   


 
#line 1347 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1362 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


#line 1388 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 1555 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



#line 1565 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 

#line 1579 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




  


 



 

#line 1602 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 
  
#line 1626 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 
  





 



 
  













 




 




 




 







 
  
  


 
#line 1700 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




 
#line 1744 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1758 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


   
  
  


 
  






#line 2273 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2422 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 



#line 2449 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2472 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2589 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2606 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2621 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
























#line 2667 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



#line 2678 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2711 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2729 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"












#line 2747 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2764 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 
  


 
  













#line 2813 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2828 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




#line 2868 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2894 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 

#line 2908 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 
#line 2929 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 
  


 













 




 











 



 












#line 3002 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3011 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3020 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 








#line 3053 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 



 
  
#line 3070 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 



 



 
#line 3103 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 
  


 







 

#line 50 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 51 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"

 



   
typedef enum 
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;



 
typedef enum 
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01  
} HAL_LockTypeDef;

 




























 


#line 120 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"







#line 135 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"


 
#line 157 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"



  









 


#line 189 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"



  



 


#line 206 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_def.h"








 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 



 



 



  
  
 

 
#line 75 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"


 
  


 
#line 88 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 




 






 
 


 


 


 



 
 



 



 
 



 



 



 





 
  
 


 


 


 








 



 


 






 

#line 197 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 222 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 

 



 



 
typedef struct
{
  uint32_t PLLState;      
 

  uint32_t PLLSource;     
           

  uint32_t PLLMUL;        
 
} RCC_PLLInitTypeDef;
   


 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 
} RCC_ClkInitTypeDef;



 

 


 



 






 



 







 



 





 



 






 



 







 



 





 



 






 



 







 



 






 



 






 



 
#line 401 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 
  


 








 



 






 




 





 



 




 



 
#line 459 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"


  
  








 
 




 
#line 485 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

 




 



 

 



 







 
#line 517 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 525 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 533 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 541 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"








 







 

#line 567 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 







 
#line 586 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 594 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 602 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 610 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 618 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 626 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 634 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"












 







 

#line 670 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 







 
#line 689 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 697 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 705 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 713 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 721 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 729 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 737 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 745 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 753 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

#line 760 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"







 
  






 

#line 795 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 




 
#line 810 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"




#line 820 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"






 




 
#line 839 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"





#line 851 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"







 



 










 









   





 



 





 






 




 



 






















 
#line 961 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 



 

















 
#line 1009 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"



 



 






 




 































 








 




 



 








 









 




 



  

#line 1126 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"











 








 

  

 






















 

                                                   






 




 




 





 



 




 




 
















 

















 


















 



















 





 
























 






 



 

 
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 



  



 

#line 74 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"






 



 






#line 105 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 127 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"










#line 191 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"






#line 204 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"










 

  



 

#line 242 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 
typedef struct
{
  uint32_t OscillatorType;       
 






  uint32_t HSEState;              
 
                          
  uint32_t HSEPredivValue;       
 

  uint32_t LSEState;              
 
                                          
  uint32_t HSIState;              
 

  uint32_t HSICalibrationValue;   
 
                               
  uint32_t LSIState;              
 

  RCC_PLLInitTypeDef PLL;                




} RCC_OscInitTypeDef;

#line 297 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 
typedef struct
{
  uint32_t PeriphClockSelection;      
 

  uint32_t RTCClockSelection;         
 

  uint32_t AdcClockSelection;         
 

#line 326 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"




  uint32_t UsbClockSelection;         
 


} RCC_PeriphCLKInitTypeDef;



 

 



 



 
#line 361 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 



 







 

#line 405 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"






 





 




#line 451 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 464 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 



#line 491 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 

#line 552 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 

#line 579 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 



 
#line 598 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"


 

#line 626 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 

 


 







 

#line 657 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 670 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 683 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 696 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 744 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 







 

#line 784 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 







 

#line 806 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"




#line 851 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 861 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"




#line 931 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 970 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1036 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1048 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1078 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 







 

#line 1169 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 







 

#line 1192 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"




#line 1225 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1240 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1262 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1283 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1304 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1334 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 







 

#line 1395 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 

#line 1421 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"




 








#line 1448 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"







#line 1473 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1485 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1503 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"







#line 1519 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 




 









#line 1546 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1555 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1564 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1572 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1580 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"

#line 1590 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 



  

#line 1611 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"







 





#line 1632 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"


 






 

#line 1685 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"




   








 







 




#line 1730 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"








 









 




 

#line 1864 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"


 

 


 



 

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);



 

#line 1906 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc_ex.h"



 



 
  


 
  






 

#line 1332 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_rcc.h"

 


 



 

 
void              HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency);



 



 

 
void              HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void              HAL_RCC_EnableCSS(void);
void              HAL_RCC_DisableCSS(void);
uint32_t          HAL_RCC_GetSysClockFreq(void);
uint32_t          HAL_RCC_GetHCLKFreq(void);
uint32_t          HAL_RCC_GetPCLK1Freq(void);
uint32_t          HAL_RCC_GetPCLK2Freq(void);
void              HAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void              HAL_RCC_NMI_IRQHandler(void);

 
void              HAL_RCC_CSSCallback(void);



 



 



 



 
  






 

#line 219 "..\\Inc\\stm32f1xx_hal_conf.h"


#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio.h"



 



  

 


  



  
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 
                           
  uint32_t Pull;      
 
                           
  uint32_t Speed;     
 
}GPIO_InitTypeDef;
 


 
typedef enum
{ 
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;



 


 



  



  
#line 120 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio.h"




  

     









  
#line 143 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio.h"


    



 






 
                                                    
                                                         



   






 


 


   






 
  


 


 


 







                            



#line 216 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio.h"



 


 


 






 

     





 







 

     





 







 


 
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"



 



  

 

 



  
  



 
  


 
  
#line 90 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 107 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"


  
  


 
  













 
  


 




 
  




 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 







 







 







 







 







 








 






 






 










 









 


#line 366 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 384 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 403 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 421 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"





 






 






 






 


#line 466 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 483 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"





 






 






 






 


#line 530 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 549 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 568 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 585 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 602 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 618 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 635 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 652 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 669 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 686 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 703 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 720 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 737 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 754 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 771 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 790 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"

#line 817 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"



  
  


 
  


 
#line 846 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio_ex.h"



 

 
 



 



 
void HAL_GPIOEx_ConfigEventout(uint32_t GPIO_PortSource, uint32_t GPIO_PinSource);
void HAL_GPIOEx_EnableEventout(void);
void HAL_GPIOEx_DisableEventout(void);



  



  



  



  
  






 
#line 269 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_gpio.h"



 



  
 


 



 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 

 


 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void          HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void          HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void          HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void          HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


 



  



  



  








 
#line 223 "..\\Inc\\stm32f1xx_hal_conf.h"

   
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma.h"



 



  

 



 
   


 
typedef struct
{
  uint32_t Direction;                 

 

  uint32_t PeriphInc;                 
 
                               
  uint32_t MemInc;                    
 
  
  uint32_t PeriphDataAlignment;       
 

  uint32_t MemDataAlignment;          
 
                               
  uint32_t Mode;                      


  

  uint32_t Priority;                   
 
} DMA_InitTypeDef;



   
typedef enum 
{
  DMA_MODE            = 0,       
  DMA_PRIORITY        = 1,       
  
} DMA_ControlTypeDef;



 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00,   
  HAL_DMA_STATE_READY             = 0x01,   
  HAL_DMA_STATE_READY_HALF        = 0x11,   
  HAL_DMA_STATE_BUSY              = 0x02,   
  HAL_DMA_STATE_TIMEOUT           = 0x03,   
  HAL_DMA_STATE_ERROR             = 0x04,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00,     
  HAL_DMA_HALF_TRANSFER      = 0x01,     
}HAL_DMA_LevelCompleteTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef   *Instance;                        
  
  DMA_InitTypeDef       Init;                              
  
  HAL_LockTypeDef       Lock;                               
  
  HAL_DMA_StateTypeDef  State;                            
  
  void                  *Parent;                                                         
  
  void                  (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);      
  
  void                  (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);  
  
  void                  (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);     
  
  volatile uint32_t         ErrorCode;                                                     
} DMA_HandleTypeDef;    


 

 



 



 






 



  






 



  




  



  




 



  





  



 





 



  




 



 






  




 





 



  
#line 280 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma.h"


 



 
 

 


 




 






 






 



 










 











 











 







 




 

 
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma_ex.h"



































 

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma_ex.h"



 



  

  
 
 


 
 
#line 162 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma_ex.h"


 





 
#line 179 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma_ex.h"





       
#line 193 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma_ex.h"





 
#line 207 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma_ex.h"











 













 




 


  


 



 



 




        



 
#line 367 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_dma.h"

 


 



 
 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit (DMA_HandleTypeDef *hdma);


 



 
 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, uint32_t CompleteLevel, uint32_t Timeout);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);


 



 
 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


 



 

 



 


  

 



 































  

 



 


 



  



 







 
#line 227 "..\\Inc\\stm32f1xx_hal_conf.h"

   



   








#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_cortex.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_cortex.h"



 



  
 


 

#line 94 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_cortex.h"



 

 


 




 

#line 119 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_cortex.h"


 



 





 

#line 263 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_cortex.h"



 
  

 


   



 







 






 



                        




 
#line 368 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_cortex.h"



 

 


 



   
 
void     HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void     HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void     HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void     HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void     HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



  
 



uint32_t HAL_NVIC_GetPriorityGrouping(void);
void     HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void     HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void     HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void     HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void     HAL_SYSTICK_IRQHandler(void);
void     HAL_SYSTICK_Callback(void);


 



 
                                                                               
    



 

#line 456 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_cortex.h"



 



  



 
  





 

 
#line 243 "..\\Inc\\stm32f1xx_hal_conf.h"






#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_crc.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_crc.h"



 



  

 



 



  
typedef enum
{
  HAL_CRC_STATE_RESET     = 0x00,   
  HAL_CRC_STATE_READY     = 0x01,   
  HAL_CRC_STATE_BUSY      = 0x02,   
  HAL_CRC_STATE_TIMEOUT   = 0x03,   
  HAL_CRC_STATE_ERROR     = 0x04    

}HAL_CRC_StateTypeDef;



  
typedef struct
{
  CRC_TypeDef                 *Instance;   

  HAL_LockTypeDef             Lock;        

  volatile HAL_CRC_StateTypeDef   State;       

}CRC_HandleTypeDef;



  

 
 



 




 






 







 






 




  

 



  



  

 
HAL_StatusTypeDef     HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef     HAL_CRC_DeInit (CRC_HandleTypeDef *hcrc);
void                  HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void                  HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);



  



  

 
uint32_t              HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t              HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);

  


  



  

 
HAL_CRC_StateTypeDef  HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);



  




  



  



 
  






 
#line 251 "..\\Inc\\stm32f1xx_hal_conf.h"






#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash.h"



































 

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash.h"
   


 



 
  


 



 



 















   

  


   




 
typedef enum 
{
  FLASH_PROC_NONE              = 0, 
  FLASH_PROC_PAGEERASE         = 1,
  FLASH_PROC_MASSERASE         = 2,
  FLASH_PROC_PROGRAMHALFWORD   = 3,
  FLASH_PROC_PROGRAMWORD       = 4,
  FLASH_PROC_PROGRAMDOUBLEWORD = 5
} FLASH_ProcedureTypeDef;



 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;  
  
  volatile uint32_t               DataRemaining;     

  volatile uint32_t               Address;           

  volatile uint64_t               Data;              

  HAL_LockTypeDef             Lock;              

  volatile uint32_t               ErrorCode;        
 
} FLASH_ProcessTypeDef;



 

 


   



 








 



  






 




 






 

#line 179 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash.h"


   
  
 




 
 



 






 







 




 





  
  





  







  




 





    



  





 




 
  


  

 
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"



































 

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"



 



  



 







   



 





















 





 
#line 106 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"

 






 





 








#line 136 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"

 






 
#line 152 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"

 
#line 160 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"

 






 
#line 175 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"



   

  


   



 
typedef struct
{
  uint32_t TypeErase;   
 
  
  uint32_t Banks;       
     
  
  uint32_t PageAddress; 

 
  
  uint32_t NbPages;     
 
                                                          
} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;  
 

  uint32_t WRPState;    
 

  uint32_t WRPPage;     
 

  uint32_t Banks;        
  
    
  uint8_t RDPLevel;     
 

#line 231 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"
  uint8_t USERConfig;   


 


  uint32_t DATAAddress; 
 
  
  uint8_t DATAData;     
 
} FLASH_OBProgramInitTypeDef;



 

 


   



  



  



        




         
        



 



  





 



 
#line 296 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"


 



 



  



 







 



  





 



 
 
#line 346 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"
        
       
 
#line 375 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"


 
#line 416 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"
         
        


 
 




 
#line 433 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"
       
 
#line 441 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"

 
#line 449 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"
      
 
#line 457 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"



 



 




 
  


  




 



  




  



  




 

#line 508 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"



 




 



 



  




 
#line 547 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"










 
  



 
#line 574 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"







   



 
  



 

 


 




  

#line 693 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"







  









  












 












 
#line 748 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash_ex.h"





 



 

 


 



 
 
HAL_StatusTypeDef  HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef  HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);



 



 
 
HAL_StatusTypeDef  HAL_FLASHEx_OBErase(void);
HAL_StatusTypeDef  HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void               HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);
uint32_t           HAL_FLASHEx_OBGetUserData(uint32_t DATAAdress);


 



 



 



 






 
#line 268 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_flash.h"

 


 
  


 
 
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

 
void       HAL_FLASH_IRQHandler(void);
  
void       HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void       HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);



 



 
 
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);



 



 
 
uint32_t HAL_FLASH_GetError(void);



 



 

 


 
void                    FLASH_PageErase(uint32_t PageAddress);
HAL_StatusTypeDef       FLASH_WaitForLastOperation(uint32_t Timeout);






 



 



 







 

#line 259 "..\\Inc\\stm32f1xx_hal_conf.h"






















#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_pwr.h"



































 

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_pwr.h"



 



 

 



  



 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;




 


 



  





 

 
 



  



 
#line 111 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_pwr.h"
                                                          


 



 
#line 126 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_pwr.h"



 




 





 



 





 



 





 



 





 



 







 



 

 


 















 







 





 





 





 





 






 






 






 






 





 






 







 





 





 



 

 


 






















 



 



 
  


 

 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);



 



 

 
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
 
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);

 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);



void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);


 



 



 



 








 
#line 283 "..\\Inc\\stm32f1xx_hal_conf.h"


















#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"



 



 

 


 



 
typedef struct
{
  uint32_t Mode;               
 

  uint32_t Direction;          
 

  uint32_t DataSize;           
 

  uint32_t CLKPolarity;        
 

  uint32_t CLKPhase;           
 

  uint32_t NSS;                

 

  uint32_t BaudRatePrescaler;  



 

  uint32_t FirstBit;           
 

  uint32_t TIMode;             
 

  uint32_t CRCCalculation;     
 

  uint32_t CRCPolynomial;      
 

}SPI_InitTypeDef;



 
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00,   
  HAL_SPI_STATE_READY      = 0x01,   
  HAL_SPI_STATE_BUSY       = 0x02,   
  HAL_SPI_STATE_BUSY_TX    = 0x12,   
  HAL_SPI_STATE_BUSY_RX    = 0x22,   
  HAL_SPI_STATE_BUSY_TX_RX = 0x32,   
  HAL_SPI_STATE_ERROR      = 0x03    
    
}HAL_SPI_StateTypeDef;




 
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;     

  SPI_InitTypeDef            Init;          

  uint8_t                    *pTxBuffPtr;   

  uint16_t                   TxXferSize;    
  
  uint16_t                   TxXferCount;   

  uint8_t                    *pRxBuffPtr;   

  uint16_t                   RxXferSize;    

  uint16_t                   RxXferCount;   

  DMA_HandleTypeDef          *hdmatx;       

  DMA_HandleTypeDef          *hdmarx;       

  void                       (*RxISR)(struct __SPI_HandleTypeDef * hspi);  

  void                       (*TxISR)(struct __SPI_HandleTypeDef * hspi);  

  HAL_LockTypeDef            Lock;          

  volatile HAL_SPI_StateTypeDef  State;         

  volatile uint32_t  ErrorCode;     

}SPI_HandleTypeDef;


 


 



 



  
#line 178 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"


 






 





 



 






 



 





  



 





 



 





 



 






  



 
#line 258 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"



  



 





 




 




 
  


 





 



 





 



 
#line 312 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"



 



 


 


 




  


 


 





 











 











 











 














 






 






 
#line 415 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"





 
#line 428 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"






                                                  

                                                 




                                            




 


 


 





 






 







 






 






 







 







 







 








 
#line 529 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_spi.h"





 







 






 







 






 






                






 





 

 


 

 


 
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit (SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);


 

 


 
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);


 


 


 
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t HAL_SPI_GetError(SPI_HandleTypeDef *hspi);



 



 


 


 
uint8_t SPI_ISCRCErrorValid(SPI_HandleTypeDef *hspi);



 




  



 
  






 
#line 303 "..\\Inc\\stm32f1xx_hal_conf.h"


#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"



































 

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"



 



 

 


 


 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 
} TIM_Base_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCFastMode;   

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
 
} TIM_OnePulse_InitTypeDef;




 
typedef struct
{
  uint32_t  ICPolarity;   
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 
typedef struct
{
  uint32_t EncoderMode;   
 

  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 

  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;




 
typedef struct
{
  uint32_t ClockSource;     
 
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;    
 
}TIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t ClearInputState;      
 
  uint32_t ClearInputSource;     
 
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;    
 
}TIM_ClearInputConfigTypeDef;



 
typedef struct {
  uint32_t  SlaveMode;      
 
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
 

}TIM_SlaveConfigTypeDef;



 
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00,     
  HAL_TIM_STATE_READY             = 0x01,     
  HAL_TIM_STATE_BUSY              = 0x02,     
  HAL_TIM_STATE_TIMEOUT           = 0x03,     
  HAL_TIM_STATE_ERROR             = 0x04      
}HAL_TIM_StateTypeDef;



 
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00     
}HAL_TIM_ActiveChannel;



 
typedef struct
{
  TIM_TypeDef              *Instance;      
  TIM_Base_InitTypeDef     Init;           
  HAL_TIM_ActiveChannel    Channel;        
  DMA_HandleTypeDef        *hdma[7];      
 
  HAL_LockTypeDef          Lock;           
  volatile HAL_TIM_StateTypeDef   State;          
}TIM_HandleTypeDef;



 

 


 



 





 



 




 



 






 



 







 



 





 



 
#line 369 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 





 



 







 



 






 



 




 



 





 



 
#line 511 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 





 



 
#line 535 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 
#line 550 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 
#line 569 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 
#line 586 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 







 



 






 



 





 



 




 



 






 



 




 



 




 



 






 



 




 



 




 


 




 



 
#line 709 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 







 



 




 



 
#line 746 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 







 



 






 



 




 



 
#line 804 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 
#line 829 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 
#line 843 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"


 



 






 



 

 


 


 





 

 


 













                              
#line 900 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"





















                                 


                                      




























#line 963 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"
















































#line 1019 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"










#line 1037 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"























#line 1079 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"

#line 1098 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"








 










 












 










 








 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState);


 

 


 




 






 






 






 
#line 1207 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"

 





 
#line 1225 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"















 
















 















 















 




















 




















 







 







 








 







 














 













 








 






 









 










 













 
#line 1451 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"





 




















 
















 













 













 


















 








 

 
#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim_ex.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim_ex.h"



 



  

  


 




 

typedef struct
{
                                  
  uint32_t IC1Polarity;            
 
                                                                   
  uint32_t IC1Prescaler;        
 
                                  
  uint32_t IC1Filter;           
   
  uint32_t Commutation_Delay;  
                               
} TIM_HallSensor_InitTypeDef;








  
typedef struct
{
  uint32_t OffStateRunMode;       
 
  uint32_t OffStateIDLEMode;      
 
  uint32_t LockLevel;             
                              
  uint32_t DeadTime;              
 
  uint32_t BreakState;            
 
  uint32_t BreakPolarity;         
 
  uint32_t AutomaticOutput;       
            
} TIM_BreakDeadTimeConfigTypeDef;


        
        



  
typedef struct {
  uint32_t  MasterOutputTrigger;   
  
  uint32_t  MasterSlaveMode;       
 
}TIM_MasterConfigTypeDef;



  

 





 
    


 



 



 

        
        

 

 


 



 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

  
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);


 







 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 

        
        



 
 



HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);

        
        
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef * sMasterConfig);


 



 
 
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);


 






 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);


 

        
        



  
 

 


 
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);


  
 



  



 
  







 
#line 1554 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_tim.h"

 


 



 
 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 



 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 



 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 



 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
  
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);


 



 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef* sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef * sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef * sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);


 



 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);



 



 



 



 







 
#line 307 "..\\Inc\\stm32f1xx_hal_conf.h"


#line 1 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_uart.h"



































  

 







 
#line 48 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_uart.h"



 



  

  


  




  
typedef struct
{
  uint32_t BaudRate;                  


 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 
 
  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 

 
  
  uint32_t OverSampling;              

  
}UART_InitTypeDef;



  
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00,     
  HAL_UART_STATE_READY             = 0x01,     
  HAL_UART_STATE_BUSY              = 0x02,     
  HAL_UART_STATE_BUSY_TX           = 0x12,     
  HAL_UART_STATE_BUSY_RX           = 0x22,     
  HAL_UART_STATE_BUSY_TX_RX        = 0x32,     
  HAL_UART_STATE_TIMEOUT           = 0x03,     
  HAL_UART_STATE_ERROR             = 0x04      
}HAL_UART_StateTypeDef;




   
typedef struct
{
  USART_TypeDef                 *Instance;         

  UART_InitTypeDef              Init;              

  uint8_t                       *pTxBuffPtr;       

  uint16_t                      TxXferSize;        

  uint16_t                      TxXferCount;       

  uint8_t                       *pRxBuffPtr;       

  uint16_t                      RxXferSize;        

  uint16_t                      RxXferCount;         

  DMA_HandleTypeDef             *hdmatx;           

  DMA_HandleTypeDef             *hdmarx;           

  HAL_LockTypeDef               Lock;              

  volatile HAL_UART_StateTypeDef    State;             
  
  volatile uint32_t                 ErrorCode;         

}UART_HandleTypeDef;



 

 


 



 

#line 166 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_uart.h"
  


 






 




 



 




  



  





  



  






 



  






 
    
 

  




 



 



 



   




 



 




 





 
#line 274 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_uart.h"


 










  














 



 

    
 


 







 






 



















 























 







 
#line 387 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_uart.h"








 







 







 







 

                                                 















 




















 



    














 
















 



















 



















 



















 












  






 




 


 


 









 
#line 602 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal_uart.h"





























                                





 








 



  





 

 



 
  


 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit (UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);



 



 

 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);



 



 

 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);



 



 

 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);



 



 



  



 







 
#line 311 "..\\Inc\\stm32f1xx_hal_conf.h"

























   

 
#line 354 "..\\Inc\\stm32f1xx_hal_conf.h"







 
#line 49 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"



 



  

 
 

 



 











 
   
 


 





 



#line 99 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 107 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 115 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 123 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 131 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 139 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 147 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"



 





 





 



#line 173 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"




 




#line 189 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"
   
 



 




#line 206 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 214 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 222 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 230 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"


#line 239 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 247 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"

#line 255 "..\\Drivers\\STM32F1xx_HAL_Driver\\Inc\\stm32f1xx_hal.h"



 



 

 



 



 

 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void              HAL_MspInit(void);
void              HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);



 



 

 
void              HAL_IncTick(void);
void              HAL_Delay(volatile uint32_t Delay);
uint32_t          HAL_GetTick(void);
void              HAL_SuspendTick(void);
void              HAL_ResumeTick(void);
uint32_t          HAL_GetHalVersion(void);
uint32_t          HAL_GetREVID(void);
uint32_t          HAL_GetDEVID(void);
void              HAL_DBGMCU_EnableDBGSleepMode(void);
void              HAL_DBGMCU_DisableDBGSleepMode(void);
void              HAL_DBGMCU_EnableDBGStopMode(void);
void              HAL_DBGMCU_DisableDBGStopMode(void);
void              HAL_DBGMCU_EnableDBGStandbyMode(void);
void              HAL_DBGMCU_DisableDBGStandbyMode(void);



 



 




  



  
  






 
#line 219 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Include\\stm32f1xx.h"










 



 
  



 
#line 78 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Source\\Templates\\system_stm32f1xx.c"



 



 



 



 











  





  
 






 



 



 



 



 



  uint32_t SystemCoreClock         = 72000000;         


const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] =  {0, 0, 0, 0, 1, 2, 3, 4};



 



 









 



 








 
void SystemInit (void)
{
   
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CR |= (uint32_t)0x00000001;

   

  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR &= (uint32_t)0xF8FF0000;



  
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CR &= (uint32_t)0xFEF6FFFF;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CR &= (uint32_t)0xFFFBFFFF;

   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR &= (uint32_t)0xFF80FFFF;

#line 213 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Source\\Templates\\system_stm32f1xx.c"
   
  ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CIR = 0x009F0000;

    









  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR = ((uint32_t)0x08000000) | 0x0;  

}



































 
void SystemCoreClockUpdate (void)
{
  uint32_t tmp = 0, pllmull = 0, pllsource = 0;








    
   
  tmp = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & (0x3U << (2U));
  
  switch (tmp)
  {
    case 0x00:   
      SystemCoreClock = ((uint32_t)8000000);
      break;
    case 0x04:   
      SystemCoreClock = ((uint32_t)8000000);
      break;
    case 0x08:   

       
      pllmull = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & (0xFU << (18U));
      pllsource = ((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & (0x1U << (16U));
      

      pllmull = ( pllmull >> 18) + 2;
      
      if (pllsource == 0x00)
      {
         
        SystemCoreClock = (((uint32_t)8000000) >> 1) * pllmull;
      }
      else
      {





         
        if ((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & (0x1U << (17U))) != (uint32_t)RESET)
        { 
          SystemCoreClock = (((uint32_t)8000000) >> 1) * pllmull;
        }
        else
        {
          SystemCoreClock = ((uint32_t)8000000) * pllmull;
        }

      }
#line 359 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Source\\Templates\\system_stm32f1xx.c"
      break;

    default:
      SystemCoreClock = ((uint32_t)8000000);
      break;
  }
  
   
   
  tmp = AHBPrescTable[((((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->CFGR & (0xFU << (4U))) >> 4)];
   
  SystemCoreClock >>= tmp;  
}

#line 436 "..\\Drivers\\CMSIS\\Device\\ST\\STM32F1xx\\Source\\Templates\\system_stm32f1xx.c"



 



 
  


     
 
