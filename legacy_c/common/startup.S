/* Minimal Cortex-M3 startup for STM32F103xB.
 *
 * Shared between bootloader and app — the linker script chooses where
 * the vector table lives (0x08000000 for bootloader, 0x08002400 for app).
 *
 * The reset handler:
 *   1. copies .data (LMA -> VMA)
 *   2. zeros .bss
 *   3. calls SystemInit (CMSIS) — we do nothing important there
 *   4. calls main()
 *   5. spins on return (main should never return)
 */

    .syntax unified
    .cpu    cortex-m3
    .thumb

/* ------- linker-provided symbols ------- */
    .extern _estack
    .extern _sdata, _edata, _etext
    .extern _sbss,  _ebss
    .extern main
    .extern SystemInit

/* ============= Vector table ============= */
    .section .isr_vector, "a", %progbits
    .type    g_pfnVectors, %object
    .global  g_pfnVectors
g_pfnVectors:
    .word   _estack                /*  0: initial SP */
    .word   Reset_Handler          /*  1: reset */
    .word   NMI_Handler            /*  2: NMI */
    .word   HardFault_Handler      /*  3: hard fault */
    .word   MemManage_Handler      /*  4 */
    .word   BusFault_Handler       /*  5 */
    .word   UsageFault_Handler     /*  6 */
    .word   0                      /*  7 reserved */
    .word   0                      /*  8 reserved */
    .word   0                      /*  9 reserved */
    .word   0                      /* 10 reserved */
    .word   SVC_Handler            /* 11 */
    .word   DebugMon_Handler       /* 12 */
    .word   0                      /* 13 reserved */
    .word   PendSV_Handler         /* 14 */
    .word   SysTick_Handler        /* 15 */

    /* External interrupts — STM32F103xB (medium-density) */
    .word   WWDG_IRQHandler                  /* 16 */
    .word   PVD_IRQHandler                   /* 17 */
    .word   TAMPER_IRQHandler                /* 18 */
    .word   RTC_IRQHandler                   /* 19 */
    .word   FLASH_IRQHandler                 /* 20 */
    .word   RCC_IRQHandler                   /* 21 */
    .word   EXTI0_IRQHandler                 /* 22 */
    .word   EXTI1_IRQHandler                 /* 23 */
    .word   EXTI2_IRQHandler                 /* 24 */
    .word   EXTI3_IRQHandler                 /* 25 */
    .word   EXTI4_IRQHandler                 /* 26 */
    .word   DMA1_Channel1_IRQHandler         /* 27 */
    .word   DMA1_Channel2_IRQHandler         /* 28 */
    .word   DMA1_Channel3_IRQHandler         /* 29 */
    .word   DMA1_Channel4_IRQHandler         /* 30 */
    .word   DMA1_Channel5_IRQHandler         /* 31 */
    .word   DMA1_Channel6_IRQHandler         /* 32 */
    .word   DMA1_Channel7_IRQHandler         /* 33 */
    .word   ADC1_2_IRQHandler                /* 34 */
    .word   USB_HP_CAN1_TX_IRQHandler        /* 35 */
    .word   USB_LP_CAN1_RX0_IRQHandler       /* 36 */
    .word   CAN1_RX1_IRQHandler              /* 37 */
    .word   CAN1_SCE_IRQHandler              /* 38 */
    .word   EXTI9_5_IRQHandler               /* 39 */
    .word   TIM1_BRK_IRQHandler              /* 40 */
    .word   TIM1_UP_IRQHandler               /* 41 */
    .word   TIM1_TRG_COM_IRQHandler          /* 42 */
    .word   TIM1_CC_IRQHandler               /* 43 */
    .word   TIM2_IRQHandler                  /* 44 */
    .word   TIM3_IRQHandler                  /* 45 */
    .word   TIM4_IRQHandler                  /* 46 */
    .word   I2C1_EV_IRQHandler               /* 47 */
    .word   I2C1_ER_IRQHandler               /* 48 */
    .word   I2C2_EV_IRQHandler               /* 49 */
    .word   I2C2_ER_IRQHandler               /* 50 */
    .word   SPI1_IRQHandler                  /* 51 */
    .word   SPI2_IRQHandler                  /* 52 */
    .word   USART1_IRQHandler                /* 53 */
    .word   USART2_IRQHandler                /* 54 */
    .word   USART3_IRQHandler                /* 55 */
    .word   EXTI15_10_IRQHandler             /* 56 */
    .word   RTC_Alarm_IRQHandler             /* 57 */
    .word   USBWakeUp_IRQHandler             /* 58 */
    .size   g_pfnVectors, .-g_pfnVectors

/* ============= Reset_Handler ============= */
    .section .text.Reset_Handler, "ax", %progbits
    .thumb_func
    .global  Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    ldr     r0, =_estack
    msr     msp, r0

    /* Copy .data: LMA = _etext, VMA = [_sdata .. _edata) */
    ldr     r0, =_sdata
    ldr     r1, =_edata
    ldr     r2, =_etext
    b       2f
1:  ldr     r3, [r2], #4
    str     r3, [r0], #4
2:  cmp     r0, r1
    bcc     1b

    /* Zero .bss */
    ldr     r0, =_sbss
    ldr     r1, =_ebss
    movs    r2, #0
    b       4f
3:  str     r2, [r0], #4
4:  cmp     r0, r1
    bcc     3b

    bl      SystemInit
    bl      main
5:  b       5b
    .size   Reset_Handler, .-Reset_Handler

/* ============= Default handler (traps) ============= */
    .section .text.Default_Handler, "ax", %progbits
    .thumb_func
    .global  Default_Handler
    .type    Default_Handler, %function
Default_Handler:
    b       .
    .size   Default_Handler, .-Default_Handler

/* Make every named handler default to Default_Handler unless overridden. */
    .macro  weak_handler name
    .weak   \name
    .thumb_set \name, Default_Handler
    .endm

    weak_handler  NMI_Handler
    weak_handler  HardFault_Handler
    weak_handler  MemManage_Handler
    weak_handler  BusFault_Handler
    weak_handler  UsageFault_Handler
    weak_handler  SVC_Handler
    weak_handler  DebugMon_Handler
    weak_handler  PendSV_Handler
    weak_handler  SysTick_Handler

    weak_handler  WWDG_IRQHandler
    weak_handler  PVD_IRQHandler
    weak_handler  TAMPER_IRQHandler
    weak_handler  RTC_IRQHandler
    weak_handler  FLASH_IRQHandler
    weak_handler  RCC_IRQHandler
    weak_handler  EXTI0_IRQHandler
    weak_handler  EXTI1_IRQHandler
    weak_handler  EXTI2_IRQHandler
    weak_handler  EXTI3_IRQHandler
    weak_handler  EXTI4_IRQHandler
    weak_handler  DMA1_Channel1_IRQHandler
    weak_handler  DMA1_Channel2_IRQHandler
    weak_handler  DMA1_Channel3_IRQHandler
    weak_handler  DMA1_Channel4_IRQHandler
    weak_handler  DMA1_Channel5_IRQHandler
    weak_handler  DMA1_Channel6_IRQHandler
    weak_handler  DMA1_Channel7_IRQHandler
    weak_handler  ADC1_2_IRQHandler
    weak_handler  USB_HP_CAN1_TX_IRQHandler
    weak_handler  USB_LP_CAN1_RX0_IRQHandler
    weak_handler  CAN1_RX1_IRQHandler
    weak_handler  CAN1_SCE_IRQHandler
    weak_handler  EXTI9_5_IRQHandler
    weak_handler  TIM1_BRK_IRQHandler
    weak_handler  TIM1_UP_IRQHandler
    weak_handler  TIM1_TRG_COM_IRQHandler
    weak_handler  TIM1_CC_IRQHandler
    weak_handler  TIM2_IRQHandler
    weak_handler  TIM3_IRQHandler
    weak_handler  TIM4_IRQHandler
    weak_handler  I2C1_EV_IRQHandler
    weak_handler  I2C1_ER_IRQHandler
    weak_handler  I2C2_EV_IRQHandler
    weak_handler  I2C2_ER_IRQHandler
    weak_handler  SPI1_IRQHandler
    weak_handler  SPI2_IRQHandler
    weak_handler  USART1_IRQHandler
    weak_handler  USART2_IRQHandler
    weak_handler  USART3_IRQHandler
    weak_handler  EXTI15_10_IRQHandler
    weak_handler  RTC_Alarm_IRQHandler
    weak_handler  USBWakeUp_IRQHandler

    .end
