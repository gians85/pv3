;*******************************************************************************
;*
;* File Name          : startup_stm32f070xb.s
;*
;*******************************************************************************
;
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table

        ;EXTERN  OTA_VALID_APP_TAG
        ;EXTERN  BLUE_FLAG_TAG

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler                  ; Reset Handler

        DCD     NMI_Handler                    ; NMI Handler
        DCD     HardFault_Handler              ; Hard Fault Handler
        DCD     0                              ; OTA Application (TBC)
        DCD     0x424C5545                     ; Reserved for blue flag DTM updater (TBC)
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     SVC_Handler                    ; SVCall Handler
        DCD     0                              ; Reserved
        DCD     0                              ; Reserved
        DCD     PendSV_Handler                 ; PendSV Handler
        DCD     SysTick_Handler                ; SysTick Handler

        ; External Interrupts
        DCD     GPIO_Handler                   ; IRQ 0: GPIO
        DCD     NVM_Handler                    ; IRQ 1: NVM
        DCD     0                              ; IRQ 2:
        DCD     0                              ; IRQ 3:
        DCD     UART_Handler                   ; IRQ 4: UART
        DCD     SPI_Handler                    ; IRQ 5: SPI
        DCD     Blue_Handler                   ; IRQ 6: Blue
        DCD     WDG_Handler                    ; IRQ 7: Watchdog
        DCD     0                              ; IRQ 8:
        DCD     0                              ; IRQ 9:
        DCD     0                              ; IRQ10:
        DCD     0                              ; IRQ11:
        DCD     0                              ; IRQ12:
        DCD     ADC_Handler                    ; IRQ13: ADC
        DCD     I2C1_Handler                   ; IRQ14: I2C2
        DCD     I2C2_Handler                   ; IRQ15: I2C1
        DCD     0                              ; IRQ16:
        DCD     MFT1A_Handler                  ; IRQ17: MFT1 irq1
        DCD     MFT1B_Handler                  ; IRQ18: MFT1 irq2
        DCD     MFT2A_Handler                  ; IRQ19: MFT2 irq1
        DCD     MFT2B_Handler                  ; IRQ20: MFT2 irq2
        DCD     RTC_Handler                    ; IRQ21: RTC
        DCD     PKA_Handler                    ; IRQ22: PKA
        DCD     DMA_Handler                    ; IRQ23: DMA
        DCD     0                              ; IRQ24:
        DCD     0                              ; IRQ25:
        DCD     0                              ; IRQ26:
        DCD     0                              ; IRQ27:
        DCD     0                              ; IRQ28:
        DCD     0                              ; IRQ29:
        DCD     0                              ; IRQ30:
        DCD     0                              ; IRQ31:


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:NOROOT:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK GPIO_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
GPIO_Handler
        B GPIO_Handler

        PUBWEAK NVM_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
NVM_Handler
        B NVM_Handler

        PUBWEAK UART_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART_Handler
        B UART_Handler

        PUBWEAK SPI_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI_Handler
        B SPI_Handler

        PUBWEAK Blue_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
Blue_Handler
        B Blue_Handler

        PUBWEAK WDG_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
WDG_Handler
        B WDG_Handler

        PUBWEAK ADC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
ADC_Handler
        B ADC_Handler

        PUBWEAK I2C1_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C1_Handler
        B I2C1_Handler

        PUBWEAK I2C2_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C2_Handler
        B I2C2_Handler

        PUBWEAK MFT1A_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT1A_Handler
        B MFT1A_Handler

        PUBWEAK MFT1B_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT1B_Handler
        B MFT1B_Handler

        PUBWEAK MFT2A_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT2A_Handler
        B MFT2A_Handler

        PUBWEAK MFT2B_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
MFT2B_Handler
        B MFT2B_Handler

        PUBWEAK RTC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
RTC_Handler
        B RTC_Handler

        PUBWEAK PKA_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
PKA_Handler
        B PKA_Handler

        PUBWEAK DMA_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Handler
        B DMA_Handler

        END
;*****************************END OF FILE*****
