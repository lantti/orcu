/*
 * mod_test.asm
 * Author: Lantti
 */ 


/*
  Reg summary

  r0, mainloop
  r1, mainloop 
  r2, 
  r3, 
  r4, 
  r5, 
  r6, 
  r7, 
  r8, 
  r9, 
  r10, 
  r11,
  r12, 
  r13,
  r14,
  r15, spi_op
  r16, mainloop, spi_op, init
  r17, mainloop, init
  r18, mainloop, init
  r19, mainloop, init
  r20, mainloop, spi_op, init
  r21, mainloop, spi_op, init
  r22,
  r23,
  r24, 
  r25, 
  r26 (Xl),
  r27 (Xh),
  r28 (Yl),
  r29 (Yh),
  r30 (Zl), init, mainloop, spi_op
  r31 (Zh), init, mainloop, spi_op
*/


.INCLUDE "tn44Adef.inc"

.equ LEDB1 = PORTA0
.equ LEDB2 = PORTA1
.equ LEDG1 = PORTA2
.equ LEDG2 = PORTA3
.equ LEDR1 = PORTB1
.equ LEDR2 = PORTB2


.equ PING_RESPONSE = 0x31747374       //"tst1"

.DSEG
.ORG 0xDE
msg_buffer:	.BYTE 33
.ORG 0x100
userspace:      .BYTE 1

.CSEG
                        ;Interrupt vectors
			rjmp   0x600            ; Reset Handler
			rjmp   INT0_H           ; IRQ0 Handler
			rjmp   PCINT0_H         ; PCINT0 Handler
			rjmp   PCINT1_H         ; PCINT1 Handler
			rjmp   WDT_H            ; Watchdog Interrupt Handler
			rjmp   TIM1_CAPT_H      ; Timer1 Capture Handler
			rjmp   TIM1_COMPA_H     ; Timer1 Compare A Handler
			rjmp   TIM1_COMPB_H     ; Timer1 Compare B Handler
			rjmp   TIM1_OVF_H       ; Timer1 Overflow Handler
			rjmp   TIM0_COMPA_H     ; Timer0 Compare A Handler
			rjmp   TIM0_COMPB_H     ; Timer0 Compare B Handler
			rjmp   TIM0_OVF_H       ; Timer0 Overflow Handler
			rjmp   ANA_COMP_H       ; Analog Comparator Handler
			rjmp   ADC_CONV_H       ; ADC Conversion Handler
			rjmp   EE_RDY_H         ; EEPROM Ready Handler
			rjmp   USI_STR_H        ; USI STart Handler
			rjmp   USI_OVF_H        ; USI Overflow Handler

                        ;RF recieve pipe widths
PW_TABLE:               .DB 255,255,255,255,255

                        ;standard function vectors
SETUP_VECT:             rjmp   SETUP
PING_VECT:              rjmp   PING
ERROR_VECT:             rjmp   ERROR

                        ;RF receive pipe callback vectors
PIPE_CB_VECT:           rjmp   PIPE1_CB
                        rjmp   PIPE2_CB
                        rjmp   PIPE3_CB
                        rjmp   PIPE4_CB
                        rjmp   PIPE5_CB

INT0_H:           ; IRQ0 Handler
PCINT0_H:         ; PCINT0 Handler
PCINT1_H:         ; PCINT1 Handler
WDT_H:            ; Watchdog Interrupt Handler
TIM1_CAPT_H:      ; Timer1 Capture Handler
TIM1_COMPA_H:     ; Timer1 Compare A Handler
TIM1_COMPB_H:     ; Timer1 Compare B Handler
TIM1_OVF_H:       ; Timer1 Overflow Handler
TIM0_COMPA_H:
TIM0_COMPB_H:
TIM0_OVF_H:
ANA_COMP_H:       ; Analog Comparator Handler
ADC_CONV_H:       ; ADC Conversion Handler
EE_RDY_H:         ; EEPROM Ready Handler
USI_STR_H:        ; USI STart Handler
USI_OVF_H:        ; USI Overflow Handler
			reti

PIPE1_CB:               sbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        cbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ret

PIPE2_CB:               cbi PORTA,LEDB1
                        sbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        cbi PORTB,LEDR2
                        ret

PIPE3_CB:               sbi PORTA,LEDB1
                        sbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        cbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ret

PIPE4_CB:               sbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        cbi PORTA,LEDG1
                        cbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ret

PIPE5_CB:               sbi PORTA,LEDB1
                        sbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ret

SETUP:                  sbi DDRA,LEDB1
                        sbi DDRA,LEDB2
                        sbi DDRA,LEDG1
                        sbi DDRA,LEDG2
                        sbi DDRB,LEDR1
                        sbi DDRB,LEDR2
                        cbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        cbi PORTB,LEDR2
                        ret

PING:                   ldi r31,0x00
                        ldi r30,msg_buffer
                        ldi r16,0x18
                        st  Z+,r16
                        ldi r16,LOW(PING_RESPONSE)
                        st  Z+,r16
                        ldi r16,HIGH(PING_RESPONSE)
                        st  Z+,r16
                        ldi r16,BYTE3(PING_RESPONSE)
                        st  Z+,r16
                        ldi r16,BYTE4(PING_RESPONSE)
                        st  Z+,r16
                        ret

ERROR:                  cbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        cbi PORTA,LEDG1
                        cbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        sbi DDRA,LEDB1
                        sbi DDRA,LEDB2
                        sbi DDRA,LEDG1
                        sbi DDRA,LEDG2
                        sbi DDRB,LEDR1
                        sbi DDRB,LEDR2

                        cli
                        sleep
                        rjmp ERROR

