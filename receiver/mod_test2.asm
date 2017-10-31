/*
 * mod_lrk.asm
 * Author: Lantti
*/ 

/*
  Reg summary

  r0, mainloop, init
  r1, mainloop, init 
  r2, PWM
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
  r22, PWM
  r23, PWM
  r24, PWM
  r25, PWM
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

.equ PRESCALER_MODE = 3

.equ PING_RESPONSE = 0x326B726C       //"lrk1"

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
PW_TABLE:               .DB 0,0,0,0,0

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
ANA_COMP_H:       ; Analog Comparator Handler
ADC_CONV_H:       ; ADC Conversion Handler
EE_RDY_H:         ; EEPROM Ready Handler
USI_STR_H:        ; USI STart Handler
USI_OVF_H:        ; USI Overflow Handler
			reti


PIPE1_CB:               
PIPE2_CB:
PIPE3_CB:
PIPE4_CB:
PIPE5_CB:
                        ret

SETUP:                  ldi r23,(1<<LEDB1)|(1<<LEDB2)
                        ldi r24,0x00
                        ldi r25,0x00

                        ldi r16, (1<<PRUSI)+(1<<PRADC)
			out PRR, r16

                        cbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        cbi PORTA,LEDG1
                        cbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        cbi PORTB,LEDR2
                        sbi DDRA,LEDB1
                        sbi DDRA,LEDB2
                        sbi DDRA,LEDG1
                        sbi DDRA,LEDG2
                        sbi DDRB,LEDR1
                        sbi DDRB,LEDR2

			ldi r16, (1<<TOIE0)+(1<<OCIE0A)+(1<<OCIE0B)
			out TIMSK0, r16		;enable the PWM interrupts
			ldi r16, PRESCALER_MODE
			out TCCR0B, r16		;start the timer and PWM routine
                        sei
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

ERROR:                  cli
                        cbi PORTA,LEDB1
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
                        sleep
                        rjmp ERROR


TIM0_COMPA_H:
			out PINA, r23
			reti

TIM0_COMPB_H:
			ldi r22,(1<<LEDR1)|(1<<LEDR2)
			out PINB, r22
			reti

TIM0_OVF_H:
			in r2, SREG
			cbi PORTA, LEDB1
			cbi PORTA, LEDB2
			cbi PORTA, LEDG1
			cbi PORTA, LEDG2
			cbi PORTB, LEDR1
			cbi PORTB, LEDR2
                        mov r22,r25
			out OCR0A, r22
                        com r22
			out OCR0B, r22
                        adiw r25:r24,63
                        brcc noswap
                        ldi r22,(1<<LEDB1)|(1<<LEDG1)|(1<<LEDB2)|(1<<LEDG2)
                        eor r23,r22                        
noswap:			out SREG,r2
			reti

