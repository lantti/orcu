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
  r4, gen_pwmlist
  r5, gen_pwmlist
  r6, gen_pwmlist
  r7, gen_pwmlist
  r8, gen_pwmlist
  r9, gen_pwmlist
  r10, gen_pwmlist, load_pwmlist
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
  r24, gen_pwmlist
  r25, gen_pwmlist
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

.equ PRESCALER_MODE = 4

.equ PING_RESPONSE = 0x326B726C       //"lrk2"

.equ BSWITCH = 0x10

.DSEG
.ORG 0xDE
msg_buffer:	.BYTE 33
.ORG 0x100
userspace:      .BYTE 1

.ORG 0x120
buffer0:        .BYTE 14
.ORG 0x130
buffer1:        .BYTE 14

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
PW_TABLE:               .DB 15,7,2,0,0

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
                        ldi r19,0x00
                        ldi r18,msg_buffer+1
                        rcall load_pwmlist
                        ldi r16,160               //(20<<3)|0
                        ldi r31,0x00
                        ldi r30,msg_buffer
                        st Z,r16
                        ret



PIPE2_CB:
                        ldi r17,0x00
                        ldi r16,msg_buffer+1
                        ldi r19,0x00
                        ldi r18,msg_buffer+8
                        rcall gen_pwmlist
                        ldi r19,0x00
                        ldi r18,msg_buffer+8
                        rcall load_pwmlist
                        ldi r16,160               //(20<<3)|0
                        ldi r31,0x00
                        ldi r30,msg_buffer
                        st Z,r16
                        ret


PIPE3_CB:               ldi r31,0x00
                        ldi r30,msg_buffer+1
                        ld  r16,Z+
                        mov r17,r16
                        swap r17
                        andi r16,0x0F
                        andi r17,0x0F
                        movw r19:r18,r17:r16
                        lsl  r16
                        lsl  r17
                        add  r16,r18
                        add  r17,r19                        
                        subi r16,-3
                        subi r17,-3
                        
			ldi r20,3
eereadloop:		out EEARL,r16
			sbi EECR,EERE
			in r18,EEDR
                        out EEARL,r17
			sbi EECR,EERE
			in r19,EEDR
			st Z+,r18
			st Z+,r19
                        inc r16
                        inc r17
			dec r20
			brne eereadloop

                        ldi r17,0x00
                        ldi r16,msg_buffer+2
                        ldi r19,0x00
                        ldi r18,msg_buffer+9
                        rcall gen_pwmlist
                        ldi r19,0x00
                        ldi r18,msg_buffer+9
                        rcall load_pwmlist
                        ldi r16,160               //(20<<3)|0
                        ldi r31,0x00
                        ldi r30,msg_buffer
                        st Z,r16
                        ret
PIPE4_CB:
PIPE5_CB:
                        ret

green:                  .DB 100,((1<<LEDG2)|(1<<LEDB1)),1,0,0,0,0,0,0,245,((1<<LEDR2)|(1<<LEDR1)),1,0,0

SETUP:                  ldi r23,LOW(buffer0)

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

                        ldi r31,HIGH(green<<1)
                        ldi r30,LOW(green<<1)
                        ldi r27,HIGH(buffer0)
                        ldi r26,LOW(buffer0)

colorloop:              lpm r16,Z+
                        st  X+,r16
                        cpi r26,LOW(buffer0+14)
                        brlo colorloop                 

                        ldi r26,LOW(buffer0+1)
                        ldi r27,HIGH(buffer0+1)
                        ldi r28,LOW(buffer0+10)
                        ldi r29,HIGH(buffer0+10)   

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


//expects r19:18 pointing to the new pwmlist
load_pwmlist:
                        ldi r21,0x01
                        mov r20,r23
                        ldi r16,BSWITCH
                        eor r20,r16
                        ldi r16,7
                        mov r10,r16
copyloop:               movw r31:r30,r19:r18
                        ld  r16,Z+
                        ld  r17,Z+
                        movw r19:r18,r31:r30
                        movw r31:r30,r21:r20
                        st  Z+,r16
                        st  Z+,r17
                        movw r21:r20,r31:r30
                        dec  r10
                        brne copyloop
                        ldi r16,BSWITCH
                        eor r23,r16
                        ret


//expects r17:r16 pointing to colour list, r19:r18 to target buffer. Mangels r10, r9, r8, r7, r6, r5, r16, r17, r18, r19, r20, r21
PIN_TOGGLE_MASKS:       .DB (1<<LEDG1),(1<<LEDG2),(1<<LEDB1),(1<<LEDB2),(1<<LEDR1),(1<<LEDR2)
gen_pwmlist:            ldi r20,4
                        mov r10,r20
                        clr r5
                        movw r25:r24,r19:r18
                        movw r31:r30,r19:r18
                        st  Z+,r5
                        st  Z+,r5
                        st  Z+,r5
                        st  Z+,r5
                        ldi r21,HIGH(PIN_TOGGLE_MASKS<<1)
                        ldi r20,LOW(PIN_TOGGLE_MASKS<<1)
gen_pl_next:            dec r10                 //load next values to insert into the pwmlist or
                        brmi gen_pl_done        //jump to end if none left
                        movw r31:r30,r17:r16
                        ld  r6,Z+
                        movw r17:r16,r31:r30
                        movw r31:r30,r21:r20
                        lpm  r7,Z+
                        movw r21:r20,r31:r30
                        tst  r6
                        breq gen_pl_next
                        neg  r6
                        movw r31:r30,r25:r24
gen_pl_sort:            cp  r30,r18              //insertion sort with merge for equal values
                        cpc r31,r19
                        breq gen_pl_place
                        sbiw r31:r30,2
                        ld r8,Z+
                        ld r9,Z+
                        cp r8,r6
                        breq gen_pl_merge
                        brlo gen_pl_place
                        st Z+,r8
                        st Z+,r9
                        sbiw r31:r30,4
                        rjmp gen_pl_sort
gen_pl_place:           st Z+,r6
                        st Z+,r7
                        adiw r25:r24,2
                        rjmp gen_pl_next
gen_pl_merge:           or  r9,r7
                        st -Z,r9
                        rjmp gen_pl_next
gen_pl_done:            movw r31:r30,r25:r24
                        clr r10
                        st Z+,r10
                        tst r5
                        brne gen_pl_ret
                        ldi r30,2
                        mov r10,r30
                        movw r25:r24,r19:r18
                        adiw r25:r24,9
                        movw r19:r18,r25:r24
                        movw r31:r30,r19:r18
                        st  Z+,r5
                        st  Z+,r5
                        st  Z+,r5
                        st  Z+,r5
                        inc r5
                        rjmp gen_pl_next
gen_pl_ret:             ret



TIM0_COMPA_H:
                        ld r22, X+
			out PINA, r22
			ld r22, X+
			out OCR0A, r22
			reti

TIM0_COMPB_H:
			ld r22, Y+
			out PINB, r22
			ld r22, Y+
			out OCR0B, r22
			reti

TIM0_OVF_H:
			in r2, SREG
			cbi PORTA, LEDB1
			cbi PORTA, LEDB2
			cbi PORTA, LEDG1
			cbi PORTA, LEDG2
			cbi PORTB, LEDR1
			cbi PORTB, LEDR2

			mov r26, r23
			mov r28, r23
			subi r28,-9

			ld r22,X+
			out OCR0A,r22
			ld r22,Y+
			out OCR0B,r22
			out SREG,r2
			reti


