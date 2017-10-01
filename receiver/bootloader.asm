/*
 * LRK.asm
 *
 *  Created: 22.12.2015 00:22:14
 *   Author: Lantti
 *
 *	Lichtröhrlikontroller
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
  r13, init 
  r14, init
  r15, init
  r16, mainloop, spi_op, init
  r17, mainloop
  r18, mainloop
  r19, mainloop, spi_op
  r20, mainloop, spi_op
  r21, mainloop, spi_op
  r22,
  r23,
  r24, 
  r25, 
  r26 (Xl), init
  r27 (Xh), init
  r28 (Yl), init
  r29 (Yh), init
  r30 (Zl), init, mainloop, spi_op
  r31 (Zh), init, mainloop, spi_op
*/


.INCLUDE "tn44Adef.inc"

.equ LEDB1 = PORTA0
.equ LEDB2 = PORTA1
.equ LEDG1 = PORTA2
.equ LEDG2 = PORTA3
.equ SCK = PORTA4
.equ MISO = PORTA5
.equ MOSI = PORTA6
.equ CSN = PORTA7

.equ IRQ = PORTB0
.equ LEDR1 = PORTB1
.equ LEDR2 = PORTB2
.equ CE = PORTB3

.equ IRQ_PCMSKREG = PCMSK1
.equ IRQ_PCIE = PCIE1
.equ IRQ_PCINT = PCINT8

.equ PORTA_DDR = (1<<LEDB1)+(1<<LEDB2)+(1<<LEDG1)+(1<<LEDG2)+(1<<SCK)+(0<<MISO)+(1<<MOSI)+(1<<CSN)
.equ PORTA_UP =  (0<<LEDB1)+(0<<LEDB2)+(0<<LEDG1)+(0<<LEDG2)+(0<<SCK)+(1<<MISO)+(0<<MOSI)+(1<<CSN)

.equ PORTB_DDR = (0<<IRQ)+(1<<LEDR1)+(1<<LEDR2)+(1<<CE)
.equ PORTB_UP =  (1<<IRQ)+(0<<LEDR1)+(0<<LEDR2)+(0<<CE)

.equ WD_TIMEOUT = 7
.equ WDP_BITS = (((WD_TIMEOUT & 8) << 2) | (WD_TIMEOUT & 7)) 

.equ R_REGISTER =            0b00000000
.equ W_REGISTER =            0b00100000
.equ R_RX_PAYLOAD =          0b01100001
.equ W_TX_PAYLOAD =          0b10100000
.equ FLUSH_TX =              0b11100001
.equ FLUSH_RX =              0b11100010
.equ REUSE_TX_PL =           0b11100011
.equ R_RX_PL_WID =           0b01100000
.equ W_ACK_PAYLOAD =         0b10101000
.equ W_TX_PAYLOAD_NO_ACK =   0b10110000
.equ RF_NOP =                0b11111111

.equ CONFIG =      0x00
.equ EN_AA =       0x01
.equ EN_RXADDR =   0x02
.equ SETUP_AW =    0x03
.equ SETUP_RETR =  0x04
.equ RF_CH =       0x05
.equ RF_SETUP =    0x06
.equ STATUS =      0x07
.equ OBSERVE_TX =  0x08
.equ RPD =         0x09
.equ RX_ADDR_P0 =  0x0A
.equ RX_ADDR_P1 =  0x0B
.equ RX_ADDR_P2 =  0x0C
.equ RX_ADDR_P3 =  0x0D
.equ RX_ADDR_P4 =  0x0E
.equ RX_ADDR_P5 =  0x0F
.equ TX_ADDR =     0x10
.equ RX_PW_P0 =    0x11
.equ RX_PW_P1 =    0x12
.equ RX_PW_P2 =    0x13
.equ RX_PW_P3 =    0x14
.equ RX_PW_P4 =    0x15
.equ RX_PW_P5 =    0x16
.equ FIFO_STATUS = 0x17
.equ DYNPD =       0x1C
.equ FEATURE =     0x1D

.equ CONFIG_INIT_POFF = 0b00001001
.equ CONFIG_INIT_PON =  0b00001011
.equ EN_AA_INIT =       0b00111111
.equ SETUP_AW_INIT =    0b00000001
.equ SETUP_RETR_INIT =  0b00010011
.equ RF_SETUP_INIT =    0b00001111
.equ STATUS_INIT =      0b01110000
.equ RX_ADDR_P0_INIT =  0xE7           //Additional 2 bytes from eeprom
.equ RX_ADDR_P1_INIT =  0xC2           //Additional 2 bytes from eeprom
.equ RX_ADDR_P2_INIT =  0xC3
.equ RX_ADDR_P3_INIT =  0xC4
.equ RX_ADDR_P4_INIT =  0xC5
.equ RX_ADDR_P5_INIT =  0xC6
.equ FEATURE_INIT =     0b00000111



.DSEG
.ORG 0x60
                .BYTE 47
stack:		.BYTE 1
page_buffer:    .BYTE 64
.ORG 0xDE
                .BYTE 1
bufferc:	.BYTE 32
.ORG 0x100
userspace:      .BYTE 1

.CSEG
                        ;Interrupt vectors
			rjmp   RESET_H          ; Reset Handler
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
PW_TABLE:               .DB 16,8,32,2,1

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
                        cbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ret
PIPE2_CB:               sbi PORTA,LEDB1
                        sbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        cbi PORTB,LEDR2
                        ret
PIPE3_CB:               cbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ret
PIPE4_CB:               sbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        cbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        cbi PORTB,LEDR2
                        ret
PIPE5_CB:               cbi PORTA,LEDB1
                        sbi PORTA,LEDB2
                        cbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ret
SETUP:                  sbi PORTA,LEDB1
                        sbi PORTA,LEDB2
                        cbi PORTA,LEDG1
                        cbi PORTA,LEDG2
                        cbi PORTB,LEDR1
                        cbi PORTB,LEDR2
                        ret
PING:                   sbi PORTA,LEDB1
                        sbi PORTA,LEDB2
                        sbi PORTA,LEDG1
                        sbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        ldi r31,0x00
                        ldi r30,bufferc-1
                        ldi r16,0x78
                        st  Z+,r16
                        st  Z+,r16
                        st  Z+,r16
                        ldi r16,0xAA
                        st  Z+,r16
                        st  Z+,r16
                        st  Z+,r16
                        ldi r16,0xBB
                        st  Z+,r16
                        st  Z+,r16
                        st  Z+,r16
                        ldi r16,0xCC
                        st  Z+,r16
                        st  Z+,r16
                        st  Z+,r16
                        ldi r16,0xDD
                        st  Z+,r16
                        st  Z+,r16
                        st  Z+,r16
                        st  Z+,r16
                        ret
ERROR:                  cbi PORTA,LEDB1
                        cbi PORTA,LEDB2
                        cbi PORTA,LEDG1
                        cbi PORTA,LEDG2
                        sbi PORTB,LEDR1
                        sbi PORTB,LEDR2
                        cli
                        sleep
                        rjmp ERROR


.ORG 0x600
/* After reset initialze stack, pointers (XYZ), IO, interrupts and rf-module
*/


//Setup MCU
RESET_H:                
			ldi r16, HIGH(stack)
			out SPH, r16
			ldi r16, LOW(stack)
			out SPL, r16

                        ldi r16, 0
                        out MCUSR, r16

                        ldi r16, (0<<WDIF|0<<WDIE|0<<WDP3|1<<WDCE|1<<WDE|0<<WDP2|0<<WDP1|0<<WDP0)
                        out WDTCSR, r16

                        ldi r16, (0<<WDIF|0<<WDIE|0<<WDCE|0<<WDE|WDP_BITS)
                        out WDTCSR, r16

initeeprom:             sbic EECR, EEPE
                        rjmp initeeprom
			ldi r16, (0<<EEPM1)|(0<<EEPM0)
                        out EECR, r16

                        clr r31
			clr r29
			clr r27

			ldi r16, PORTA_UP
			out PORTA, r16
			ldi r16, PORTA_DDR
			out DDRA, r16

			ldi r16, PORTB_UP
			out PORTB, r16
			ldi r16, PORTB_DDR
			out DDRB, r16

			ldi r16, (1<<SE)
			out MCUCR, r16
			ldi r16, (1<<PRUSI)+(1<<PRADC)
			out PRR, r16

			ldi r16, (1<<IRQ_PCIE)
			out GIMSK, r16
			ldi r16, (1<<IRQ_PCINT)
			out IRQ_PCMSKREG, r16

//read rf-parameters stored on the eeprom
			ldi r16,2
eereadloop:		out EEARL,r16
			sbi EECR,EERE
			in r23,EEDR
			push r23
			dec r16
			brpl eereadloop
                        pop r13
                        pop r14
                        pop r15
//Setup RF-module
//Try to detect nrf24L01+ by reading TX_ADDR and checking for the default
			ldi r30, bufferc
			ldi r16, R_REGISTER + TX_ADDR
			st -Z,r16
			ldi r16,5
			rcall spi_op

			ldi r28, bufferc
			ldi r23,5
funkcheckloop:		ld r16,Y+
			cpi r16,0xE7
			brne funkfail
			dec r23
			brne funkcheckloop
                        rjmp funkok
funkfail:               rjmp ERROR
funkok:

			ldi r30, bufferc
			ldi r16, CONFIG_INIT_POFF	;set rf-module to power-off and RX mode
			st Z,r16
			ldi r16, W_REGISTER + CONFIG
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30, bufferc
                        ldi r16, FLUSH_TX
                        st  -Z,r16
                        ldi r16,0
                        rcall spi_op

                        ldi r30, bufferc
                        ldi r16, FLUSH_RX
                        st  -Z,r16
                        ldi r16,0
                        rcall spi_op

                        ldi r30, bufferc
			ldi r16, STATUS_INIT            //clear IRQs
			st Z,r16
			ldi r16, W_REGISTER + STATUS
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,EN_AA_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + EN_AA
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,SETUP_AW_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + SETUP_AW
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,SETUP_RETR_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + SETUP_RETR
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,RF_SETUP_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + RF_SETUP
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,FEATURE_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + FEATURE
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30,bufferc
                        st Z,r13
			ldi r16,W_REGISTER + RF_CH
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r16,RX_ADDR_P0_INIT
                        ldi r30,bufferc+2
                        st  Z,r14
                        st  -Z,r15
                        st  -Z,r16
			ldi r16,W_REGISTER + RX_ADDR_P0	;write control pipe address to rf
			st -Z,r16
			ldi r16,3
			rcall spi_op

                        ldi r16,TX_ADDR
                        ldi r30,bufferc
                        st -Z,r16
                        ldi r16,3
                        rcall spi_op

                        ldi r16,RX_ADDR_P1_INIT
                        ldi r30,bufferc+2
                        st  Z,r14
                        st  -Z,r15
                        st  -Z,r16
			ldi r16,W_REGISTER + RX_ADDR_P1	;write pipe1 address to rf
			st -Z,r16
			ldi r16,3
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,RX_ADDR_P2_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + RX_ADDR_P2
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,RX_ADDR_P3_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + RX_ADDR_P3
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,RX_ADDR_P4_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + RX_ADDR_P4
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,RX_ADDR_P5_INIT
                        st  Z,r16
			ldi r16,W_REGISTER + RX_ADDR_P5
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30, bufferc
			ldi r16,32		        ;set control pipe length
			st Z,r16
			ldi r16, W_REGISTER + RX_PW_P0
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30,PW_TABLE<<1
                        lpm r16,Z
                        ldi r30,bufferc
                        st Z,r16
			ldi r16,W_REGISTER + RX_PW_P1	;set pipe1 length
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30,(PW_TABLE<<1)+1
                        lpm r16,Z
                        ldi r30,bufferc
                        st Z,r16
			ldi r16,W_REGISTER + RX_PW_P2	;set pipe2 length
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30,(PW_TABLE<<1)+2
                        lpm r16,Z
                        ldi r30,bufferc
                        st Z,r16
			ldi r16,W_REGISTER + RX_PW_P3	;set pipe3 length
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30,(PW_TABLE<<1)+3
                        lpm r16,Z
                        ldi r30,bufferc
                        st Z,r16
			ldi r16,W_REGISTER + RX_PW_P4	;set pipe4 length
			st -Z,r16
			ldi r16,1
			rcall spi_op

                        ldi r30,(PW_TABLE<<1)+4
                        lpm r16,Z
                        ldi r30,bufferc
                        st Z,r16
			ldi r16,W_REGISTER + RX_PW_P5	;set pipe5 length
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,0b00111111
                        st  Z,r16
			ldi r16,W_REGISTER + EN_RXADDR   ;enable all rf pipes
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30,bufferc
                        ldi r16,0b00000001
                        st  Z,r16
			ldi r16,W_REGISTER + DYNPD
			st -Z,r16
			ldi r16,1
			rcall spi_op

			ldi r30, bufferc
			ldi r16, CONFIG_INIT_PON	;set rf-module to power-on and RX mode
			st Z,r16
			ldi r16, W_REGISTER + CONFIG
			st -Z,r16
			ldi r16,1
			rcall spi_op


                        rcall SETUP_VECT

			sei                   ; Enable interrupts

/*Main loop
  Waits for interrupt, checks if there is a new packet to read,
  if yes, reads the packet, calls an appropriate handler and
  returns to wait
  Mangels: Z,r16,r18,r19,r20,r21
*/
FOREVER:	        sleep
			sbic PINB,IRQ
			rjmp FOREVER

rf_op:                  ldi r31,0x00
                        ldi r30, bufferc
                        ldi r16, R_RX_PL_WID  ;read PL width and status to get the current pipe number
                        st  -Z,r16
                        ldi r16,1
                        rcall spi_op
                        
                        ldi r30,bufferc-1
                        ld r17,Z+
                        ld r18,Z

                        mov r16,r17
                        andi r17,0b00001110
                        breq dynamic_pl       ;control pipe has always dynamic width
                        cpi r17,0x0C
                        brlo get_pipe_width
                        andi r16,0b01110000   ;nothing to read so cleanup or go back to wait
                        breq FOREVER
                        rjmp clear_irq
                        
get_pipe_width:         lsr r17               ;pipe number of the current pipe
                        ldi r30,(PW_TABLE<<1)-1
                        add r30,r17
                        lpm r16,Z             ;get the width spec of the current pipe


                        cpi r16,0xFF
                        brne read_packet
dynamic_pl:             cpi r18,33
                        brsh rx_flush
                        mov r16,r18           ;dynamic payload width

read_packet:            ldi r30, bufferc
			ldi r19,R_RX_PAYLOAD
			st  -Z,r19
			rcall spi_op

                        ldi r31,0x00
                        ldi r30,bufferc-1
                        ldi r16,0xFF
                        st Z,r16

                        cpi r17,0x00
                        breq control_handler

                        ldi r30,PIPE_CB_VECT-1
                        add r30,r17
                        icall

end_handler:            ldi r31,0x00
                        ldi r30,bufferc-1
                        ld  r16,Z
                        mov r17,r16
                        lsr r16
                        lsr r16
                        lsr r16
                        inc r16
                        andi r17,0x07

                        cpi r17,0x06
                        brsh clear_irq
                        ldi r18,W_ACK_PAYLOAD
                        add r18,r17
                        st Z,r18
                        rcall spi_op

clear_irq:              ldi r30, bufferc
			ldi r16, 0b01110000	;IRQs to clear
			st Z,r16
			ldi r16, W_REGISTER + STATUS
			st -Z,r16
			ldi r16,1
			rcall spi_op
                        rjmp rf_op

tx_rx_flush:            ldi r30, bufferc
                        ldi r16, FLUSH_TX
                        st  -Z,r16
                        ldi r16,0
                        rcall spi_op
rx_flush:               ldi r30, bufferc
                        ldi r16, FLUSH_RX
                        st  -Z,r16
                        ldi r16,0
                        rcall spi_op
                        rjmp clear_irq

//To work around a bug in counterfeit nrf24L01+ chips
//a running number is included to the end of each payload
//thus reducing the effective payload size by one.
control_handler:           
                        ldi r31,0x00
                        ldi r30,bufferc
                        ld  r16,Z+

                        cpi r16, 0x00
                        breq reply_ping
                        cpi r16, 0x01
                        breq load_page
                        cpi r16, 0x02
                        breq write_buffer
                        cpi r16, 0x03
                        breq program_page
                        cpi r16, 0x04
                        breq program_eeprom_lj
                        cpi r16, 0x05
                        breq force_reset
                        cpi r16, 0x06
                        breq tx_rx_flush
                        cpi r16, 0xFF
                        breq clear_irq
                        rjmp clear_irq


program_eeprom_lj:      rjmp program_eeprom

reply_ping:             rcall PING
                        rjmp end_handler


force_reset:
                        cli
                        ldi r16, (0<<WDIF|0<<WDIE|0<<WDP3|1<<WDCE|1<<WDE|0<<WDP2|0<<WDP1|0<<WDP0)
                        out WDTCSR, r16
                        ldi r16, (0<<WDIF|0<<WDIE|0<<WDP3|0<<WDCE|1<<WDE|1<<WDP2|1<<WDP1|0<<WDP0)
                        out WDTCSR, r16
block:                  nop
                        rjmp block


load_page:
                        ld  r16,Z
                        andi r16,0x3F
                        ldi r31,0x00
                        mov r30,r16
                        ldi r16,6
l_page_mul:             lsl r30
                        rol r31
                        dec r16
                        brne l_page_mul
                        ldi r16,64
page_load_loop:         lpm r18, Z
                        push r31
                        push r30
                        ldi r31,0x00
                        andi r30,0x3F
                        subi r30,-page_buffer
                        st  Z,r18
                        pop r30
                        pop r31
                        inc r30
                        dec r16
                        brne page_load_loop
                        ldi r31,0x00
                        rjmp end_handler


write_buffer:
                        ld  r18,Z+
                        ld  r19,Z+
                        andi r18,0x3F
                        andi r19,0x0F
                        subi r18,-page_buffer
                        add r19,r18
                        cpi r19,page_buffer+0x40
                        brcs buffer_write_loop
                        ldi r19,page_buffer+0x3F
buffer_write_loop:      ld  r16,Z+
                        mov r20,r30
                        mov r30,r18
                        st  Z+,r16
                        mov r18,r30
                        mov r30,r20
                        cp  r19,r18
                        brcc buffer_write_loop
                        rjmp end_handler


program_page:
                        cli
                        push r28
                        push r29
                        ldi r29,0x00
                        ldi r28,page_buffer
                        ld  r16,Z
                        andi r16,0x3F
                        ldi r31,0x00
                        mov r30,r16
                        ldi r16,6
p_page_mul:             lsl r30
                        rol r31
                        dec r16
                        brne p_page_mul
                        ldi r16,32
                        ldi r18,0x01
page_program_loop:      ld  r0,Y+
                        ld  r1,Y+
                        out SPMCSR,r18
                        spm
                        subi r30,-2
                        dec r16
                        brne page_program_loop
f_erase_wait:           in  r18,SPMCSR
                        sbrc r18,SPMEN
                        rjmp f_erase_wait
                        andi r30,0xC0
                        ldi r18,0x03
                        out SPMCSR,r18
                        spm                        
f_prog_wait:            in  r18,SPMCSR
                        sbrc r18,SPMEN
                        rjmp f_prog_wait
                        andi r30,0xC0
                        ldi r18,0x05
                        out SPMCSR,r18
                        spm                        
f_ready_wait:           in  r18,SPMCSR
                        sbrc r18,SPMEN
                        rjmp f_ready_wait
                        ldi r31,0x00
                        pop  r29
                        pop  r28
                        sei
                        rjmp end_handler

;UNTESTESTED!!!
program_eeprom:
                        ld  r18,Z+
                        ld  r19,Z+
                        andi r19,0x0F
eewrite_loop:           sbic EECR, EEPE
                        rjmp eewrite_loop
                        ld  r16,Z+
                        out EEARL, r18
                        out EEDR, r16
                        sbi EECR, EEMPE
                        sbi EECR, EEPE
                        inc r18
                        dec r19
                        brpl eewrite_loop
ee_wait:                sbic EECR, EEPE
                        rjmp ee_wait
                        rjmp end_handler

/*SPI operations subroutine
  Requires a pointer to the SPI transfer buffer in pointer Z and
  transfer length - 1 in r16
  Returns: SPI operation response in the transfer buffer
  Mangles: Z, r16, r19, r20, r21*/
spi_op:
			cbi PORTA,CSN
spi_byte:		ldi r20,7
			ld r19,Z
spi_bit:		in r21,PINA
			andi r21,(1<<MISO)
			neg r21
			rol r19
			brcs spi_set
			cbi PORTA,MOSI
			rjmp spi_clear
spi_set:		sbi PORTA,MOSI
			nop
			nop
spi_clear:		sbi PORTA,SCK
			nop
			nop
			cbi PORTA,SCK
			dec r20
			brpl spi_bit
			st Z+,r19
			dec r16
			brpl spi_byte
spi_done:		sbi PORTA,CSN
			ret
