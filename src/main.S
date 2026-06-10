#define __SFR_OFFSET 0
#include <avr/io.h>

/* ======= КОНСТАНТИ ТА ВИЗНАЧЕННЯ ======= */
.equ fq, 4000000
.equ DELTIME, (500*fq)/40000  ; величина задания задержки времени 

TEMP = 22                     ; Аналог .def TEMP=r22 у GAS

/* ======= МАКРОСИ ======= */
.macro outi port, val
    ldi r16, \val
    out \port, r16
    nop
.endm

/* ======= СЕКЦІЯ EEPROM ======= */
.section .eeprom
.org 0x0100
data1: 
    .byte 0x00, 0x00, 0x30, 0x60, 0x90

/* ======= СЕКЦІЯ КОДУ (FLASH) ======= */
.section .text
.global main                  ; Робимо мітку main видимою для лінкера PlatformIO

.org 0x0000
main:                         ; Точка входу, яку шукає лінкер GCC
    rjmp reset          ; 0x0000 - Reset Handler
    reti                ; 0x0002 - INT0
    reti                ; 0x0004 - INT1
    reti                ; 0x0006 - OC2
    reti                ; 0x0008 - OVF2
    reti                ; 0x000A - ICP1
    reti                ; 0x000C - OC1A
    reti                ; 0x000E - OC1B
    reti                ; 0x0010 - OVF1
    reti                ; 0x0012 - OVF0
    reti                ; 0x0014 - SPI
    rjmp USART_rx       ; 0x0016 - USART RX Complete (MIDI)
    reti                ; 0x0018 - UDRE
    reti                ; 0x001A - UTX
    reti                ; 0x001C - ADCC
    reti                ; 0x001E - ERDY
    reti                ; 0x0020 - ACI
    reti                ; 0x0022 - TWI
    reti                ; 0x0024 - SPMR

.org 0x0030
; Инициализация девайса
reset:  
    cli
    ldi r16, 0xC0
    mov r6, r16
    ldi r16, 0x0
    ldi r17, 0x0
    ldi r18, 0x0
    ldi r19, 0x0
    ldi r20, 0x0
    ldi r21, 0x0
    ldi r22, 0x0
    ldi r23, 0x0
    ldi r24, 0x0
    ldi r25, 0x0
    ldi r26, 0x0
    ldi r27, 0x0
    ldi r28, 0x0
    ldi r29, 0x0
    ldi r30, 0x0
    ldi r31, 0x0
    outi SPH, hi8(RAMEND)
    outi SPL, lo8(RAMEND)  
    outi UCSRB, (1<<RXCIE)|(0<<TXEN)|(1<<RXEN)
    outi UCSRC, (1<<URSEL)|(0<<USBS)|(1<<UCSZ0)|(1<<UCSZ1) ; Конфігурація UCSRC з бітом URSEL для Atmega8
    outi UBRRH, 0
    outi UBRRL, 7
    ldi r17, 0x00
    ldi r18, 0b11111000
    out DDRD, r18             ; ВИПРАВЛЕНО НА ВЕЛИКІ ЛІТЕРИ
    nop
    out PORTD, r17            ; ВИПРАВЛЕНО НА ВЕЛИКІ ЛІТЕРИ
    nop
    ldi r18, 0x0
    ldi r17, 0xFF
    out DDRB, r18             ; ВИПРАВЛЕНО НА ВЕЛИКІ ЛІТЕРИ
    nop
    out PORTB, r17            ; ВИПРАВЛЕНО НА ВЕЛИКІ ЛІТЕРИ
    nop
    out DDRC, r18             ; ВИПРАВЛЕНО НА ВЕЛИКІ ЛІТЕРИ
    nop
    out PORTC, r17            ; ВИПРАВЛЕНО НА ВЕЛИКІ ЛІТЕРИ
    nop
    ldi zh, 0x01
    ldi zl, 0x00
    rcall reedEP
    rjmp ololo
 
; Прерывание MIDI
USART_rx:
    in r19, UDR
    push r19
    andi r19, 0b11110000
    cpse r19, r6
    rjmp RX_second
    sbr r21, 1
    pop r19
    reti

RX_second: 
    pop r19
    sbrs r21, 0
    reti
    cli
    sbrc r19, 7
    rjmp error_Receive
    mov zl, r19
    ldi zh, 0x00
    cbr r21, 1
    sbrc r20, 0
    rjmp s1
    rcall reedEP
    pop r19
    rjmp ololo

s1: 
    cbr r20, 0b00000001
    rcall light
    rcall writeEP
    reti

error_Receive:
    cbr r21, 1
    sei
    reti

; чтение EEP
reedEP:
    cli
sbic_eep_read:
    sbic EECR, EEWE
    rjmp sbic_eep_read
    out EEARH, zh
    out EEARL, zl
    sbi EECR, EERE
    in TEMP, EEDR
    sei
    ret

; Запись EEP
writeEP:
    cli
sbic_eep_write:
    sbic EECR, EEWE
    rjmp sbic_eep_write
    out EEARH, zh
    out EEARL, zl
    out EEDR, TEMP
    sbi EECR, EEMWE
    sbi EECR, EEWE
    sei
    ret

; Выбор режима
ololo:
    mov r16, TEMP
    cpi r16, 0xff
    breq keyscan
    lsr r16
    lsr r16
    lsr r16
    lsr r16
    cpi r16, 0
    breq j1
    cpi r16, 1
    breq j2
    cpi r16, 2
    breq j3
    cpi r16, 3
    breq j4
    cpi r16, 4
    breq j5
    cpi r16, 5
    breq j6
    cpi r16, 6
    breq j7
    cpi r16, 7
    breq j8
    cpi r16, 8
    breq j9
    cpi r16, 9
    breq j10
    cpi r16, 10
    breq j11
    cpi r16, 11
    breq j12
    rjmp keyscan

j1:  rjmp clg
j2:  rjmp clo
j3:  rjmp clred
j4:  rjmp crg
j5:  rjmp cro
j6:  rjmp crr
j7:  rjmp odg
j8:  rjmp odo
j9:  rjmp odr
j10: rjmp od2g
j11: rjmp od2o
j12: rjmp od2r
 
; Основной цикл
keyscan:
    sei
    sbis PINB, 0
    rjmp cla
    sbis PINB, 1
    rjmp cra
    sbis PINB, 2
    rjmp od1a
    sbis PINB, 3
    rjmp od2a
    sbis PINC, 0
    rjmp maon
    sbis PINC, 1
    rjmp revon
    sbis PINC, 2
    rjmp fxon
    sbis PINC, 3
    rjmp prog_fs
    rjmp keyscan

; вкл. выкл. режим програмирования
prog_fs:
    sbrc r20, 0
    rjmp s
    sbr r20, 0b00000001
    rcall light
    rcall delay
    rjmp keyscan
s:  
    cbr r20, 0b00000001
    rcall light
    rcall delay
    rjmp keyscan

; Переключатель Мастер Громкости
maon:
    ldi r17, 0b00000100
    eor TEMP, r17
    rjmp ololo

; Переключатель Ревера
revon:
    ldi r17, 0b00000001
    eor TEMP, r17
    rjmp ololo

; Переключатель Петли
fxon:
    ldi r17, 0b00000010
    eor TEMP, r17
    rjmp ololo

; клацалка релюшок и лампочок каналов
outport:  
    ldi r17, 0b00101000 ; l1
    out PORTD, r17
    nop
    ldi r18, 0b00001111
    out DDRB, r18             ; ВИПРАВЛЕНО
    nop
    out PORTB, r0             ; ВИПРАВЛЕНО
    nop
    out DDRC, r18             ; ВИПРАВЛЕНО
    nop
    out PORTC, r1             ; ВИПРАВЛЕНО
    nop
    ldi r17, 0b00110000 ; l2
    out PORTD, r17
    nop
    out PORTB, r2             ; ВИПРАВЛЕНО
    nop
    out PORTC, r3             ; ВИПРАВЛЕНО
    sbrc TEMP, 0
    cbi PORTC, 2              ; ВИПРАВЛЕНО
    nop
    ldi r17, 0b01100000 ; d1
    out PORTD, r17
    nop
    out PORTB, r4             ; ВИПРАВЛЕНО
    nop
    out PORTC, r5             ; ВИПРАВЛЕНО
    nop
    ldi r17, 0b00100000 ; d1
    out PORTD, r17
    nop
    ret

; Лампочки Prog Rev FX Master
light:
    ldi r17, 0b10100000 ; d2
    out PORTD, r17
    nop
    ldi r17, 0b1111
    ldi r18, 0b1111
    sbrc r20, 0
    cbr r17, 0b1000
    sbrc TEMP, 0
    cbr r17, 0b0010
    sbrc TEMP, 1
    cbr r17, 0b0100
    sbrc TEMP, 2
    cbr r17, 0b0001
    out DDRB, r18             ; ВИПРАВЛЕНО
    nop
    out PORTB, r17            ; ВИПРАВЛЕНО
    nop
    ldi r18, 0xFF
    ldi r17, 0b00100000 ; d2
    out PORTD, r17
    nop
    ldi r17, 0b0
    out DDRB, r17
    nop
    out PORTB, r18
    nop
    out DDRC, r17
    nop
    out PORTC, r18
    ldi r17, 0b00000000 ; d2
    out PORTD, r17
    nop
    ret

/* Задержка реакции на кнопки */
delay:
    ldi  YH, 0
    ldi  YL, 10
    ldi  XH, hi8(DELTIME) 
    ldi  XL, lo8(DELTIME)
delay_loop_X:
    sbiw xl, 1
    brne delay_loop_X
delay_loop_Y:
    sbiw yl, 1
    brne delay_loop_X     
    ret
  
chtypesell:
    ldi zh, 0x01
    ldi zl, 0x00
    rcall reedEP
    mov r23, TEMP
    mov zl, r25
    rcall reedEP
    mov r24, TEMP
    lsr r23
    lsr r23
    lsr r23
    lsr r23
    lsr r24
    lsr r24
    lsr r24
    lsr r24
    cp r23, r24
    breq nexttype
    ret

nexttype:
    cp r24, r12
    breq backtogreen
    ldi r24, 0b00010000
    add TEMP, r24
    ret

backtogreen:
    subi TEMP, 0b00100000
    ret

cla:  ; Выбор режима канала cleen
    ldi r25, 0b10
    mov r12, r25
    ldi r25, 0x01
    rcall chtypesell
    rjmp ololo

cra:   ; Выбор режима канала crunch
    ldi r25, 0b101
    mov r12, r25
    ldi r25, 0x02
    rcall chtypesell
    rjmp ololo

od1a: ; Выбор режима канала od1
    ldi r25, 0b1000
    mov r12, r25
    ldi r25, 0x03
    rcall chtypesell
    rjmp ololo
	
od2a:  ; Выбор режима канала od2
    ldi r25, 0b1011
    mov r12, r25
    ldi r25, 0x04
    rcall chtypesell
    rjmp ololo

clg: ; включатель Clean green
    ldi r17, 0b0000
    mov r0, r17
    ldi r17, 0b0000
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1110
    mov r2, r17
    ldi r17, 0b0100
    mov r3, r17
    ldi r17, 0b00001001
    mov r4, r17
    ldi r17, 0b00000011
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x00
    rcall over
    rjmp keyscan

clo:  ; включатель cleen orange
    ldi r17, 0b0000
    mov r0, r17
    ldi r17, 0b0011
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1110
    mov r2, r17
    ldi r17, 0b0100
    mov r3, r17
    ldi r17, 0b00001011
    mov r4, r17
    ldi r17, 0b00000011
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x01
    rcall over
    rjmp keyscan

clred:   ; включатель cleen red
    ldi r17, 0b0000
    mov r0, r17
    ldi r17, 0b0010
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1110
    mov r2, r17
    ldi r17, 0b100
    mov r3, r17
    ldi r17, 0b1010
    mov r4, r17
    ldi r17, 0b11
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x01
    rcall over
    rjmp keyscan

crg:   ; включатель crunch green
    ldi r17, 0b0010
    mov r0, r17
    ldi r17, 0b0000
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1101
    mov r2, r17
    ldi r17, 0b00000101
    mov r3, r17
    ldi r17, 0b00000101
    mov r4, r17
    ldi r17, 0b00000011
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x02
    rcall over
    rjmp keyscan
 
cro:   ; включатель crunch orange
    ldi r17, 0b0010
    mov r0, r17
    ldi r17, 0b0011
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1101
    mov r2, r17
    ldi r17, 0b00000101
    mov r3, r17
    ldi r17, 0b0111
    mov r4, r17
    ldi r17, 0b11
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x02
    rcall over
    rjmp keyscan

crr:  ; включатель crunch red
    ldi r17, 0b1010
    mov r0, r17
    ldi r17, 0b0011
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1101
    mov r2, r17
    ldi r17, 0b101
    mov r3, r17
    ldi r17, 0b0110
    mov r4, r17
    ldi r17, 0b11
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x02
    rcall over
    rjmp keyscan
 
odg: ; включатель od1 green
    ldi r17, 0b1001
    mov r0, r17
    ldi r17, 0b0011
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1011
    mov r2, r17
    ldi r17, 0b110
    mov r3, r17
    ldi r17, 0b1101
    mov r4, r17
    ldi r17, 0b10
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x03
    rcall over
    rjmp keyscan

odo:  ; включатель od1 orange
    ldi r17, 0b0001
    mov r0, r17
    ldi r17, 0b0010
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1011
    mov r2, r17
    ldi r17, 0b110
    mov r3, r17
    ldi r17, 0b1111
    mov r4, r17
    ldi r17, 0b10
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x03
    rcall over
    rjmp keyscan

odr:   ; включатель od1 red
    ldi r17, 0b1001
    mov r0, r17
    ldi r17, 0b0010
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b1011 ; l2
    mov r2, r17
    ldi r17, 0b110
    mov r3, r17
    ldi r17, 0b1110
    mov r4, r17
    ldi r17, 0b10
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x03
    rcall over
    rjmp keyscan

od2g:  ; включатель od2 green
    ldi r17, 0b1101
    mov r0, r17
    ldi r17, 0b00011
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b0111
    mov r2, r17
    ldi r17, 0b111
    mov r3, r17
    ldi r17, 0b1101
    mov r4, r17
    ldi r17, 0b01
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x04
    rcall over
    rjmp keyscan
 
od2o:  ; включатель od2 orange
    ldi r17, 0b0101
    mov r0, r17
    ldi r17, 0b0010
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b0111
    mov r2, r17
    ldi r17, 0b111
    mov r3, r17
    ldi r17, 0b1111
    mov r4, r17
    ldi r17, 0b01
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x04
    rcall over
    rjmp keyscan

od2r: ; включатель od2 red
    ldi r17, 0b1101
    mov r0, r17
    ldi r17, 0b0010
    sbrc TEMP, 1
    sbr r17, 0b0100
    sbrc TEMP, 2
    sbr r17, 0b1000
    mov r1, r17
    ldi r17, 0b0111
    mov r2, r17
    ldi r17, 0b111
    mov r3, r17
    ldi r17, 0b1110
    mov r4, r17
    ldi r17, 0b01
    mov r5, r17
    ldi zh, 0x01
    ldi zl, 0x04
    rcall over
    rjmp keyscan

over: 
    rcall outport
    rcall light
    nop
    rcall writeEP
    ldi zl, 0x00
    rcall writeEP
    rcall delay
    ret
    rjmp reset