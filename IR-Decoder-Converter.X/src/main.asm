;*******************************************************************************                                        *
;                                                                              *
;    SOFTWARE IS PROVIDED "AS IS".  Northern Lights Electronic Design, LLC AND ITS LICENSORS EXPRESSLY      *
;    DISCLAIM ANY WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING  *
;    BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS    *
;    FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL          *
;    Northern Lights Electronic Design, LLC OR ITS LICENSORS BE LIABLE FOR ANY INCIDENTAL, SPECIAL,         *
;    INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO     *
;    YOUR EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR    *
;    SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY   *
;    DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER      *
;    SIMILAR COSTS.                                                            *
;                                                                              *
;    To the fullest extend allowed by law, Northern Lights Electronic Design, LLC and its licensors         *
;    liability shall not exceed the amount of fee, if any, that you have paid directly to NLED to use this software.
;                                                                              *
;    Northern Lights Electronic Design, LLC PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
;    THESE TERMS.                                                              *
;                                                                              *
; Original Author: Jeffrey Nygaard
; Company: Northern Lights Electronic Design, LLC
; Contact: JNygaard@NLEDShop.com
; Date Updated: May  5, 2018
; Webpage: www.NLEDShop.com
;Title: NLED Modulated NEC Infrared Decoder and Encoder
;Project IDE: MPLABX v4.05
;Compiler: MPASM v5.76 (shouldn't ever matter)
;License: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) https://creativecommons.org/licenses/by-nc/4.0/legalcode


;Supported Processors For Reception(short list): PIC12F1822, PIC12F1840
;Supported Processors For Full Duplex(short list): PIC12F1572
;MCU Requirements:
;   - TMR0, TMR1, TMR2
;   - one additional 16-bit timer required for transmission - used here is a unused PWM with dedicated timer
;   - 1x UART
;   - 1x PWM output if transmission is utilized
;   - 32Mhz or 8 MIPs - otherwise timings would be all wrong

;Description:
;PLEASE READ THE README.TXT FILE THAT WAS INCLUDED 

;Expects the extended NEC Protocol, with 16-bit Address, rather than a 8-bit address with 8-bit inverse
;Note: NEC sends LSB first for extended address, UART transmits MSB first since that is the common standard

;Reception UART Transmits: 255 -> 254 -> AdrMSB -> AdrLSB -> Key Number
;Example: AddressLow - 0x00, AddressHi - 0xEF, KeyID#4(key IDnumbers start at 0)
;       255 -> 254 -> 239(0xEF) -> 0(0x00) -> 4
;   Repeat: 250 -> 253, repeats while key is held

;Transmission UART Receives: 
;	251 -> 122 -> 252 -> 123 -> AdrMSB -> AdrLSB - Command ID -> Number of Repeats
;Example: 
;	AddressLow - 0x00, AddressHi - 0xEF, KeyID#4(key IDnumbers start at 0), 2 Repeats
;	251 -> 122 -> 252 -> 123 -> 239(0xEF) -> 0(0x00) - 4  -> 2

;How it Works:
;   A lot of confusing code. But basically a progressive state machine reads the NEC protocol step by step. All interrupt
;       based with some program loop data crunching. Demodulated is straight forward, modulated reception is even
;       more confusing, it either counts carrier pulses to establish a hi/low time, or uses timers.
;   I learned a lot doing this project, most of all, I learned I should have done things completely differently. Not worth rewriting.

;Usage:
;   1. Decide what functions you will need and choose a I.R. reciever module, transciever, and/or I.R. LED
;   2. See datasheet for specifics.
;   3. Leave BAUD jumper open or closed to select the baud rate
;   4. To build a HEX file, select the compile DEFINEs to select usage.
;   5. Program HEX file
;   6. Power it up, connect serial host device and start using, if it doesn't work double check Baud rate settings

;Found these projects and links useful:
;https://reference.digilentinc.com/learn/courses/unit-5/start
;http://www.sbprojects.com/knowledge/ir/nec.php
;https://www.laser.com/dhouston/ir-rf_fundamentals.html
;http://www.ti.com/lit/an/slaa644b/slaa644b.pdf
;https://www.vishay.com/docs/80071/dataform.pdf

;*******************************************************************************

;Processor Notes:
;At 8 MIPS it is 125nS instruction time
;TMR2 at 1:1, 1:1, takes 31.875uS for PR2 of 255
;Max TMR2 duration is 32.64mS
;TMR0 - Pulse Timer
;TMR1 - reset timer
;TMR2 - master timer
;UART - connected to MCU
;PWM2 - IRTX

;RA0/ICSPDAT = TX
;RA1/ICSPCLK = RX
;RA2 = IRRX - INT pin
;RA3/MCLR = GPIO input - Configuration Mode IO pin, tie to HEADER or GND, pulled high - selects modulated or demodulated usage.
;RA4 = IRTX - PWM2
;RA5 = GPIO input - Baud rate Jumper, pulled high. Open = 19200 baud, Closed = 250K baud

;NOTE: On the v1b & v1c hardware  ICSP CLK and DAT are physically swapped  to maintain compatibility with NLED addon
;           headers and connectors. That makes it harder to flash the program, but otherwise is not an issue.

;Protocol Notes:
;38Khz carrier is 26.31uS officially, about 2.6uS low, 24.6uS high on my remote as measured
;562.5uS bit time, which is 21? carrier cycles

;NEC Main Code
;a 9ms leading pulse burst (16 times the pulse burst length used for a logical data bit)
;a 4.5ms space
;the 8-bit address for the receiving device
;the 8-bit logical inverse of the address
;the 8-bit command
;the 8-bit logical inverse of the command
;a final 562.5µs pulse burst to signify the end of message transmission.
;bits take 1.125mS for 0, and 2.25mS for 1

;REPEAT CODES
;If the key on the remote controller is kept depressed, a repeat code will be issued, typically around 40ms after the 
;    pulse burst that signified the end of the message. A repeat code will continue to be sent out at 108ms intervals,
;    until the key is finally released. The repeat code consists of the following, in order:
;a 9ms leading pulse burst
;a 2.25ms space
;a 562.5µs pulse burst to mark the end of the space (and hence end of the transmitted repeat code).

;*******************************************************************************
; Start  Defines
;*******************************************************************************

#include "defines.inc" ;hardware profile, RAM defines, software constants

;*******************************************************************************
;Start Compile Time Flags
;Uncomment to enable, comment out to disable
;*******************************************************************************

;Will only work on some processors, see MCU requirements
#define cEnableNECTransmission
#define cPreventTXReception ;will disable reception when transmitting, preventing a transciever from receiving its 
                                                   ;    own message but it could miss receiving a message during the process. Once the 
                                                    ;   message is transmitted, reception continues normally.

;edit these to suit, on init.inc there is a list of the other available baud rates with their SPBRG values
#define cSlowBaudRate .103 ;19200 baud
#define cFastBaudRate .7 ;250k baud

;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    Initialize                   ; go to beginning of program

;*******************************************************************************

ISR_VECT  CODE    0x0004           ; processor ISR vector
;The latency for synchronous interrupts is three or four instruction cycles. For
;asynchronous interrupts, the latency is three to fiveinstruction cycles, depending on when the interrupt occurs.

;Critical/Core registers are automatically saved to the shadow registers, see datasheet for listing
;-----------------------------------------------------------------------------------------

#ifdef cEnableNECTransmission
    ;For Transmission use, check if UART has received any bytes
    banksel PIR1
    BTFSC PIR1, RCIF
        goto HandleReceivedByte ; check if needs paging, retfie out
#endif

;-----------------------------------------------------------------------------------------

    ;INT -  then it is a edge detect interrupt
    BCF INTCON, INTF ;clear interrupt flag bit

;-----------------------------------------------------------------------------------------

#ifdef cPreventTXReception
    BTFSS DataWaitingTransmission
        bra skipReceptionDisable
    ;a interrupt has fired, but device is transmitting a message, could be its own message, ignore
    banksel PIR1
    clrf PIR1 ;clear all potential interrupts(excluding INTF which was done above, not great but works
    retfie
skipReceptionDisable
#endif

;-----------------------------------------------------------------------------------------

    banksel PIR1
    BTFSS PIR1, TMR1IF
        bra isrSkip
    ;TMR1 IF flag is set

    BTFSS RepeatFlag ;check if waiting for repeat frames
        bra skipRepeat
    ;repeat flag is set, that means it needs more time to wait for repeat packets before timing out
    BCF RepeatFlag ;clear flag, only toggles once, next time it times out
    BCF PIR1, TMR1IF

        bra isrSkip

skipRepeat
    ;TMR1 has interrupted, reset State Machine and wait for next remote command
    BCF PIR1, TMR1IF
    clrf ReceiveState

    banksel PIE1
    BTFSS PIE1, TMR1IE
        bra isrSkip
    ;reset timer has timed out, turn off enable and wait for next remote command
    BCF PIE1, TMR1IE    ;if enabled turn off IE and retfie
        retfie
    ;TMR1 and RepeatFlag handled, continue ISR

isrSkip ;no TMR1 interrupt, continue
    BTFSC ModulationConvertFlag ;flag set only at power up
    goto  ReadDemodulatedEncoding
    ;else read Modulated Encoding
    goto ReadModulatedEncoding
    ;never gets here, both above functions RETFIE

;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

;MAIN_PROG CODE                      ; let linker place main program - screws things up

START

    BANK1
    clrf transmitFuncState
    BANK0
    clrf ReceiveState ;set to initial state

    BCF INTCON, IOCIF
    BCF INTCON, INTF ;clear interrupt flag bit before setting GIE so it doens't interrupt right away
    BSF INTCON, GIE ;enables global interrupts
    BSF INTCON, PEIE ;TMR1 won't interrupt unless this is set, but INT will

MainLoop
    clrwdt

   BTFSC ConvertToBytesFlag ;waits til validated NEC code has been received
   call FunctionToSerialBytes

    #ifdef cEnableNECTransmission
    BTFSC DataWaitingTransmission
    call TransmitModulatedEncoding
    #endif

      bra MainLoop

;*******************************************************************************

FunctionToSerialBytes ;both reception methods use this same function
;converts the 32 bytes of 0 or 1 as a bit, into 4 bytes(8x4=32)
    BCF ConvertToBytesFlag ;clear flag

    ;set FSR1 to starting byte
    movlw 0x20 ;address of bank 0 RAM
    movwf FSR1H
    movlw 0x00 ;address of bank 0 RAM
    movwf FSR1L

    ;start converting bytes to bits
    call FunctionConvertByte
    movwf ConvertedByte1 ;AddressLo
    call FunctionConvertByte
    movwf ConvertedByte2 ;AddressHi
    call FunctionConvertByte
    movwf ConvertedByte3 ;Command
    call FunctionConvertByte
    movwf ConvertedByte4 ;Command Inverse
    ;now all converted to normal bit order ready to use

    ;start sending converted values out UART
    ;Choice of two output functions that sends the data out
    bra SendCommandCode ;builds a command code sequence with byte framing, checks command inverse
    ;bra SendRawBytes ;sends the raw, AddressLo, AddressHi, Command, Command Inverse
        ;both functions return to program loop, so use BRA instead of CALL

;-----------------------------------------------------------------------------------------

FunctionConvertByte
    ;NEC sends LSB first, reads each stored byte in RAM and converts it to a bit
    clrf Scratch

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch, 0

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch, 1

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch,2

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch,3

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch,4

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch,5

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch,6

    MOVIW FSR1++ ;move FSR byte value to working, post increment
    BTFSC WREG, 0 ;check if logical 1 or 0 stored in byte
    BSF Scratch,7

    movfw Scratch ;returns with result in WREG
        return

;*******************************************************************************

SendCommandCode
    ;uses the received NEC data bytes to build a command to send to the main MCU
    ; This assumes that the addressing is done using the non-standard 16-bit method, 
        ; rather than 8-bit address + 8-bit address inverse.

    ;It is easier to update the main MCU, so this will send the IR remote address and the main MCU can
        ; choose to use or ignore if the remote is the wrong address. Or it can use all addresses, main MCU's choice

    ;first check if the command and command inverse match
    movfw ConvertedByte4
    COMF WREG, w ;result stored in WREG
    ;WREG holds inverted Command Inverse, so should be same as Command
    xorwf ConvertedByte3, w
    BTFSS STATUS, Z
        return ;leave if the Command and Command Inverse are mismatched, could handle or send error

    ;sends: 255 -> 254 -> AddressHi -> AddressLo -> Command

    banksel TXREG
    movlw .255 ;send framing byte
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1

    banksel TXREG
    movlw .254  ;send framing byte
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1
    ;sent framing bytes, could have done a BREAK or similar

    banksel TXREG
    movfw ConvertedByte2 ;sending Address MSB first that is more standard
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1

    banksel TXREG
    movfw ConvertedByte1 ;then Address LSB
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1

    banksel TXREG
    movfw ConvertedByte3 ; 
    movwf TXREG
    ; all done sending bytes
        return ; to program loop

;-----------------------------------------------------------------------------------------

SendRawBytes
    ;sends the raw, AddressLo, AddressHi, Command, Command Inverse
    banksel TXREG
    movfw ConvertedByte1
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1

    banksel TXREG
    movfw ConvertedByte2
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1

    banksel TXREG
    movfw ConvertedByte3
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1

    banksel TXREG
    movfw ConvertedByte4
    movwf TXREG
    ; all done sending bytes
        return ; to program loop

;*******************************************************************************
;External Assembly Includes - Easy way to break code onto multiple files but keep memory linear to avoid page boundaries
;*******************************************************************************

#include "modulated-reception.inc"
#include "demodulated-reception.inc"

#ifdef cEnableNECTransmission
#include "transmission.inc"
#endif

#include "init.inc"

;*******************************************************************************

    END
