;*******************************************************************************
;File is placed at in main.asm using #include
;This is in iterrupt space, so account for that. Such as core registers are restored on exit
;*******************************************************************************

ReadModulatedEncoding
    movfw ReceiveState
    BRW 
    bra PulseLeaderWait
    bra CheckCarrierTiming ;PulseLeaderMeasure
    bra PulseSpaceMeasure
    bra CheckCarrierTiming ;CollectPulseDistanceHi
    bra CollectPulseDistanceLo
    bra CheckCarrierTiming ;CollectEndPulse
    bra WaitForRepeatPulse
    bra CheckCarrierTiming ;CollectRepeatLeaderPulse
    bra MeasureRepeatCode
    bra CheckCarrierTiming ;CollectRepeatEndPulse

;-----------------------------------------------------------------------------------------

CheckCarrierTiming ;general use, no state#
    ;TMR2 not elapsed, get time value

    ;pulse value of 200 is 25uS, 216 is 27uS
    ;but there is a latency before the ISR fires and additional latency getting to this point
        ; values are guessed at via trial and error, could make them real tight with a bit of effort, but close enough

    banksel TMR0
    movfw TMR0 ;move to WREG
    addlw .60 ;add 56(loosened timing), if over 200 or more register will roll over and set Carry flag
    BTFSS STATUS, C
        bra IgnorePulse

    ;TMR is 200 or more and under 216

    movfw TMR0 ;move to WREG
    addlw .38 ;add 40(loosened timing), if over 216 or more it will roll over
    BTFSC STATUS, C
        bra IgnorePulse
    ;proper pulse length detected

    banksel TMR0 ;clear pulse timer
    clrf TMR0

    movfw ReceiveState
    BRW 
    retfie ;PulseLeaderWait - doesn't get here
    bra PulseLeaderMeasure
    retfie ;PulseSpaceMeasure - doesn't get here
    bra CollectPulseDistanceHi
    retfie ;CollectPulseDistanceLo  - doesn't get here
    bra CollectEndPulse
    retfie ;WaitForRepeatPulse - doesn't get here
    bra CollectRepeatLeaderPulse
   retfie ;MeasureRepeatCode - doesn't get here
    bra CollectRepeatEndPulse

;-----------------------------------------------------------------------------------------

PulseLeaderWait     ;state 0
;if first pulse, init timers
    banksel TMR0 ;clear pulse timer
    clrf TMR0

    incf ReceiveState, f ;set to measuring burst

    clrf PulseCountMSB ;reset counter registers
    clrf PulseCountLSB
    clrf CounterMain ;counts bits

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L
    banksel PIR1
    BCF PIR1, TMR1IF
    banksel PIE1 ;enable reset timer interrupt enable, so it can reset the state machine after inactivity
    BSF PIE1, TMR1IE

    ;reset FSR0 to starting byte, write to shadow registers rather than normal
        ; or the values won't be applied since the automatic context saving on retfie will overwrite the changes
    banksel FSR0L_SHAD
    movlw 0x20 ;address of bank 0 RAM
    movwf FSR0H_SHAD
    movlw 0x00 ;address of bank 0 RAM
    movwf FSR0L_SHAD

        retfie

;-----------------------------------------------------------------------------------------

PulseLeaderMeasure ;state 1
    ;measuring burst state, comes here many times waiting for master timer to count up

    ;pulse detected, increment counter
     incf PulseCountMSB,f
     incfsz PulseCountLSB,f
     decf PulseCountMSB,f

    ;check if matches
    movfw PulseCountMSB
    xorlw 0x01
    BTFSS STATUS, Z
        bra PBMa

    movfw PulseCountLSB
    ;addlw .171 ;171 is exactly for my controller, but don't count on it
    addlw .200 ;catches a lot less than the 9mS required
    BTFSS STATUS, C
       bra PBMa
    ;at least 342 pulses detected = leader burst detected

    incf ReceiveState, f ;set to measuring space

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L

    ;setup TMR2 for space measuring
    banksel TMR2 ;clear master timer
    movlw .65 ; 70 ticks at 1:8, 1:64 = 64uS per tick, 70 x 64uS = 4480uS
    movwf PR2
    clrf TMR2
    banksel PIR1
    BCF PIR1, TMR2IF

        retfie

;-----------------------------------------------------------------------------------------

PulseSpaceMeasure ;state 2
    ;measuring space between last burst pulse and next pulse
    ;interrupt flag is set after 70 ticks of 64uS = 4.5mS
    banksel PIR1
    BTFSS PIR1, TMR2IF
        retfie ;pulse detected before timer elasped, HANDLE

    ;there was at least 4.5mS between pulses, space detected
    BCF PIR1, TMR2IF ;clear flag

    banksel TMR2 ;clear master timer
    movlw .255
    movwf PR2 ;reset PR2 for now
    clrf TMR2 ;clear master timer

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L

    incf ReceiveState, f ;set to measuring space
    clrf PulseCountLSB ;clear for next state

       retfie

;-----------------------------------------------------------------------------------------
;Logical '0' ? a 562.5µs pulse burst followed by a 562.5µs space, with a total transmit time of 1.125ms
;Logical '1' ? a 562.5µs pulse burst followed by a 1.6875ms space, with a total transmit time of 2.25ms

CollectPulseDistanceHi ;state 3
    ;counts pulses, detects bit start with pulse burst of 562uS
    incf PulseCountLSB, f
    movfw PulseCountLSB
    xorlw .16 ; not what I counted, but works - 19 was exact, made it 16 for some wiggle room
    BTFSS STATUS, Z
        retfie ;not yet counted enough pulses

    ;counted 21 pulses, a bit is confirmed
    incf ReceiveState, f ;set to measuring pulse distance lo
    clrf PulseCountLSB

    banksel TMR2 ;clear master timer
    clrf TMR2

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L

      retfie

CollectPulseDistanceLo ;state 4
    ;detects how long the space was since bit start
    ;test for 562.5uS and 1.6875mS space

    banksel TMR2
    movfw TMR2  ;should be a 210 count
    addlw .47 ;this would make it roll over, added 1 for margin
    BTFSS STATUS, C
        bra CPDLa
    ;1.6875mS detected, equals logical 1

    movlw .1
    MOVWI FSR0++ ;move working into FSR0, post increment
    ;since this ran from an ISR, with automatic context saving, it restores FSR0 value on exit
    movfw FSR0L
    banksel FSR0L_SHAD
    movwf FSR0L_SHAD    ;so save new FSR value to the shadow register so it is maintained between ISRs

    incf CounterMain, f
    movfw CounterMain
    xorlw .32  ;number of bits to collect
    BTFSC STATUS, Z
        bra StartEndReceive

    decf ReceiveState, f ;set to PulseDistanceHi

       retfie

CPDLa
    banksel TMR2
    movfw TMR2  ;should be a 70 count
    addlw .187 ;
    BTFSS STATUS, C
        bra CPDLb
    ;562.5uS  detected, equals logical 0

    movlw .0
    MOVWI FSR0++ ;move working into FSR0, post increment
    ;since this ran from an ISR, with automatic context saving, it restores FSR0 value on exit
    movfw FSR0L
    banksel FSR0L_SHAD
    movwf FSR0L_SHAD    ;so save new FSR value to the shadow register so it is maintained between ISRs

    incf CounterMain, f
    movfw CounterMain
    xorlw .32 ;number of bits to collect
    BTFSC STATUS, Z
        bra StartEndReceive

    decf ReceiveState, f ;set to PulseDistanceHi

       retfie

StartEndReceive 
    incf ReceiveState, f ;set to collectendpulse
    clrf PulseCountLSB ;clear counter
        retfie

;-----------------------------------------------------------------------------------------

CollectEndPulse  ;state 5
    ;counts pulses, detects end bit of 562uS length
    incf PulseCountLSB, f
    movfw PulseCountLSB
    xorlw .16 ; not what I counted, but works - 19 was exact, made it 16 for some wiggle room
    BTFSS STATUS, Z
      retfie ;not yet counted enough pulses
    ;it has collected enough pulses to indicate the last pulse burst to mark the end of the space
   
    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L
    banksel PIR1
    BCF PIR1, TMR1IF

    incf ReceiveState, f ;set to WaitForRepeatPulse

    BSF ConvertToBytesFlag ;set flag that is polled in program loop that sends the command codes

      retfie

;-----------------------------------------------------------------------------------------

WaitForRepeatPulse ;state 6
    ;about 40mS after end burst, it would send 9mS leader burst again
;TMR1 for 40mS would be 0x9C40, but make it 30mS which is 0x7530

    ;check if matches
    banksel TMR1
    movfw TMR1H
    ;xorlw 0x75
    addlw .139 ;256 - 0x75
    BTFSS STATUS, C
       bra WFRa

    movfw TMR1L
    addlw .208 ;256 - 48(0x30)
    BTFSS STATUS, C
      bra WFRa
    ;it detected a pulse within time to possibly be a repeat burst

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L

    incf ReceiveState, f ;set to CollectRepeatLeaderPulse
    clrf PulseCountMSB ;reset counter registers
    clrf PulseCountLSB

        retfie

WFRa
;pulse came early
    retfie

;-----------------------------------------------------------------------------------------

CollectRepeatLeaderPulse ;state 7
    ;repeat pulse validating, start counting, pretty much the same as PulseLeaderMeasure

    ;pulse detected, increment counter
     incf PulseCountMSB,f
     incfsz PulseCountLSB,f
     decf PulseCountMSB,f

    ;check if matches
    movfw PulseCountMSB
    xorlw 0x01
    BTFSS STATUS, Z
       bra PBMa

    movfw PulseCountLSB
    ;addlw .171 ;171 is exactly for my controller, but don't count on it
    addlw .181 ;catches 8.6mS
    BTFSS STATUS, C
      bra PBMa
    ;at least 342 pulses detected = leader burst detected

    incf ReceiveState, f ;set to measuring space

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L

    ;setup TMR2 for space measuring
    banksel TMR2 ;clear master timer
    movlw .34 ; 35 ticks at 1:8, 1:64 = 64uS per tick, 35 x 64uS = 2240uS
    movwf PR2
    clrf TMR2
    banksel PIR1
    BCF PIR1, TMR2IF

        retfie

;-----------------------------------------------------------------------------------------

MeasureRepeatCode ;state 8
    ;measuring space between last burst pulse and next pulse
    ;interrupt flag is set after 70 ticks of 64uS = 4.5mS
    banksel PIR1
    BTFSS PIR1, TMR2IF
        retfie ;pulse detected before timer elasped, HANDLE

    ;there was at least 2.25mS between pulses, space detected
    BCF PIR1, TMR2IF ;clear flag

    banksel TMR2 ;clear master timer
    movlw .255
    movwf PR2 ;reset PR2 for now
    clrf TMR2 ;clear master timer

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L

    incf ReceiveState, f ;set to CollectRepeatEndPulse
    clrf PulseCountLSB ;clear for next state

       retfie

;-----------------------------------------------------------------------------------------

CollectRepeatEndPulse ;state 9
    ;counts pulses, detects bit start with pulse burst of 562uS
    incf PulseCountLSB, f
    movfw PulseCountLSB
    xorlw .16 ; not what I counted, but works - 19 was exact, made it 16 for some wiggle room
    BTFSS STATUS, Z
      retfie ;not yet counted enough pulses

    ;counted enough pulses, a bit is confirmed
    ;now it is complete

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L
    banksel PIR1
    BCF PIR1, TMR1IF

    ;setup to receive additional repeat pulses
    movlw .6 ;start on WaitForRepeatPulse
    movwf ReceiveState

    BSF RepeatFlag ;set flag saying repeats are coming in need to adjust reset timer usage, since it times out 
            ;after 65mS, and spacing is around 96mS between repeat bursts

    ;a repeat burst has been received, indicate to UART
    banksel TXREG
    movlw .250 ;command frame code
    movwf TXREG

    BTFSS TXSTA, TRMT ;check if TSR is empty, wait if its not
    bra $-1

    banksel TXREG
    movlw .253 ;indicates REPEAT code to UART
    movwf TXREG

        retfie

;-----------------------------------------------------------------------------------------

IgnorePulse
  ;  banksel TMR2 ;clear master timer
  ;  clrf TMR2
    banksel TMR0 ;clear pulse timer
    clrf TMR0 

        retfie

CPDLb
    ;Lo pulse length not long enough to be 1 or 0
    banksel TMR2 ;clear master timer
    clrf TMR2
     retfie

PSMa
    ;space too short
    retfie

PBMa
    ;burst too short - will happen a lot while it counts up
 retfie


