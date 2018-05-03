;*******************************************************************************
;File is placed at in main.asm using #include
;This is in iterrupt space, so account for that. Such as core registers are restored on exit
;*******************************************************************************

ReadDemodulatedEncoding
    movfw ReceiveState
    BRW 
    bra DemodulatedLeaderPulseWait
    bra DemodulatedLeaderPulseFinish
    bra DemodulatedSpacePulseFinish
    bra DemodulatedCollectDataBits
    bra DemodulatedWaitForRepeat
    bra DemodulatedConfirmRepeatPulse

;-----------------------------------------------------------------------------------------

DemodulatedLeaderPulseWait
    ;detected a falling edge
        incf ReceiveState, f ;set to 
        clrf CounterMain ;counts bits

        banksel TMR1 ;clear reset timer
        clrf TMR1H
        clrf TMR1L
        banksel PIR1
        BCF PIR1, TMR1IF
        banksel PIE1 ;enable reset timer interrupt enable, so it can reset the state machine after inactivity
        BSF PIE1, TMR1IE

        banksel TMR2 ;clear master timer
        clrf TMR2 ;clear master timer
        movlw .138 ;140 ticks at 64uS is 8960uS, lower value gives some wiggle room
        movwf PR2 ;reset PR2 for now
        banksel PIR1
        BCF PIR1, TMR2IF ;clear flag

        banksel OPTION_REG
        BSF OPTION_REG, INTEDG ;set to rising edge
        BCF INTCON, INTF     ; need to clear this???

           retfie

;-----------------------------------------------------------------------------------------

DemodulatedLeaderPulseFinish
     ;measuring space between last burst pulse and this current pulse
    ;interrupt flag is set after 140 ticks of 64uS = 9mS
        banksel PIR1
        BTFSS PIR1, TMR2IF
            retfie ;pulse detected before timer elasped, HANDLE

        ;the flag was set, meaning there was at least 8.8mS between pulses
        ;leader pulse confirmed
        incf ReceiveState, f ;set to next state

        banksel TMR1 ;clear reset timer
        clrf TMR1H
        clrf TMR1L

        ;setup master timer for space measuring
        banksel TMR2 ;clear master timer
        clrf TMR2
        movlw .68 ; 70 ticks at 1:8, 1:64 = 64uS per tick, 68 x 64uS = 4352uS
        movwf PR2
        banksel PIR1
        BCF PIR1, TMR2IF

    ;reset FSR0 to starting byte, write to shadow registers rather than normal
        ; or the values won't be applied since the automatic context saving on retfie will overwrite the changes
    banksel FSR0L_SHAD
    movlw 0x20 ;address of bank 0 RAM
    movwf FSR0H_SHAD
    movlw 0x00 ;address of bank 0 RAM
    movwf FSR0L_SHAD

    banksel OPTION_REG
    BCF OPTION_REG, INTEDG ;set to falling edge
    BCF INTCON, INTF     ; need to clear this???
        retfie

;-----------------------------------------------------------------------------------------

DemodulatedSpacePulseFinish
        banksel PIR1
        BTFSS PIR1, TMR2IF
             bra EarlyPulse ;pulse detected before timer elasped, HANDLE

        ;high pulse detected, with minimum time of 512uS, or end of space
        incf ReceiveState, f ;set to next state

        banksel TMR1 ;clear reset timer
        clrf TMR1H
        clrf TMR1L

;set the postscaler to 1:1 - so can use the TMR2 value and IF, 8uS ticks
        banksel T2CON
        BCF T2CON, TMR2ON
        movlw  b'0000011';TMR2 on, Post 1:1 - Pre 1:64 
        movwf T2CON
        BSF T2CON, TMR2ON
        ;setup master timer for space measuring
        banksel TMR2 ;clear master timer
        clrf TMR2 ;reset timer
        movlw .255 ;not using interrupt flag, using timer value
        movwf PR2
        banksel PIR1
        BCF PIR1, TMR2IF

           retfie

EarlyPulse
    retfie

;-----------------------------------------------------------------------------------------

DemodulatedCollectDataBits
  ;detects how long the space was since bit start
  ;test for 1.125mS and 2.25mS
    banksel PIR1
    BTFSS PIR1, TMR2IF
        bra DCDBa
    ;flag was set, it was at least 2040uS

    BCF PIR1, TMR2IF
    banksel TMR2 ;clear master timer
    clrf TMR2

    movlw .1
    MOVWI FSR0++ ;move working into FSR0, post increment
    ;since this ran from an ISR, with automatic context saving, it restores FSR0 value on exit
    movfw FSR0L
    banksel FSR0L_SHAD
    movwf FSR0L_SHAD    ;so save new FSR value to the shadow register so it is maintained between ISRs

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L
    banksel TMR2 ;reset master timer
    clrf TMR2

    incf CounterMain, f
    movfw CounterMain
    xorlw .32 ;number of bits to collect, 32 bits + 1 end of message
    BTFSC STATUS, Z
          bra DCDBLc ;it has collected all the bits, move state forward
    ;hasn't collected enough bits yet, continue

        retfie

DCDBa
    banksel TMR2
    movfw TMR2  ;should be a 70 count, 560uS
    addlw .188 
    BTFSS STATUS, C
        bra DCDBLb
    ;562.5uS  detected

    movlw .0
    MOVWI FSR0++ ;move working into FSR0, post increment
    ;since this ran from an ISR, with automatic context saving, it restores FSR0 value on exit
    movfw FSR0L
    banksel FSR0L_SHAD
    movwf FSR0L_SHAD    ;so save new FSR value to the shadow register so it is maintained between ISRs

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L
    banksel TMR2 ;reset master timer
    clrf TMR2

    incf CounterMain, f
    movfw CounterMain
    xorlw .32 ;number of bits to collect, 32 bits + 1 end of message
    BTFSC STATUS, Z
        bra DCDBLc ;it has collected all the bits, move state forward
    ;hasn't collected enough bits yet, continue

        retfie

DCDBLb
    retfie

DCDBLc
    ;All the bits are collected, move state machine forward
    incf ReceiveState, f ;set to 
    BSF RepeatFlag ;set the flag to indicate repeat packets may arrive
    BSF ConvertToBytesFlag ;set flag that is polled in program loop that sends the command codes

    ;set it back
    banksel T2CON
    BCF T2CON, TMR2ON
    movlw  b'0011111';TMR2 on, Post 1:8 - Pre 1:64 
    movwf T2CON
    BSF T2CON, TMR2ON

        retfie

;-----------------------------------------------------------------------------------------

DemodulatedWaitForRepeat
    ;about 40mS after end burst, it would send 9mS leader burst again
    ;TMR1 for 40mS would be 0x9C40, but make it 30mS which is 0x7530

        ;check if matches
        banksel TMR1
        movfw TMR1H
        addlw .139 ;256 - 0x75
        BTFSS STATUS, C
        retfie ;pulse after end of message came early

        movfw TMR1L
        addlw .208 ;256 - 48(0x30)
        BTFSS STATUS, C
        retfie ;pulse after end of message came early

        ;it detected a pulse within time to possibly be a repeat burst

        incf ReceiveState, f ;set t
        BSF RepeatFlag ;set the flag to indicate repeat packets may arrive

        banksel TMR1 ;clear reset timer
        clrf TMR1H
        clrf TMR1L

         ;setup master timer to detect repeat leader burst and space
        banksel TMR2 ;clear master timer
        clrf TMR2 ;clear master timer
        movlw .158 ;160 at 64us per tick is 10.25mS, which is the time for the 9mS leader and 2.25mS space, burst not measured
        movwf PR2 ;reset PR2 for now
        banksel PIR1
        BCF PIR1, TMR2IF ;clear flag

             retfie

;-----------------------------------------------------------------------------------------

DemodulatedConfirmRepeatPulse
    banksel PIR1
    BTFSS PIR1, TMR2IF
        retfie 

    ;repeat confirmed
    BCF PIR1, TMR2IF
    BSF RepeatFlag ;set the flag to indicate repeat packets may arrive

    decf ReceiveState, f ;setup for next repeat pulse

    banksel TMR1 ;clear reset timer
    clrf TMR1H
    clrf TMR1L

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


