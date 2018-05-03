;*******************************************************************************
;File is placed at in main.asm using #include
;*******************************************************************************

;-----------------------------------------------------------------------------

HandleReceivedByte ;byte has been received from host, store it for later transmission
 ;   banksel PIR1     ;RCIF is set, banked to PIR1
  ;  BCF PIR1, RCIF ;clear flag

;Recieve: Framing -> AdrMSB -> AdrLSB - Command
;251 -> 122 -> 252 -> 123 -> UNLOCKED -> AdrMSB -> AdrLSB -> command -> number of repeats
    BANK1 ;where the transmit function variables are stored
    movfw transmitFuncState
    BRW
    bra WaitKeyByte1
    bra WaitKeyByte2
    bra WaitKeyByte3
    bra WaitKeyByte4
    bra StoreAdrMSB
    bra StoreAdrLSB
    bra StoreCommand
    bra StoreRepeatNumber

WrongByteReceived
    ;the byte it received was not expected, restart state machine
    BANK1;already banked, but play it safe
    clrf transmitFuncState
     retfie

WaitKeyByte1
    banksel RCREG 
    movfw RCREG
    BANK1 ;where transmitFuncState is stored
    xorlw .251
    BTFSS STATUS, Z
         bra WrongByteReceived
    incf transmitFuncState, f ;if RCREG==251, increment state machine
       retfie

WaitKeyByte2
    banksel RCREG 
    movfw RCREG
    BANK1 ;where transmitFuncState is stored
    xorlw .122
    BTFSS STATUS, Z
        bra WrongByteReceived
    incf transmitFuncState, f ;if RCREG==122, increment state machine
       retfie

WaitKeyByte3
    banksel RCREG 
    movfw RCREG
    BANK1 ;where transmitFuncState is stored
    xorlw .252
    BTFSS STATUS, Z
        bra WrongByteReceived
    incf transmitFuncState, f ;if RCREG==252, increment state machine
       retfie

WaitKeyByte4
    banksel RCREG 
    movfw RCREG
    BANK1 ;where transmitFuncState is stored
    xorlw .123
    BTFSS STATUS, Z
        bra WrongByteReceived
    incf transmitFuncState, f ;if RCREG==123, increment state machine
       retfie

StoreAdrMSB
    banksel RCREG 
    movfw RCREG
    BANK1
    movwf transmitAdrMSB
    incf transmitFuncState, f 
        retfie

StoreAdrLSB
    banksel RCREG 
    movfw RCREG
    BANK1
    movwf transmitAdrLSB
    incf transmitFuncState, f 
        retfie

StoreCommand
    banksel RCREG 
    movfw RCREG
    BANK1
    movwf  transmitCommandByte
    incf transmitFuncState, f 
        retfie

StoreRepeatNumber
    banksel RCREG 
    movfw RCREG
    BANK1
    movwf  transmitRepeats

    ;bytes framed and stored, begin transfer

    BANK1
    clrf transmitFuncState ;clear now for next command
    BANK0

    BTFSC DataWaitingTransmission
        retfie ;if the flag is set, transmission is not completed, received command will not be sent
    ;flag must be clear

    ;now run function
    BSF DataWaitingTransmission
    clrf TransmitBitNum ; used for sending bits
    clrf TransmitState
    call TransmitLeaderPulse ;init pulse sequence

        retfie

;-----------------------------------------------------------------------------

TransmitModulatedEncoding
;Modulation Output
;set PWM2 to period to 38Khz, 26.3uS
;set PWM2 duty cycle to about 10%, 2uS pulse, 24.3uS idle
;use timer to turn on and off the PWM, 9mS on, 4.5mS off, then bit times
;   ran out of timers, 16-bit timer is required, using a PWM timer as a timing source, PWM is disabled

;a 9ms leading pulse burst (16 times the pulse burst length used for a logical data bit)
;a 4.5ms space
;the 8-bit address for the receiving device
;the 8-bit logical inverse of the address
;the 8-bit command
;the 8-bit logical inverse of the command
;a final 562.5µs pulse burst to signify the end of message transmission.
;takes about 13.5+(32x2.25mS) = 72mS max packet length without repeat packets

banksel PWM1INTF ;used as for the timer, no PWM output
BTFSS PWM1INTF, PRIF ;1 = Period match event occurred
    return  ;bra $-1

BCF PWM1INTF, PRIF ;clear I guess, no auto clearing

    movfw TransmitState
    BRW 
    bra TransmitLeaderPulse
    bra TransmitSpace
    bra TransmitSetBitTime
    bra TransmitAddressLSB
    bra TransmitAddressMSB
    bra TransmitCommand
    bra TransmitCommandInverse
    bra TransmitEndOfMsg ;7
    bra TransmitMessageRepeat ;8
    bra TransmitRepeatLeader
    bra TransmitRepeatSpace
    bra TransmitRepeatEndPulse
    bra TransmitRepeatEnd
    return ;never gets here

;-----------------------------------------------------------------------------

TransmitLeaderPulse
    ;start 9mS leader pulse

    banksel PWMEN
    clrf PWMEN

    ;set for 9mS
    banksel PWM1PRH
    movlw 0x8C
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0xA0; with a 1:8 prescaler, = 250nS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 

    banksel PWMLD
    BSF PWMLD,PWM1LDA_A ;need to set this flag to apply PR changes
    BSF PWMLD,PWM2LDA_A ;need to set this flag to apply PR changes

    banksel PWMEN
    BSF PWMEN, PWM1EN_A ;start 16-bit timer
    BSF PWMEN, PWM2EN_A ;start carrier PWM

    incf TransmitState, f

        return

;-----------------------------------------------------------------------------

TransmitSpace
    ;leave PWM carrier off for 4.5mS

    banksel PWMEN
    clrf PWMEN ;disable both PWMs, otherwise timer does not update properly

;set for 4.5mS = 18000 period
    banksel PWM1PRH
    movlw 0x46
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0x50 ; with a 1:8 prescaler, = 250nS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 
    banksel PWMLD
    BSF PWMLD,PWM1LDA_A;need to set this flag to latch  PR changes

    banksel PWMEN
    BSF PWMEN, PWM1EN_A ;start 16-bit timer

    incf TransmitState, f
        return

;-----------------------------------------------------------------------------
;Starting bit transmit, each byte is sent LSB first
;-----------------------------------------------------------------------------

TransmitSetBitTime

    banksel PWMEN
    clrf PWMEN ;disable both PWMs, otherwise timer does not update properly

;set for 562.5uS= 2250 period, a '0' bit takes 2 periods=1.25mS, a '1' bit takes 4 periods = 2.25mS
    banksel PWM1PRH
    movlw 0x08
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0xCA ; with a 1:8 prescaler, = 250nS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 
    banksel PWMLD
    BSF PWMLD,PWM1LDA_A;need to set this flag to latch  PR changes

    banksel PWMEN
    BSF PWMEN, PWM1EN_A ;start 16-bit timer
    BSF PWMEN, PWM2EN_A ;start carrier PWM - all bits(0 or 1) start with a high pulse

    clrf TransmitBitNum
    clrf TransmitBitTick
    BANK1
    movfw transmitAdrMSB;mov byte to be sent next
    BANK0
    movwf TransmitWork

    incf TransmitState, f
        return

;-----------------------------------------------------------------------------

TransmitAddressLSB

    movfw TransmitBitNum
    xorlw .8
    BTFSS STATUS, Z
    bra HandleTxBit
    ;TransmitBitNum == 8
    ;move on to next byte

    clrf TransmitBitNum
    clrf TransmitBitTick

    BANK1
    movfw transmitAdrLSB;mov byte to be sent next
    BANK0 ;doesn't really need to bank back, but good practice
    movwf TransmitWork

    incf TransmitState, f
 ;   let it go right into next state

;-----------------------------------------------------------------------------

TransmitAddressMSB

    movfw TransmitBitNum
    xorlw .8
    BTFSS STATUS, Z
    bra HandleTxBit
    ;TransmitBitNum == 8
    ;move on to next byte

    clrf TransmitBitNum
    clrf TransmitBitTick

    BANK1
    movfw transmitCommandByte;mov byte to be sent next
    BANK0 ;doesn't really need to bank back, but good practice
    movwf TransmitWork

    incf TransmitState, f
   ;  let it go right into next state

;-----------------------------------------------------------------------------

TransmitCommand

    movfw TransmitBitNum
    xorlw .8
    BTFSS STATUS, Z
    bra HandleTxBit
    ;TransmitBitNum == 8
    ;move on to next byte

    clrf TransmitBitNum
    clrf TransmitBitTick

    BANK1
    movfw transmitCommandByte;mov byte to be sent next
    BANK0 ;doesn't really need to bank back, but good practice
    movwf TransmitWork
    comf TransmitWork, f ;make inverse(0s are 1s, 1s are 0s)

    incf TransmitState, f
  ;let it go right into next state

;-----------------------------------------------------------------------------

TransmitCommandInverse

    movfw TransmitBitNum
    xorlw .8
    BTFSS STATUS, Z
    bra HandleTxBit
    ;TransmitBitNum == 8
    ;move on to next byte

    clrf TransmitBitNum
    clrf TransmitBitTick

    incf TransmitState, f
    ; let it go right into next state

;-----------------------------------------------------------------------------

TransmitEndOfMsg
    ;the pulse burst for end of message is automatically generated from bit functions, it always ends a byte with a high pulse
    banksel PWMEN
    BCF PWMEN, PWM2EN_A ;disable carrier PWM

    clrf TransmitBitNum ;do this here, not needed again til next message
    clrf TransmitBitTick

    incf TransmitState, f ; incase of repeats, otherwise will be cleared

    BANK1
    movfw transmitRepeats
    BTFSS STATUS, Z
        bra TransmitMessageRepeat ;repeat value was greater than 0, start repeats

    ;transmitRepeats was 0, no repeats, end message
    BANK0
    BCF DataWaitingTransmission
    clrf TransmitState ;in case it somehow called the table again, it wouldn't be out of bounds
        return

;-----------------------------------------------------------------------------

TransmitMessageRepeat
    ; start 40mS, space

;setup timers and PWM
    banksel PWMEN
    clrf PWMEN ;disable both PWMs, otherwise timer does not update properly

;set for 40mS. Need to change prescaler to 1:32, 40mS = 40,000 ticks
    movlw b'01010000'
    movwf PWM1CLKCON    ;Set Clock Control: Fosc, 1:32 prescaler

    banksel PWM1PRH
    movlw 0x9C
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0x40 ; with a 1:32 prescaler, = 1uS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 
    banksel PWMLD
    BSF PWMLD,PWM1LDA_A;need to set this flag to latch  PR changes

    banksel PWMEN
    BSF PWMEN, PWM1EN_A ;start 16-bit timer

    incf TransmitState, f
        return

;-----------------------------------------------------------------------------
TransmitRepeatLeader
    ;start 9mS leader pulse

;setup timers and PWM
    banksel PWMEN
    clrf PWMEN ;disable both PWMs, otherwise timer does not update properly

    ;reset CLK prescaler, so can reuse leader pulse function
    movlw b'00110000'
    movwf PWM1CLKCON    ;Set Clock Control: Fosc, 1:8 prescaler

    call TransmitLeaderPulse ;reuse function
        ;transmitstate was incremented in called function
        return
;-----------------------------------------------------------------------------

TransmitRepeatSpace
    ;start 2.25mS space

    banksel PWMEN
    clrf PWMEN ;disable both PWMs, otherwise timer does not update properly

;set for 2.25mS
    banksel PWM1PRH
    movlw 0x23
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0x28          ;with a 1:8 prescaler, = 250nS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 
    banksel PWMLD
    BSF PWMLD,PWM1LDA_A;need to set this flag to latch  PR changes

    banksel PWMEN
    BSF PWMEN, PWM1EN_A ;start 16-bit timer

    incf TransmitState, f
        return

;-----------------------------------------------------------------------------

TransmitRepeatEndPulse
    ;start 562.5uS pulse

    banksel PWMEN
    clrf PWMEN ;disable both PWMs, otherwise timer does not update properly

;set for 562.5uS
    banksel PWM1PRH
    movlw 0x08
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0xCA ; with a 1:8 prescaler, = 250nS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 
    banksel PWMLD
    BSF PWMLD,PWM1LDA_A;need to set this flag to latch  PR changes

    banksel PWMEN
    BSF PWMEN, PWM1EN_A ;start 16-bit timer
    BSF PWMEN, PWM2EN_A ;start carrier PWM - all bits(0 or 1) start with a high pulse

    incf TransmitState, f
        return

;-----------------------------------------------------------------------------
TransmitRepeatEnd
;check repeat value and handle

    BANK1
    decfsz transmitRepeats,f
        bra TransmitContinueRepeats

    ;transmitRepeats was 0, no repeats, end message
    banksel PWMEN
    clrf PWMEN ;disable both PWMs

    ;set clock prescaler back to normal
    movlw b'00110000'
    movwf PWM1CLKCON    ;Set Clock Control: Fosc, 1:8 prescaler

    BANK0
    BCF DataWaitingTransmission
    clrf TransmitState ;in case it somehow called the table again, it wouldn't be out of bounds
        return

;-----------------------------------------------------------------------------

TransmitContinueRepeats
    movlw .8
    movwf TransmitState
    call TransmitContinueRepeatSpace
    return

;-----------------------------------------------------------------------------

TransmitContinueRepeatSpace
    ;start 108mS space

;setup timers and PWM
    banksel PWMEN
    clrf PWMEN ;disable both PWMs, otherwise timer does not update properly

;set for 108mS. Need to change prescaler to 1:64, 108mS = 54,000
    movlw b'01100000'
    movwf PWM1CLKCON    ;Set Clock Control: Fosc, 1:32 prescaler

    banksel PWM1PRH
    movlw 0xD2
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0xF0; with a 1:64 prescaler, = 2uS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 
    banksel PWMLD
    BSF PWMLD,PWM1LDA_A;need to set this flag to latch  PR changes

    banksel PWMEN
    BSF PWMEN, PWM1EN_A ;start 16-bit timer

    incf TransmitState, f
        return

;-----------------------------------------------------------------------------

HandleTxBit

    BTFSS TransmitWork, 0
    bra BitLow ;bit is 0

    bra BitHigh ;bit must be 1

;-----------------------------------------------------------------------------

BitLow ;low bits are 1 period high, 1 period low

    banksel PWMEN
    BCF PWMEN, PWM2EN_A ;disable carrier PWM

    incf TransmitBitTick, f

    movfw TransmitBitTick
    xorlw .2
    BTFSS STATUS, Z
        return ;required ticks not yet elapsed
        bra BitEnd ;ticks have been met, end the bit cycle

;-----------------------------------------------------------------------------

BitHigh ;hi bits are 1 period high, 3 periods low

    banksel PWMEN
    BCF PWMEN, PWM2EN_A ;disable carrier PWM

    incf TransmitBitTick, f

    movfw TransmitBitTick
    xorlw .4
    BTFSS STATUS, Z
        return ;required ticks not yet elapsed
        bra BitEnd ;ticks have been met, end the bit cycle

;-----------------------------------------------------------------------------

BitEnd

    incf TransmitBitNum,f
    LSRF TransmitWork, f ;shift to next bit

    clrf TransmitBitTick

    banksel PWMEN
    BSF PWMEN, PWM2EN_A ;enable carrier PWM
        return

;-----------------------------------------------------------------------------


