;*******************************************************************************
;File is placed at in main.asm using #include
;All currently supported processors use the same initialization code
;*******************************************************************************

Initialize
    ;START HARDWARE INIT    
    BANKSEL OSCCON          ; Switch to Bank 1.
    MOVLW   b'11110000'     ; PLL On, 8Mhz, 
    MOVWF   OSCCON

    banksel ADCON0
    clrf ADCON0 ;disable AdC
    clrf ADCON1   

    BANKSEL ANSELA           ; Switch to Bank 1.
    CLRF    ANSELA           ; Set I/O pins to Digital   

    banksel LATA
    clrf LATA

    banksel TRISA
    movlw b'00101110' ;RA1,RA2,RA3,RA5,
    movwf TRISA

    ;setup Internal Pull Ups
    banksel OPTION_REG  
    clrf OPTION_REG

    banksel WPUA
    movlw b'00101100' ;RA2,RA3,RA5 pull up enabled, all others disabled
    movwf WPUA

    ;===================== Setup Interrupt Pin ============================

    ;When the INTEDG bit is set, the rising edge will cause the interrupt. When
    ;   the INTEDG bit is clear, the falling edge will cause the interrupt.
    banksel OPTION_REG
    BCF OPTION_REG, INTEDG ;set to falling edge

    banksel INTCON
    BSF INTCON, INTE ;set enable bit

    ;======================= setup TMR0 ============================

    banksel OPTION_REG
    BSF OPTION_REG, PSA
    ;1:1 scaler
    ;PS<2:0>: Prescaler Rate Select bits
    ;BSF OPTION_REG, 0
    ;BSF OPTION_REG, 1
    ;BSF OPTION_REG, 2

    ;===================== setup TMR1 ============================

    banksel T1CON
    movlw b'00110100' ;Timer1 clock source is the instruction clock (FOSC/4), 11 = 1:8 Prescale value
        ;at 1:8, TMR1 ticks are 1uS, max length of 65.535mS
    movwf T1CON ;TMR OFF
    BSF T1CON, TMR1ON
    clrf T1GCON ;clear

    banksel PIE1 ;enable reset timer interrupt
    BSF PIE1, TMR1IE

    ;===================== setup TMR2 ============================

    banksel PIR1
    BCF PIR1, TMR2IF
    banksel PIE1
    BCF PIE1, TMR2IE ;disable interrupt

    banksel TMR2
    clrf TMR2		
    banksel T2CON
    movlw  b'00111011';TMR2 on, Post 1:8 - Pre 1:64 
    ;movlw  b'00000100';
    movwf T2CON	
    BSF T2CON, TMR2ON
    ;70.3 count for 4.5mS, 140.6 for 9mS count

    banksel PR2
    movlw .255
    movwf PR2

;===================== setup UART ===========================================

;set to alternate pins - already clear but whatever
    banksel APFCON
    BCF APFCON, RXDTSEL ;1 = RX/DT function is on RA5, 0 = RA1
    BCF APFCON, TXCKSEL ;1 = TX/CK function is on RA4, 0 = RA0
    BSF APFCON, P2SEL ;1 = PWM2 function is on RA4


    banksel SPBRG
    ;movlw .207 ;set for 9600
    ;movlw .103 ;set for 19200
    ;movlw .51 ;set for 38400
    ;movlw .34 ;set for 57600
    ;movlw .16 ;set for 115200 ;2.12% error, a bit much, avoid if possible
    movlw .7 ;set for 250k
    movwf SPBRG

    banksel SPBRGH	
    clrf SPBRGH

    banksel BAUDCON
    BCF BAUDCON, BRG16

    banksel TXSTA
    clrf TXSTA			; Clear TXSTA register
    BSF TXSTA, BRGH             ;set BRGH
    BSF TXSTA,TXEN              ;enable transmission

    banksel RCSTA
    movlw 	B'10010000' ;SPEN & CREN enabled
    movwf	RCSTA	

    banksel RCREG
    movfw RCREG

    #ifdef cEnableNECTransmission
    banksel PIE1
    BSF PIE1, RCIE ;enable Reception interrupt
    #endif

;============ Init PWM outputs - All PWM registers in BANK 31 ==================

    #ifdef cEnableNECTransmission ;will work only on 12F1572, maybe others but not 12F1840/12F1822
    ; would have to write newPWM init code for other processors

    ;INIT PWM2  RA4
    banksel PWM2PHL
    clrf PWM2PHL ;clear PWM Phase(LOW) 
    clrf PWM2PHH ;clear PWM Phase(HI) 	

    clrf PWM2DCL ;clear PWM Duty Cycle(LOW) 
    clrf PWM2DCH ;clear PWM Duty Cycle(HI)   

    movlw 0xD2 ; with a 1:4 prescaler, this creates a 38Khz Period
    movwf PWM2PRL ;Set PWM Period Register(LOW) 
    clrf PWM2PRH	;Set PWM Period Register(HI) 

    ;clrf PWM2CLKCON    ;Set Clock Control: Fosc, No prescaler	
    movlw b'00100000'
    movwf PWM2CLKCON    ;Set Clock Control: Fosc, 1:4 prescaler = TCY = FOSC/4

    movlw b'01000000' ;module not activated, EN
    movwf PWM2CON    ;Set PWM Control: Output Enabled, Standard Mode, PWM output active state is high

    movlw b'10000000' ;Loads buffers on trigger disabled, Armed/Set to Trigger	
    movwf PWM2LDCON

    clrf PWM2OFCON ; independent Run mode


clrf PWM2DCH ;DUTY CYCLE COUNT
movlw .21 ;sets high time, to 10% for carrier generation. What should it be? 50/50?
movwf PWM2DCL


;TEST METHOD - Enable PWM1 on RA5, and use it's timer with interrupt

    banksel PWM1PHL
    clrf PWM1PHL ;clear PWM Phase(LOW) 
    clrf PWM1PHH ;clear PWM Phase(HI) 	

    clrf PWM1DCL ;clear PWM Duty Cycle(LOW) 
    clrf PWM1DCH ;clear PWM Duty Cycle(HI)   

    ;clrf PWM2CLKCON    ;Set Clock Control: Fosc, No prescaler	
    movlw b'00110000'
    movwf PWM1CLKCON    ;Set Clock Control: Fosc, 1:8 prescaler

    movlw b'00000000' ;module not activated, pin disabled
    movwf PWM1CON    ;Set PWM Control: Output Enabled, Standard Mode, PWM output active state is high

    movlw b'00000000' ;Loads buffers on trigger disabled, Armed/Set to Trigger	
    movwf PWM1LDCON

    ;Set timer value for whatever
    movlw 0xFF
    movwf PWM1PRH	;Set PWM Period Register(HI) 
    movlw 0xFF; with a 1:8 prescaler, = 250nS per tick
    movwf PWM1PRL ;Set PWM Period Register(LOW) 

;doesn't matter for this usage
clrf PWM1DCH ;DUTY CYCLE COUNT
movlw 0xFF
movwf PWM1DCL


banksel PWMLD
BSF PWMLD,PWM1LDA_A
BSF PWMLD,PWM2LDA_A

    ;PWM Common
    banksel PWMEN
     clrf PWMEN
    ;  both the carrier PWM generator and the PWM timer used for delay - both disabled

    #endif

;============ End Hardware Intializations ==================

    movlw .5    ;set timer value
    movwf CounterMain

    ;long power up timer for things to stablize
    decfsz Delay, f
    bra $-1
    decfsz Delay2, f
    bra $-3
    decfsz CounterMain, f ;used as scratch here
    bra $-5

    clrf Flags ;init Flags byte cleared

    ;select baud rate using jumper input, Open = Slow baud, Closed = Fast baud
    movlw cSlowBaudRate ;load slow baud rate SPBRG value into working

    banksel portJumperInput
    BTFSS pJumperInput ;skip next instruction if jumper is open/pulled high
    movlw cFastBaudRate ;load fast baud rate SPBRG value into working IF jumper is closed

    ;WREG holds the selected baud rate
    banksel SPBRG
    movwf SPBRG ;now load into baud rate generator register
    ;end baud rate selection


    ;set hardware RX usage, either expect a modulated signal or demodualted signal
    banksel portHeaderInput ;PORTA
    BTFSS pHeaderInput ;skip next instruction if jumper is open/pulled high
    bra $+3
    BSF ModulationConvertFlag ;0 = modulated, 1 = demodulated
    bra $+2
    BCF ModulationConvertFlag ;0 = modulated, 1 = demodulated


       GOTO START ; goto program start



