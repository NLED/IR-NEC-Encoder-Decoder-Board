;*******************************************************************************
; Configuration Defines
;*******************************************************************************

        #ifdef __12F1572
        LIST      P=PIC12F1572      ; list directive to define processor
        #INCLUDE <p12f1572.INC>         ; processor specific variable definitions
        ; CONFIG1
        ; __config 0xFF9C
         __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_ON & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _BOREN_ON & _CLKOUTEN_OFF
        ; CONFIG2
        ; __config 0xFFFF
         __CONFIG _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_OFF & _BORV_LO & _LPBOREN_OFF & _LVP_OFF
        #endif

        #ifdef __12F1822
        LIST      P=PIC12F1822      ; list directive to define processor
        #INCLUDE <p12f1822.INC>         ; processor specific variable definitions
        ; CONFIG1
        ; __config 0x3F9C
         __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_ON
        ; CONFIG2
        ; __config 0x3FFF
         __CONFIG _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_OFF & _BORV_LO & _LVP_OFF
        #endif

        #ifdef __12F1840
        LIST      P=PIC12F1840      ; list directive to define processor
        #INCLUDE <p12f1840.INC>         ; processor specific variable definitions
        ; CONFIG1
        ; __config 0x3F9C
         __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_ON
        ; CONFIG2
        ; __config 0x3FFF
         __CONFIG _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_OFF & _BORV_LO & _LVP_OFF
        #endif

;*******************************************************************************
;Surpress some warnings
;*******************************************************************************

    errorlevel -203 ;warns of formating column position
    errorlevel -205 ;like 203
    errorlevel -302 ;warns of register not in BANK0

;*******************************************************************************
;Start RAM Defines
;*******************************************************************************
	
CBLOCK 0x20	;96(80) bytes bank0

;reserved for bit building function

	ENDC

CBLOCK 0xA0	;32 bytes - Bank 1
    transmitAdrMSB
    transmitAdrLSB
    transmitCommandByte
    transmitRepeats

    transmitHoldRCREG
    transmitFuncState

	ENDC

CBLOCK	0x70	;16 bytes, accessed from all Banks			
    Flags
    PulseCountLSB
    PulseCountMSB
    ReceiveState

    CounterMain
    Delay
    Delay2
    Scratch

    TransmitState
    TransmitBitNum
    TransmitWork
    TransmitBitTick

    ConvertedByte1
    ConvertedByte2
    ConvertedByte3
    ConvertedByte4

	ENDC

;*******************************************************************************
;Start Variable Flags
;*******************************************************************************

   #define RepeatFlag Flags, 0
   #define ConvertToBytesFlag Flags, 1
   #define ModulationConvertFlag Flags, 2 ;0 = modulated, 1 = demodulated
   #define DataWaitingTransmission Flags, 3 ;only used for transmission

;*******************************************************************************
;Start Pin Defines
;*******************************************************************************

    #define portJumperInput PORTA
   #define pJumperInput  PORTA, RA5
    #define portHeaderInput PORTA
   #define pHeaderInput  PORTA, RA3 ;MCLR/CONFIG pin

;*******************************************************************************
;Start Macros
;*******************************************************************************

;MACROS
    #define		BANK0		movlb .0 ;should be bank 0
    #define		BANK1		movlb .1 ;should be bank 1
    #define		BANK2		movlb .2 ;should be bank 1