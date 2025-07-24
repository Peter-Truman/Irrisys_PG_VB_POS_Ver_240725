;   /\\\\\\\\\
;  /\\\///////\\\
;  \/\\\     \/\\\                                                 /\\\          /\\\
;   \/\\\\\\\\\\\/        /\\\\\     /\\\\\\\\\\     /\\\\\\\\   /\\\\\\\\\\\  /\\\\\\\\\\\  /\\\\\\\\\
;    \/\\\//////\\\      /\\\///\\\  \/\\\//////    /\\\/////\\\ \////\\\////  \////\\\////  \////////\\\
;     \/\\\    \//\\\    /\\\  \//\\\ \/\\\\\\\\\\  /\\\\\\\\\\\     \/\\\         \/\\\        /\\\\\\\\\\
;      \/\\\     \//\\\  \//\\\  /\\\  \////////\\\ \//\\///////      \/\\\ /\\     \/\\\ /\\   /\\\/////\\\
;       \/\\\      \//\\\  \///\\\\\/    /\\\\\\\\\\  \//\\\\\\\\\\    \//\\\\\      \//\\\\\   \//\\\\\\\\/\\
;        \///        \///     \/////     \//////////    \//////////      \/////        \/////     \////////\//
;                                  Let's find out together what makes a PIC Tick!
;
; Code Produced by the Positron8 Compiler. Version 4.0.5.1
; Created and Written by Les Johnson. 
; Compiler version for Peter Truman
;----------------------------------------------------------
;
#define config_req 1
 LIST  P = 18F2525, F = INHX32, W = 2, X = ON, R = DEC, MM = ON, N = 0, C = 255, T = ON
; MICROCONTROLLER'S SFRS
; START OF ACCESS SFRS
PORTA equ 0X0F80
PORTB equ 0X0F81
PORTC equ 0X0F82
PORTE equ 0X0F84
LATA equ 0X0F89
LATB equ 0X0F8A
LATC equ 0X0F8B
DDRA equ 0X0F92
TRISA equ 0X0F92
DDRB equ 0X0F93
TRISB equ 0X0F93
DDRC equ 0X0F94
TRISC equ 0X0F94
OSCTUNE equ 0X0F9B
PIE1 equ 0X0F9D
PIR1 equ 0X0F9E
IPR1 equ 0X0F9F
PIE2 equ 0X0FA0
PIR2 equ 0X0FA1
IPR2 equ 0X0FA2
EECON1 equ 0X0FA6
EECON2 equ 0X0FA7
EEDATL equ 0X0FA8
EEDATA equ 0X0FA8
EEADR equ 0X0FA9
EEADRH equ 0X0FAA
RCSTA equ 0X0FAB
TXSTA equ 0X0FAC
TXREG equ 0X0FAD
RCREG equ 0X0FAE
SPBRG equ 0X0FAF
SPBRGH equ 0X0FB0
T3CON equ 0X0FB1
TMR3L equ 0X0FB2
TMR3LH equ 0X0FB3
TMR3H equ 0X0FB3
CMCON equ 0X0FB4
CVRCON equ 0X0FB5
BAUDCON equ 0X0FB8
BAUDCTL equ 0X0FB8
CCP2CON equ 0X0FBA
CCPR2 equ 0X0FBB
CCPR2L equ 0X0FBB
CCPR2LH equ 0X0FBC
CCPR2H equ 0X0FBC
CCP1CON equ 0X0FBD
CCPR1 equ 0X0FBE
CCPR1L equ 0X0FBE
CCPR1LH equ 0X0FBF
CCPR1H equ 0X0FBF
ADCON2 equ 0X0FC0
ADCON1 equ 0X0FC1
ADCON0 equ 0X0FC2
ADRES equ 0X0FC3
ADRESL equ 0X0FC3
ADRESLH equ 0X0FC4
ADRESH equ 0X0FC4
SSPCON2 equ 0X0FC5
SSPCON1 equ 0X0FC6
SSPSTAT equ 0X0FC7
SSPADD equ 0X0FC8
SSPBUF equ 0X0FC9
T2CON equ 0X0FCA
PR2 equ 0X0FCB
TMR2 equ 0X0FCC
T1CON equ 0X0FCD
TMR1L equ 0X0FCE
TMR1LH equ 0X0FCF
TMR1H equ 0X0FCF
RCON equ 0X0FD0
WDTCON equ 0X0FD1
HLVDCON equ 0X0FD2
LVDCON equ 0X0FD2
OSCCON equ 0X0FD3
DEBUG equ 0X0FD4
T0CON equ 0X0FD5
TMR0L equ 0X0FD6
TMR0LH equ 0X0FD7
TMR0H equ 0X0FD7
STATUS equ 0X0FD8
FSR2L equ 0X0FD9
FSR2LH equ 0X0FDA
FSR2H equ 0X0FDA
PLUSW2 equ 0X0FDB
PREINC2 equ 0X0FDC
POSTDEC2 equ 0X0FDD
POSTINC2 equ 0X0FDE
INDF2 equ 0X0FDF
BSR equ 0X0FE0
FSR1L equ 0X0FE1
FSR1LH equ 0X0FE2
FSR1H equ 0X0FE2
PLUSW1 equ 0X0FE3
PREINC1 equ 0X0FE4
POSTDEC1 equ 0X0FE5
POSTINC1 equ 0X0FE6
INDF1 equ 0X0FE7
WREG equ 0X0FE8
FSR0L equ 0X0FE9
FSR0LH equ 0X0FEA
FSR0H equ 0X0FEA
PLUSW0 equ 0X0FEB
PREINC0 equ 0X0FEC
POSTDEC0 equ 0X0FED
POSTINC0 equ 0X0FEE
INDF0 equ 0X0FEF
INTCON3 equ 0X0FF0
INTCON2 equ 0X0FF1
INTCON equ 0X0FF2
PRODL equ 0X0FF3
PRODLH equ 0X0FF4
PRODH equ 0X0FF4
TABLAT equ 0X0FF5
TBLPTRL equ 0X0FF6
TBLPTRLH equ 0X0FF7
TBLPTRH equ 0X0FF7
TBLPTRU equ 0X0FF8
TBLPTRLHH equ 0X0FF8
PC equ 0X0FF9
PCL equ 0X0FF9
PCLATH equ 0X0FFA
PCLATU equ 0X0FFB
STKPTR equ 0X0FFC
TOS equ 0X0FFD
TOSL equ 0X0FFD
TOSLH equ 0X0FFE
TOSH equ 0X0FFE
TOSU equ 0X0FFF
; I2C PINS USED BY HBUSIN AND HBUSOUT
_I2C_SCL_port=TRISC
_I2C_SCL_pin=3
_I2C_SDA_port=TRISC
_I2C_SDA_pin=4
; SFR BITS USED INTERNALLY BY THE COMPILER
C=0
DC=1
Z=2
OV=3
N=4
PD=5
To=6
PP_TXIF=4
PP_RCIF=5
PP_RD=0
PP_WR=1
PP_WREN=2
PP_WRERR=3
PP_EEPGD=7
PP_OERR=1
PP_CREN=4
PP_BRGH=2
PP_SENDB=3
PP_T3CCP1=3
PP_T3CCP2=6
PP_RD16=7
PP_BRG16=3
PP_ADON=0
PP_GO_DONE=1
PP_SEN=0
PP_RSEN=1
PP_PEN=2
PP_RCEN=3
PP_ACKEN=4
PP_ACKDT=5
PP_R_W=2
PP_T2CKPS0=0
PP_T2CKPS1=1
PP_TMR2ON=2
PP_TMR1ON=0
PP_RBPU=7
; MEMORY MAP OF THE DEVICE
  __MAXRAM  0X0FFF
  __BADRAM  0X0F83
  __BADRAM  0X0F85-0X0F88
  __BADRAM  0X0F8C-0X0F91
  __BADRAM  0X0F95-0X0F9A
  __BADRAM  0X0F9C
  __BADRAM  0X0FA3-0X0FA5
  __BADRAM  0X0FB6-0X0FB7
  __BADRAM  0X0FB9
; CONFIG FUSE NAME VALUES
config1h equ 0X300001
config2l equ 0X300002
config2h equ 0X300003
config3h equ 0X300005
config4l equ 0X300006
config5l equ 0X300008
config5h equ 0X300009
config6l equ 0X30000A
config6h equ 0X30000B
config7l equ 0X30000C
config7h equ 0X30000D
OSC_LP_1 equ 0xF0
OSC_XT_1 equ 0xF1
OSC_HS_1 equ 0xF2
OSC_RC_1 equ 0xF3
OSC_EC_1 equ 0xF4
OSC_ECIO6_1 equ 0xF5
OSC_HSPLL_1 equ 0xF6
OSC_RCIO6_1 equ 0xF7
OSC_INTIO67_1 equ 0xF8
OSC_INTIO7_1 equ 0xF9
FCMEN_OFF_1 equ 0xBF
FCMEN_ON_1 equ 0xFF
IESO_OFF_1 equ 0x7F
IESO_ON_1 equ 0xFF
PWRT_ON_2 equ 0xFE
PWRT_OFF_2 equ 0xFF
BOREN_OFF_2 equ 0xF9
BOREN_ON_2 equ 0xFB
BOREN_NOSLP_2 equ 0xFD
BOREN_SBORDIS_2 equ 0xFF
BORV_0_2 equ 0xE7
BORV_1_2 equ 0xEF
BORV_2_2 equ 0xF7
BORV_3_2 equ 0xFF
WDT_OFF_2 equ 0xFE
WDT_ON_2 equ 0xFF
WDTPS_1_2 equ 0xE1
WDTPS_2_2 equ 0xE3
WDTPS_4_2 equ 0xE5
WDTPS_8_2 equ 0xE7
WDTPS_16_2 equ 0xE9
WDTPS_32_2 equ 0xEB
WDTPS_64_2 equ 0xED
WDTPS_128_2 equ 0xEF
WDTPS_256_2 equ 0xF1
WDTPS_512_2 equ 0xF3
WDTPS_1024_2 equ 0xF5
WDTPS_2048_2 equ 0xF7
WDTPS_4096_2 equ 0xF9
WDTPS_8192_2 equ 0xFB
WDTPS_16384_2 equ 0xFD
WDTPS_32768_2 equ 0xFF
MCLRE_OFF_3 equ 0x7F
MCLRE_ON_3 equ 0xFF
LPT1OSC_OFF_3 equ 0xFB
LPT1OSC_ON_3 equ 0xFF
PBADEN_OFF_3 equ 0xFD
PBADEN_ON_3 equ 0xFF
CCP2MX_PORTBE_3 equ 0xFE
CCP2MX_PORTC_3 equ 0xFF
STVREN_OFF_4 equ 0xFE
STVREN_ON_4 equ 0xFF
LVP_OFF_4 equ 0xFB
LVP_ON_4 equ 0xFF
XINST_OFF_4 equ 0xBF
XINST_ON_4 equ 0xFF
DEBUG_ON_4 equ 0x7F
DEBUG_OFF_4 equ 0xFF
CP0_ON_5 equ 0xFE
CP0_OFF_5 equ 0xFF
CP1_ON_5 equ 0xFD
CP1_OFF_5 equ 0xFF
CP2_ON_5 equ 0xFB
CP2_OFF_5 equ 0xFF
CPB_ON_5 equ 0xBF
CPB_OFF_5 equ 0xFF
CPD_ON_5 equ 0x7F
CPD_OFF_5 equ 0xFF
WRT0_ON_6 equ 0xFE
WRT0_OFF_6 equ 0xFF
WRT1_ON_6 equ 0xFD
WRT1_OFF_6 equ 0xFF
WRT2_ON_6 equ 0xFB
WRT2_OFF_6 equ 0xFF
WRTB_ON_6 equ 0xBF
WRTB_OFF_6 equ 0xFF
WRTC_ON_6 equ 0xDF
WRTC_OFF_6 equ 0xFF
WRTD_ON_6 equ 0x7F
WRTD_OFF_6 equ 0xFF
EBTR0_ON_7 equ 0xFE
EBTR0_OFF_7 equ 0xFF
EBTR1_ON_7 equ 0xFD
EBTR1_OFF_7 equ 0xFF
EBTR2_ON_7 equ 0xFB
EBTR2_OFF_7 equ 0xFF
EBTRB_ON_7 equ 0xBF
EBTRB_OFF_7 equ 0xFF
DEVID1 equ 0X3FFFFE
DEVID2 equ 0X3FFFFF
; COMPILER'S INTERNAL CONSTANTS AND ALIASES
#define __18F2525 1
#define xtal 32
#define _core 16
#define _MaxRAM 3967
#define _RAM_End 0X0F7F
#define _MaxMem 0XC000
#define _ADC 10
#define _ADC_res 10
#define _eeprom 1024
#define ram_banks 15
#define _USART 1
#define _USB 0
#define _flash 1
#define _cwrite_block 64
#define _TRIS_offset 18
#define __EE_RW_type 1
#define __MSSP_type 1
#define __HPWM_type 1
#define __adin_type 1
#define __UART_type 1
#define BankA_Start 0x00
#define BankA_End 0x7F
#define clrw clrf WREG
#define negw negf WREG
#define skpc btfss STATUS,0
#define skpnc btfsc STATUS,0
#define clrc bcf STATUS,0
#define setc bsf STATUS,0
#define skpz btfss STATUS,2
#define skpnz btfsc STATUS,2
#define clrz bcf STATUS,2
#define setz bsf STATUS,2
; COMPILER SYSTEM VARIABLES
BPF equ 0x00
BPFH equ 0x01
PP0 equ 0x02
PP0H equ 0x03
PP1 equ 0x04
PP1H equ 0x05
PP3 equ 0x06
PP3H equ 0x07
; STANDARD VARIABLES
LCD_WriteCharCh equ 0x08
LCD_SetCursorB_Line equ 0x09
LCD_SetCursorPos equ 0x0A
LCD_SetCursorAddr equ 0x0B
LCD_WriteNibbleNibble equ 0x0C
LCD_CommandCmd equ 0x0D
; ALIAS VARIABLES
#define LCD_RS PORTA,6
#define LCD_RW PORTA,5
#define LCD_EN PORTA,7
#define LCD_D4 PORTA,0
#define LCD_D5 PORTA,1
#define LCD_D6 PORTA,2
#define LCD_D7 PORTA,3
#define ENC_A PORTB,1
#define ENC_B PORTB,2
#define ENC_SW PORTB,6
#define RTC_INT PORTB,0
#define SDA PORTC,4
#define SCL PORTC,3
; CONSTANTS
#define __xtal 32
#define LCD_WIDTH 20
#define LCD_Lines 4
#define LONG_PRESS 2000
#define LONG_PRESSH 7
#define I2C_ADDR_DS3231 208
;---------------------------------------------
; START OF THE COMPILER'S LIBRARY ROUTINES
_compiler__start_
    org 0x00
    nop
    nop
    goto _compiler_main_start_
    org 0x08
__lcd_cls_
    movlw 128
    movwf BPFH
__cls_
    movlw 254
    rcall __byte_send__
    movlw 1
    rcall __byte_send__
    movlw 117
    movwf PP0H
    movlw 48
    bra __delay_us_wreg_
__print_
    movwf PP3H
    bcf LATB,2
    bcf LATB,3
    bcf TRISB,2
    bcf TRISB,3
    movlw 15
    andwf TRISB,F
    movf PP3H,W
    btfsc BPF,1
    bra __prt_1__
    movlw 58
    movwf PP0H
    movlw 152
    rcall __delay_us_wreg_
    movlw 51
    movwf PP3
    rcall __print_loop__
    movlw 19
    movwf PP0H
    movlw 136
    rcall __delay_us_wreg_
    rcall __print_loop__
    movlw 100
    rcall __delay_us_
    rcall __print_loop__
    movlw 100
    rcall __delay_us_
    movlw 34
    movwf PP3
    rcall __print_loop__
    movlw 0x28
    rcall __print_sendcommand
    movlw 12
    rcall __print_sendcommand
    movlw 6
    rcall __print_sendcommand
    bsf BPF,1
    movf PP3H,W
    bra __prt_1__
__print_sendcommand
    bsf BPF,0
__prt_1__
    movwf PP3
    btfss BPF,0
    bra __print_combyte__
    bcf LATB,3
    sublw 3
    bnc __print_shortdelay__
    rcall __print_shortdelay__
    movlw 7
    movwf PP0H
    movlw 208
    rcall __delay_us_wreg_
    return
__print_combyte__
    bsf BPF,0
    sublw 254
    bz __print_exit__
    bsf LATB,3
__print_shortdelay__
    btfss BPF,0
__print_loop__
    bcf BPF,0
    bsf LATB,2
    movlw 15
    andwf PORTB,F
    movf PP3,W
    andlw 240
    iorwf PORTB,F
    nop
    bra $ + 2
    bcf LATB,2
    swapf PP3,F
    nop
    bra $ + 2
    btfsc BPF,0
    bra __print_loop__
    movlw 50
    rcall __delay_us_
__print_exit__
    movf PP3H,W
    return
__byte_send__
    btfsc BPFH,7
    bra __print_
__delay_ms_
    clrf PP1H
__delay_ms_wreg_
    movwf PP1
__delayms_from_regs__
    movlw 255
    addwf PP1,F
    addwfc PP1H,F
    bra $ + 2
    btfss STATUS,0
    return
    movlw 3
    movwf PP0H
    movlw 230
    rcall __delay_us_wreg_
    bra __delayms_from_regs__
__delay_us_
    clrf PP0H
__delay_us_wreg_
    addlw 253
    movwf PP0
    bra $ + 4
    decf PP0,F
__delay_us_high_count__
    nop
    bra $ + 2
    clrf WREG
    subwfb PP0H,F
    bc $ - 10
    return
_compiler_main_start_
    clrf BPF
    movlb 0
;---------------------------------------------
; START OF THE USER'S PROGRAM CODE
F1_SOF equ $ ; IRRISYS_PG_VB_POS_240725.BAS
    movlw 15
    movwf ADCON1,0
    movlw 7
    movwf CMCON,0
F1_000050 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] OSCCON.6=1
    bsf OSCCON,6,0
F1_000051 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] OSCCON.5=1
    bsf OSCCON,5,0
F1_000052 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] OSCCON.4=1
    bsf OSCCON,4,0
F1_000053 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] OSCTUNE.6=1
    bsf OSCTUNE,6,0
Main
F1_000163 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] ADCON1 = $0F
    movlw 15
    movwf ADCON1,0
F1_000164 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] TRISA = %00000000
    clrf TRISA,0
F1_000165 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] TRISB = %11000111
    movlw 199
    movwf TRISB,0
F1_000166 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] TRISC = %00011000
    movlw 24
    movwf TRISC,0
F1_000167 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] OSCCON = %01110010
    movlw 114
    movwf OSCCON,0
F1_000169 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_Init()
    rcall LCD_Init
F1_000170 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_Clear()
    rcall LCD_Clear
F1_000171 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_SetCursor(1, 0)
    movlw 1
    movwf LCD_SetCursorB_Line,0
    clrf LCD_SetCursorPos,0
    rcall LCD_SetCursor
F1_000172 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("H")
    movlw 72
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000173 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("E")
    movlw 101
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000174 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("L")
    movlw 108
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000175 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("L")
    movlw 108
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000176 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("O")
    movlw 111
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000177 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar(" ")
    movlw 32
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000178 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("W")
    movlw 87
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000179 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("O")
    movlw 111
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000180 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("R")
    movlw 114
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000181 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("L")
    movlw 108
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000182 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteChar("D")
    movlw 100
    movwf LCD_WriteCharCh,0
    rcall LCD_WriteChar
F1_000184 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] While 1 = 1
_lbl__2
F1_000185 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayMs 1000
    movlw 3
    movwf PP1H,0
    movlw 232
    rcall __delay_ms_wreg_
F1_000186 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Wend
    bra _lbl__2
_lbl__3
F1_000187 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] end
_pblb__4
    bra _pblb__4
F1_000002 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Stop
_pblb__5
    bra _pblb__5
;---------------------------------------------
F1_000085 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Proc LCD_Init()
LCD_Init
F1_000086 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_RS
    bcf TRISA,6,0
    bcf LATA,6,0
F1_000087 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_RW
    bcf TRISA,5,0
    bcf LATA,5,0
F1_000088 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_EN
    bcf TRISA,7,0
    bcf LATA,7,0
F1_000089 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_D4
    bcf TRISA,0,0
    bcf LATA,0,0
F1_000090 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_D5
    bcf TRISA,1,0
    bcf LATA,1,0
F1_000091 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_D6
    bcf TRISA,2,0
    bcf LATA,2,0
F1_000092 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_D7
    bcf TRISA,3,0
    bcf LATA,3,0
F1_000093 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayMs 50
    movlw 50
    rcall __delay_ms_
F1_000094 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble($03)
    movlw 3
    movwf LCD_WriteNibbleNibble,0
    rcall LCD_WriteNibble
F1_000095 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayMs 5
    movlw 5
    rcall __delay_ms_
F1_000096 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble($03)
    movlw 3
    movwf LCD_WriteNibbleNibble,0
    rcall LCD_WriteNibble
F1_000097 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayMs 5
    movlw 5
    rcall __delay_ms_
F1_000098 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble($03)
    movlw 3
    movwf LCD_WriteNibbleNibble,0
    rcall LCD_WriteNibble
F1_000099 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayMs 5
    movlw 5
    rcall __delay_ms_
F1_000100 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble($02)
    movlw 2
    movwf LCD_WriteNibbleNibble,0
    rcall LCD_WriteNibble
F1_000101 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_Command($28)
    movlw 40
    movwf LCD_CommandCmd,0
    rcall LCD_Command
F1_000102 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_Command($0C)
    movlw 12
    movwf LCD_CommandCmd,0
    rcall LCD_Command
F1_000103 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_Command($06)
    movlw 6
    movwf LCD_CommandCmd,0
    rcall LCD_Command
F1_000104 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_Command($01)
    movlw 1
    movwf LCD_CommandCmd,0
    rcall LCD_Command
F1_000105 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayMs 2
    movlw 2
    bra __delay_ms_
;---------------------------------------------
F1_000129 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Proc LCD_WriteChar(Ch As byte)
LCD_WriteChar
F1_000130 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] high LCD_RS
    bcf TRISA,6,0
    bsf LATA,6,0
F1_000131 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble(Ch >> 4)
    swapf LCD_WriteCharCh,W,0
    andlw 15
    movwf LCD_WriteNibbleNibble,0
    rcall LCD_WriteNibble
F1_000132 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble(Ch)
    movff LCD_WriteCharCh,LCD_WriteNibbleNibble
    rcall LCD_WriteNibble
F1_000133 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayUs 50
    movlw 50
    bra __delay_us_
;---------------------------------------------
F1_000137 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Proc LCD_SetCursor(B_Line As byte, Pos As byte)
LCD_SetCursor
F1_000139 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Select B_Line
F1_000140 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Case 1
    decfsz LCD_SetCursorB_Line,W,0
    bra _lbl__7
F1_000141 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Addr = $80 + Pos
    movlw 128
    addwf LCD_SetCursorPos,W,0
    movwf LCD_SetCursorAddr,0
    bra _lbl__6
_lbl__7
F1_000142 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Case 2
    movlw 2
    cpfseq LCD_SetCursorB_Line,0
    bra _lbl__10
F1_000143 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Addr = $C0 + Pos
    movlw 192
    addwf LCD_SetCursorPos,W,0
    movwf LCD_SetCursorAddr,0
    bra _lbl__6
_lbl__10
F1_000144 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Case 3
    movlw 3
    cpfseq LCD_SetCursorB_Line,0
    bra _lbl__12
F1_000145 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Addr = $94 + Pos
    movlw 148
    addwf LCD_SetCursorPos,W,0
    movwf LCD_SetCursorAddr,0
    bra _lbl__6
_lbl__12
F1_000146 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Case 4
    movlw 4
    cpfseq LCD_SetCursorB_Line,0
    bra _lbl__14
F1_000147 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Addr = $D4 + Pos
    movlw 212
    addwf LCD_SetCursorPos,W,0
    movwf LCD_SetCursorAddr,0
F1_000149 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Addr = $80
    movlw 128
    movwf LCD_SetCursorAddr,0
F1_000150 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] EndSelect
_lbl__14
_lbl__6
F1_000151 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_Command(Addr)
    movff LCD_SetCursorAddr,LCD_CommandCmd
    bra LCD_Command
;---------------------------------------------
F1_000155 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Proc LCD_Clear()
LCD_Clear
F1_000156 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Cls
    rcall __lcd_cls_
F1_000158 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayMs 2
    movlw 2
    bra __delay_ms_
;---------------------------------------------
F1_000109 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Proc LCD_WriteNibble(Nibble As byte)
LCD_WriteNibble
F1_000110 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_D4 = Nibble.0
    btfsc LCD_WriteNibbleNibble,0,0
    bsf PORTA,0,0
    btfss LCD_WriteNibbleNibble,0,0
    bcf PORTA,0,0
F1_000111 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_D5 = Nibble.1
    btfsc LCD_WriteNibbleNibble,1,0
    bsf PORTA,1,0
    btfss LCD_WriteNibbleNibble,1,0
    bcf PORTA,1,0
F1_000112 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_D6 = Nibble.2
    btfsc LCD_WriteNibbleNibble,2,0
    bsf PORTA,2,0
    btfss LCD_WriteNibbleNibble,2,0
    bcf PORTA,2,0
F1_000113 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_D7 = Nibble.3
    btfsc LCD_WriteNibbleNibble,3,0
    bsf PORTA,3,0
    btfss LCD_WriteNibbleNibble,3,0
    bcf PORTA,3,0
F1_000114 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] high LCD_EN
    bcf TRISA,7,0
    bsf LATA,7,0
F1_000115 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayUs 1
    movlw 2
_pblb__15
    decfsz WREG,F,0
    bra _pblb__15
    nop
    nop
F1_000116 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_EN
    bcf TRISA,7,0
    bcf LATA,7,0
F1_000117 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayUs 50
    movlw 50
    bra __delay_us_
;---------------------------------------------
F1_000121 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] Proc LCD_Command(Cmd As byte)
LCD_Command
F1_000122 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] low LCD_RS
    bcf TRISA,6,0
    bcf LATA,6,0
F1_000123 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble(Cmd >> 4)
    swapf LCD_CommandCmd,W,0
    andlw 15
    movwf LCD_WriteNibbleNibble,0
    rcall LCD_WriteNibble
F1_000124 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] LCD_WriteNibble(Cmd)
    movff LCD_CommandCmd,LCD_WriteNibbleNibble
    rcall LCD_WriteNibble
F1_000125 equ $ ; in [IRRISYS_PG_VB_POS_240725.BAS] DelayUs 50
    movlw 50
    bra __delay_us_
;---------------------------------------------
F1_EOF equ $ ; IRRISYS_PG_VB_POS_240725.BAS
__eof
;---------------------------------------------
; CONFIG FUSES
config OSC = INTIO67
config FCMEN = off
config IESO = off
config PWRT = off
config BOREN = SBORDIS
config BORV = 3
config WDT = off
config WDTPS = 32768
config CCP2MX = PORTC
config PBADEN = off
config LPT1OSC = off
config MCLRE = on
config STVREN = on
config LVP = off
config XINST = off
config DEBUG = off
config CP0 = off
config CP1 = off
config CP2 = off
config CPB = off
config CPD = off
config WRT0 = off
config WRT1 = off
config WRT2 = off
config WRTC = off
config WRTB = off
config WRTD = off
config EBTR0 = off
config EBTR1 = off
config EBTR2 = off
config EBTRB = off
    end
