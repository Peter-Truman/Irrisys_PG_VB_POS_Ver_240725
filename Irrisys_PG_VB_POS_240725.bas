'****************************************************************
'*  Name    : Irrisys_PG_VB_POS_240725.BAS                      *
'*  Author  : Peter W Truman                                    *
'*  Notice  : Copyright (c) 2025 PCT Remote Sensing Pty Ltd     *
'*          : All Rights Reserved                               *
'*  Date    : 24/07/2025                                        *
'*  Version : 1.0                                               *
'*  Notes   :                                                   *
'*          :                                                   *
'****************************************************************

Device = 18F2525

Config_Start
  OSC = INTIO67	;Internal oscillator block, port function on RA6 and RA7
  FCMEN = OFF	;Fail-Safe Clock Monitor disabled
  IESO = OFF	;Oscillator Switchover mode disabled
  PWRT = OFF	;PWRT disabled
  BOREN = SBORDIS	;Brown-out Reset enabled in hardware only (SBOREN is disabled)
  BORV = 3	;Minimum setting
  WDT = OFF	;WDT disabled (control is placed on the SWDTEN bit)
  WDTPS = 32768	;1:32768
  CCP2MX = PORTC	;CCP2 input/output is multiplexed with RC1
  PBADEN = OFF	;PORTB<4:0> pins are configured as digital I/O on Reset
  LPT1OSC = OFF	;Timer1 configured for higher power operation
  MCLRE = On	;MCLR pin enabled; RE3 input pin disabled
  STVREN = On	;Stack full/underflow will cause Reset
  LVP = OFF	;Single-Supply ICSP disabled
  XINST = OFF	;Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
  Debug = OFF	;Background debugger disabled, RB6 and RB7 configured as general purpose I/O pins
  Cp0 = OFF	;Block 0 (000800-003FFFh) not code-protected
  CP1 = OFF	;Block 1 (004000-007FFFh) not code-protected
  CP2 = OFF	;Block 2 (008000-00BFFFh) not code-protected
  CPB = OFF	;Boot block (000000-0007FFh) not code-protected
  CPD = OFF	;Data EEPROM not code-protected
  WRT0 = OFF	;Block 0 (000800-003FFFh) not write-protected
  WRT1 = OFF	;Block 1 (004000-007FFFh) not write-protected
  WRT2 = OFF	;Block 2 (008000-00BFFFh) not write-protected
  WRTC = OFF	;Configuration registers (300000-3000FFh) not write-protected
  WRTB = OFF	;Boot Block (000000-0007FFh) not write-protected
  WRTD = OFF	;Data EEPROM not write-protected
  EBTR0 = OFF	;Block 0 (000800-003FFFh) not protected from table reads executed in other blocks
  EBTR1 = OFF	;Block 1 (004000-007FFFh) not protected from table reads executed in other blocks
  EBTR2 = OFF	;Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks
  EBTRB = OFF	;Boot Block (000000-0007FFh) not protected from table reads executed in other blocks
Config_End

;**** End of Fuse Configurator Settings ****
;-------------------------------------------------------------------------------
OSCCON.6=1
OSCCON.5=1
OSCCON.4=1          'setup at 8MGz
OSCTUNE.6=1         'x 4

Declare Xtal=32

' LCD Pin Definitions (NHD-0420AZ-FL-YBW-33V3, 3.3V, 4-bit parallel)
Symbol LCD_RS = PORTA.6
Symbol LCD_RW = PORTA.5
Symbol LCD_EN = PORTA.7
Symbol LCD_D4 = PORTA.0
Symbol LCD_D5 = PORTA.1
Symbol LCD_D6 = PORTA.2
Symbol LCD_D7 = PORTA.3

' Rotary Encoder Definitions
Symbol ENC_A = PORTB.1
Symbol ENC_B = PORTB.2
Symbol ENC_SW = PORTB.6

' RTC Interrupt
Symbol RTC_INT = PORTB.0

' I2C Pins for DS3231
Symbol SDA = PORTC.4
Symbol SCL = PORTC.3

' Constants
Symbol LCD_WIDTH = 20
Symbol LCD_Lines = 4
Symbol LONG_PRESS = 2000  ' 2 seconds for long press (in ms)
Symbol I2C_ADDR_DS3231 = $D0  ' DS3231 I2C address

' LCD Initialization for NHD-0420AZ-FL-YBW-33V3
Proc LCD_Init()
  Low LCD_RS
  Low LCD_RW
  Low LCD_EN
  Low LCD_D4
  Low LCD_D5
  Low LCD_D6
  Low LCD_D7
  DelayMS 50          ' Wait for LCD power-up (>40ms per datasheet)
  LCD_WriteNibble($03)
  DelayMS 5           ' Wait >4.1ms
  LCD_WriteNibble($03)
  DelayMS 5           ' Wait >100us
  LCD_WriteNibble($03)
  DelayMS 5
  LCD_WriteNibble($02)  ' Set 4-bit mode
  LCD_Command($28)      ' 4-bit, 2 lines (configured for 4x20), 5x8 font
  LCD_Command($0C)      ' Display on, cursor off, blink off
  LCD_Command($06)      ' Entry mode: increment, no shift
  LCD_Command($01)      ' Clear display
  DelayMS 2            ' Clear display delay (>1.53ms)
EndProc

' Write Nibble to LCD
Proc LCD_WriteNibble(Nibble As Byte)
  LCD_D4 = Nibble.0
  LCD_D5 = Nibble.1
  LCD_D6 = Nibble.2
  LCD_D7 = Nibble.3
  High LCD_EN   ' Enable pulse (>450ns)
  DelayUS 1
  Low LCD_EN
  DelayUS 50           ' Execution time (>37us per datasheet)
EndProc

' Send Command to LCD
Proc LCD_Command(Cmd As Byte)
  Low LCD_RS
  LCD_WriteNibble(Cmd >> 4)
  LCD_WriteNibble(Cmd)
  DelayUS 50           ' Command execution time
EndProc

' Write Character to LCD
Proc LCD_WriteChar(Ch As Byte)
  High LCD_RS
  LCD_WriteNibble(Ch >> 4)
  LCD_WriteNibble(Ch)
  DelayUS 50           ' Data write time
EndProc

' Set LCD Cursor Position for 4x20 Display
Proc LCD_SetCursor(B_Line As Byte, Pos As Byte)
  Dim Addr As Byte
  Select B_Line
    Case 1
        Addr = $80 + Pos  ' Line 1: 0x00-0x13
    Case 2
        Addr = $C0 + Pos  ' Line 2: 0x40-0x53
    Case 3
        Addr = $94 + Pos  ' Line 3: 0x14-0x27
    Case 4
        Addr = $D4 + Pos  ' Line 4: 0x54-0x67
    Else
        Addr = $80        ' Default to line 1, position 0
  EndSelect
  LCD_Command(Addr)
EndProc

' Clear LCD
Proc LCD_Clear()
    Cls
  'CD_Command($01)
  DelayMS 2            ' Clear display delay
EndProc

' Main Program
Main:
  ADCON1 = $0F       ' All pins digital
  TRISA = %00000000  ' PORTA as output for LCD
  TRISB = %11000111  ' PORTB.0,1,2,6 as inputs for encoder and RTC
  TRISC = %00011000  ' PORTC.3,4 for I2C
  OSCCON = %01110010 ' 16 MHz internal oscillator

  LCD_Init()
  LCD_Clear()
  LCD_SetCursor(1, 0)
  LCD_WriteChar("H")
  LCD_WriteChar("e")
  LCD_WriteChar("l")
  LCD_WriteChar("l")
  LCD_WriteChar("o")
  LCD_WriteChar(" ")
  LCD_WriteChar("W")
  LCD_WriteChar("o")
  LCD_WriteChar("r")
  LCD_WriteChar("l")
  LCD_WriteChar("d")

  While 1 = 1
    DelayMS 1000  ' Loop forever with a delay
  Wend
End
