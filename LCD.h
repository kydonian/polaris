#ifndef __LCD_H__
#define __LCD_H__

#include "typedefs.h"
#include  <msp430xG46x.h>

#define LCD_TEST 		0					// Set if you wish to use the LCD
											// testings functions provided at the end
											// of this header

// LCD Constants
#define LCD_NUM_DIGITS  7                   // Number of digits on LCD
#define LCD_MEM_OFFSET  2                   // Offset from LCDMEM[0]
#define LCD_MEM_LOC     11                  // Num of LCDMEM[] locations used

// LCD characters defined in LCD_Char_Array
typedef enum LCDChar
{
	LCD_CHAR_0,
	LCD_CHAR_1,
	LCD_CHAR_2,
	LCD_CHAR_3,
	LCD_CHAR_4,
	LCD_CHAR_5,
	LCD_CHAR_6,
	LCD_CHAR_7,
	LCD_CHAR_8,
	LCD_CHAR_9,
	LCD_CHAR_A,
	LCD_CHAR_C,
	LCD_CHAR_d,
	LCD_CHAR_E,
	LCD_CHAR_F,
	LCD_CHAR_H,
	LCD_CHAR_J,
	LCD_CHAR_L,
	LCD_CHAR_P,
	LCD_CHAR_U,
	LCD_CHAR_DEGREE,
	LCD_CHAR_BLANK,
	LCD_CHAR_MINUS,
	LCD_CHAR_NUM
}	LCDChar_t;

// LCD Segments
#define LCD_A    BIT0
#define LCD_B    BIT1
#define LCD_C    BIT2
#define LCD_D    BIT3
#define LCD_E    BIT6
#define LCD_F    BIT4
#define LCD_G    BIT5
#define LCD_H    BIT7

// Display Power Level
#define LCD_PWR_LVL_0   0x01
#define LCD_PWR_LVL_1   0x02
#define LCD_PWR_LVL_2   0x04
#define LCD_PWR_LVL_3   0x08
#define LCD_PWR_LVL_4   0x10
#define LCD_PWR_LVL_5   0x20
#define LCD_PWR_OFF     0x40

void LCDDisplayPowerLevel(uint8 lvl);


// Display Function F1 thru F5
#define LCD_F1          0x01
#define LCD_F2          0x02
#define LCD_F3          0x04
#define LCD_F4          0x08
#define LCD_F5          0x10
#define LCD_FUNC_OFF    0x20

void LCDDisplayFunction(uint8 mask);


// Display Battery Level
#define LCD_BATT_LOW    0x01
#define LCD_BATT_HALF   0x02
#define LCD_BATT_FULL   0x04
#define LCD_BATT_OFF    0x08

void LCDDisplayBatteryLevel(uint8 lvl);


// Display Signal Level
#define LCD_SIG_LVL_0   0x01
#define LCD_SIG_LVL_1   0x02
#define LCD_SIG_LVL_2   0x04
#define LCD_SIG_LVL_3   0x08
#define LCD_SIG_OFF     0x10

void LCDDisplaySignalLevel(uint8 lvl);


// Display Arrow
#define LCD_ARROW_UP     0x01
#define LCD_ARROW_RIGHT  0x02
#define LCD_ARROW_DOWN   0x04
#define LCD_ARROW_LEFT   0x08
#define LCD_ARROW_OFF    0x10

void LCDDisplayArrow(uint8 mask);


// Display Symbol
#define LCD_SYM_DOL      0x01
#define LCD_SYM_ERR      0x02
#define LCD_SYM_MINUS    0x04
#define LCD_SYM_MEM      0x08
#define LCD_SYM_ENV      0x10
#define LCD_SYM_TX       0x20
#define LCD_SYM_RX       0x40
#define LCD_SYM_OFF      0x80

void LCDDisplaySymbol(uint8 mask);


// Special LCD characters
#define LCD_DP1          0x0001
#define LCD_DP2          0x0002
#define LCD_COL3         0x0004
#define LCD_DP4          0x0008
#define LCD_COL5         0x0010
#define LCD_DP6          0x0020
#define LCD_DP7          0x0040
#define LCD_8BC          0x0080
#define LCD_SPC_CHAR_OFF 0x0100

void LCDDisplaySpecialCharacter(uint16 mask);





//
// Clear LCD display
//
void LCDClear(void);


//
// Display all segments on LCD
//
void LCDDispayAllSegments(void);


//
// Display character on LCD
//
//   pos - character position on LCD
//   index - index into LCD_Char_Map[] array
//
void LCDDisplayCharacter(uint8 pos, uint8 index);


extern const uint8 LCD_MAX_CHARS;
extern const uint8 LCD_Char_Map[];


#if LCD_TEST > 0

void testAll(void);
void testSymbol(void);
void testArrow(void);
void testFunc(void);
void testBatt(void);
void testSigLvl(void);
void testPwrLvl(void);
void testSpecialChar(void);
void testChar(void);

#endif // LCD_TEST



#endif // __LCD_H__
