#include "LCD.h"


// LCD Segment Mapping
const uint8 LCD_Char_Map[] =
{
  LCD_A+LCD_B+LCD_C+LCD_D+LCD_E+LCD_F,        // '0' or 'O'
  LCD_B+LCD_C,                                // '1' or 'I'
  LCD_A+LCD_B+LCD_D+LCD_E+LCD_G,              // '2' or 'Z'
  LCD_A+LCD_B+LCD_C+LCD_D+LCD_G,              // '3'
  LCD_B+LCD_C+LCD_F+LCD_G,                    // '4' or 'y'
  LCD_A+LCD_C+LCD_D+LCD_F+LCD_G,              // '5' or 'S'
  LCD_A+LCD_C+LCD_D+LCD_E+LCD_F+LCD_G,        // '6' or 'b'
  LCD_A+LCD_B+LCD_C,                          // '7'
  LCD_A+LCD_B+LCD_C+LCD_D+LCD_E+LCD_F+LCD_G,  // '8' or 'B'
  LCD_A+LCD_B+LCD_C+LCD_F+LCD_G,              // '9' or 'g'
  LCD_A+LCD_B+LCD_C+LCD_E+LCD_F+LCD_G,        // 'A'
  LCD_A+LCD_D+LCD_E+LCD_F,                    // 'C'
  LCD_B+LCD_C+LCD_D+LCD_E+LCD_G,              // 'd'
  LCD_A+LCD_D+LCD_E+LCD_F+LCD_G,              // 'E'
  LCD_A+LCD_E+LCD_F+LCD_G,                    // 'F'
  LCD_B+LCD_C+LCD_E+LCD_F+LCD_G,              // 'H'
  LCD_B+LCD_C+LCD_D+LCD_E,                    // 'J'
  LCD_D+LCD_E+LCD_F,                          // 'L'
  LCD_A+LCD_B+LCD_E+LCD_F+LCD_G,              // 'P'
  LCD_B+LCD_C+LCD_D+LCD_E+LCD_F,              // 'U'
  LCD_A+LCD_B+LCD_F+LCD_G, 					  // degree symbol
  0x00,										  // blank
  LCD_G										  // minus sign
};

const uint8 LCD_MAX_CHARS = (sizeof(LCD_Char_Map)/sizeof(uint8));



//
// Clear LCD
//
void LCDClear(void)
{
  int i;

  for(i = LCD_MEM_OFFSET; i < (LCD_MEM_OFFSET+LCD_MEM_LOC);  i++)
  {
    LCDMEM[i] = 0;
  }
}


//
// Display all segments on LCD
//
void LCDDispayAllSegments(void)
{
  int i;

  for(i = LCD_MEM_OFFSET; i < (LCD_MEM_OFFSET+LCD_MEM_LOC);  i++)
  {
    LCDMEM[i] = 0xff;
  }
}


//
// Display character on LCD
//
void LCDDisplayCharacter(uint8 pos, uint8 index)
{
  LCDMEM[pos + LCD_MEM_OFFSET] &= ~LCD_Char_Map[8];

  if( pos < LCD_NUM_DIGITS )
  {
    if( index < LCD_MAX_CHARS )
    {
      LCDMEM[pos + LCD_MEM_OFFSET] |= LCD_Char_Map[index];
    }
  }
}


//
// Display power level on LCD
//
void LCDDisplayPowerLevel(uint8 lvl)
{
  LCDMEM[7+LCD_MEM_OFFSET] &= ~(BIT1+BIT2+BIT3);
  LCDMEM[8+LCD_MEM_OFFSET] &= ~(BIT0+BIT1+BIT2+BIT3);

  switch(lvl)
  {
  case LCD_PWR_LVL_0:
    LCDMEM[7+LCD_MEM_OFFSET] |= (BIT1);
    LCDMEM[8+LCD_MEM_OFFSET] |= (BIT0);
    break;
  case LCD_PWR_LVL_1:
    LCDMEM[7+LCD_MEM_OFFSET] |= (BIT1);
    LCDMEM[8+LCD_MEM_OFFSET] |= (BIT0+BIT1);
    break;
  case LCD_PWR_LVL_2:
    LCDMEM[7+LCD_MEM_OFFSET] |= (BIT1);
    LCDMEM[8+LCD_MEM_OFFSET] |= (BIT0+BIT1+BIT2);
    break;
  case LCD_PWR_LVL_3:
    LCDMEM[7+LCD_MEM_OFFSET] |= (BIT1);
    LCDMEM[8+LCD_MEM_OFFSET] |= (BIT0+BIT1+BIT2+BIT3);
    break;
  case LCD_PWR_LVL_4:
    LCDMEM[7+LCD_MEM_OFFSET] |= (BIT1+BIT3);
    LCDMEM[8+LCD_MEM_OFFSET] |= (BIT0+BIT1+BIT2+BIT3);
    break;
  case LCD_PWR_LVL_5:
    LCDMEM[7+LCD_MEM_OFFSET] |= (BIT1+BIT2+BIT3);
    LCDMEM[8+LCD_MEM_OFFSET] |= (BIT0+BIT1+BIT2+BIT3);
    break;
  };
}


//
// Diplay function F1 thru F5 on LCD
//
void LCDDisplayFunction(uint8 mask)
{
  uint8 tmp;

  LCDMEM[7+LCD_MEM_OFFSET] &= ~0xf1;

  if( mask < LCD_FUNC_OFF )
  {
    tmp = mask << 4;

    if( (mask & LCD_F5) )
    {
      tmp |= BIT0;
    }

    LCDMEM[7+LCD_MEM_OFFSET] |= tmp;
  }
}


//
// Display battery level on LCD
//
void LCDDisplayBatteryLevel(uint8 lvl)
{
  LCDMEM[9+LCD_MEM_OFFSET] &= ~(BIT0+BIT1+BIT2+BIT3);

  switch(lvl)
  {
  case LCD_BATT_LOW:
    LCDMEM[9+LCD_MEM_OFFSET] |= (BIT0+BIT3);
    break;
  case LCD_BATT_HALF:
    LCDMEM[9+LCD_MEM_OFFSET] |= (BIT0+BIT2+BIT3);
    break;
  case LCD_BATT_FULL:
    LCDMEM[9+LCD_MEM_OFFSET] |= (BIT0+BIT1+BIT2+BIT3);
    break;
  };
}


//
// Display signal level on LCD
//
void LCDDisplaySignalLevel(uint8 lvl)
{
  LCDMEM[9+LCD_MEM_OFFSET] &= ~(BIT4+BIT5+BIT6+BIT7);

  switch(lvl)
  {
  case LCD_SIG_LVL_0:
    LCDMEM[9+LCD_MEM_OFFSET] |= (BIT4);
    break;
  case LCD_SIG_LVL_1:
    LCDMEM[9+LCD_MEM_OFFSET] |= (BIT4+BIT7);
    break;
  case LCD_SIG_LVL_2:
    LCDMEM[9+LCD_MEM_OFFSET] |= (BIT4+BIT6+BIT7);
    break;
  case LCD_SIG_LVL_3:
    LCDMEM[9+LCD_MEM_OFFSET] |= (BIT4+BIT5+BIT6+BIT7);
    break;
  };
}


//
// Display arrow on LCD
//
void LCDDisplayArrow(uint8 mask)
{
  LCDMEM[8+LCD_MEM_OFFSET] &= ~(BIT4+BIT5+BIT6+BIT7);

  if( mask < LCD_ARROW_OFF )
  {
    LCDMEM[8+LCD_MEM_OFFSET] |= (mask << 4);
  }
}


//
// Display symbol on LCD
//
void LCDDisplaySymbol(uint8 mask)
{
  uint8 tmp;

  LCDMEM[10+LCD_MEM_OFFSET] &= ~(BIT0+BIT1+BIT2+BIT4+BIT5+BIT6+BIT7);

  if( mask < LCD_SYM_OFF )
  {
    tmp = mask << 4;
    tmp |= mask >> 4;

    LCDMEM[10+LCD_MEM_OFFSET] |= tmp;
  }
}


//
// Display special character on LCD
//
void LCDDisplaySpecialCharacter(uint16 mask)
{
  uint16 x;
  uint8 index = 0;

  if( mask & LCD_8BC )
  {
    LCDMEM[10+LCD_MEM_OFFSET] |= BIT3;
  }
  else
  {
    LCDMEM[10+LCD_MEM_OFFSET] &= ~BIT3;
  }

  for(x = 1; x < LCD_8BC; x <<= 1)
  {
    if( x & mask )
    {
      LCDMEM[index+LCD_MEM_OFFSET] |= BIT7;
    }
    else
    {
      LCDMEM[index+LCD_MEM_OFFSET] &= ~(BIT7);
    }

    ++index;
  }
}


#if LCD_TEST > 0

void testChar(void)
{
  static uint8 pos = 0;
  static uint8 index = 0;

  LCDDisplayCharacter(pos, index);

  if( pos++ >= LCD_NUM_DIGITS )
  {
    pos = 0;

    if( index++ >= LCD_MAX_CHARS )
    {
      index = 0;
    }
  }
}


void testSpecialChar(void)
{
  static uint16 mask = 1;

  LCDDisplaySpecialCharacter(mask);

  if( mask >= LCD_SPC_CHAR_OFF )
  {
    mask = 1;
  }
  else
  {
    mask <<= 1;
  }
}


void testPwrLvl(void)
{
  static uint8 lvl = LCD_PWR_LVL_0;

  LCDDisplayPowerLevel(lvl);

  if( lvl >= LCD_PWR_OFF )
  {
    lvl = LCD_PWR_LVL_0;
  }
  else
  {
    lvl <<= 1;
  }
}



void testSigLvl(void)
{
  static uint8 lvl = LCD_SIG_LVL_0;

  LCDDisplaySignalLevel(lvl);

  if( lvl >= LCD_SIG_OFF )
  {
    lvl = LCD_SIG_LVL_0;
  }
  else
  {
    lvl <<= 1;
  }
}


void testBatt(void)
{
  static uint8 lvl = LCD_BATT_LOW;

  LCDDisplayBatteryLevel(lvl);

  if( lvl >= LCD_BATT_OFF )
  {
    lvl = LCD_BATT_LOW;
  }
  else
  {
    lvl <<= 1;
  }
}


void testFunc(void)
{
  static uint8 func = LCD_F1;

  LCDDisplayFunction(func);

  if( func >= LCD_FUNC_OFF )
  {
    func = LCD_F1;
  }
  else
  {
    func <<= 1;
  }
}


void testArrow(void)
{
  static uint8 dir = LCD_ARROW_UP;

  LCDDisplayArrow(dir);

  if( dir >= LCD_ARROW_OFF )
  {
    dir = LCD_ARROW_UP;
  }
  else
  {
    dir <<= 1;
  }
}


void testSymbol(void)
{
  static uint8 sym = LCD_SYM_DOL;

  LCDDisplaySymbol(sym);

  if( sym >= LCD_SYM_OFF )
  {
    sym = LCD_SYM_DOL;
  }
  else
  {
    sym <<= 1;
  }
}


void testAll(void)
{
  uint8 x;

  for(x = 0; x < 3; x++)
  {
    _BIS_SR(LPM3_bits + GIE);               // LPM3, enable interrupts

    LCDDispayAllSegments();
  }

  LCDClear();
}

#endif // LCD_TEST
