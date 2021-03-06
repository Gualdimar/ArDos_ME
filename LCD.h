// *** Переменные для управления дисплеем ***
#define pulseClock bitClear(SCK_PORT, SCK_BIT); asm ("nop"); bitSet(SCK_PORT, SCK_BIT)
#define resetLCD bitSet(DC_PORT, DC_BIT); bitSet(MOSI_PORT, MOSI_BIT); bitSet(SCK_PORT, SCK_BIT); bitClear(RST_PORT, RST_BIT); _delay_ms(10); bitSet(RST_PORT, RST_BIT)

#define fontbyte(x) pgm_read_byte(&cfont.font[x])
#define bitmapbyte(x) pgm_read_byte(&bitmap[x])

#define bitmapdatatype uint8_t*

// ------------------
#define LEFT 0
#define RIGHT 255
#define CENTER 254

#define LCD_COMMAND 0
#define LCD_DATA 1

// *** Команды для PCD8544 ***

// Основные команды
#define PCD8544_POWERDOWN      0x04
#define PCD8544_ENTRYMODE     0x02
#define PCD8544_EXTENDEDINSTRUCTION 0x01
#define PCD8544_DISPLAYBLANK    0x00
#define PCD8544_DISPLAYNORMAL   0x04
#define PCD8544_DISPLAYALLON    0x01
#define PCD8544_DISPLAYINVERTED   0x05
// Основные инструкции
#define PCD8544_FUNCTIONSET     0x20
#define PCD8544_DISPLAYCONTROL    0x08
#define PCD8544_SETYADDR      0x40
#define PCD8544_SETXADDR      0x80
// Внешние инструкции
#define PCD8544_SETTEMP       0x04
#define PCD8544_SETBIAS       0x10
#define PCD8544_SETVOP        0x80
// Установки дисплея
#define LCD_BIAS          0x03  // Range: 0-7 (0x00-0x07)
#define LCD_TEMP          0x02  // Range: 0-3 (0x00-0x03)
#define LCD_CONTRAST        0x46  // Range: 0-127 (0x00-0x7F)

struct
{
  uint8_t* font;
  uint8_t x_size;
  uint8_t y_size;
  uint8_t offset;
  uint8_t numchars;
  uint8_t inverted;
} cfont;

void _LCD_Write(unsigned char data, unsigned char mode)
{
  switch (mode) {
    case LCD_COMMAND: bitClear(DC_PORT, DC_BIT); break;
    case LCD_DATA: bitSet(DC_PORT, DC_BIT); break;
  }

  for (uint8_t c = 0; c < 8; c++)
  {
    if (data & 0x80)
      bitSet(MOSI_PORT, MOSI_BIT);
    else
      bitClear(MOSI_PORT, MOSI_BIT);
    data = data << 1;
    pulseClock;
  }
}

void InitLCD(uint8_t contrast = LCD_CONTRAST)
{
  resetLCD;

  _LCD_Write(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION, LCD_COMMAND);
  _LCD_Write(PCD8544_SETVOP | contrast, LCD_COMMAND);
  _LCD_Write(PCD8544_SETTEMP | LCD_TEMP, LCD_COMMAND);
  _LCD_Write(PCD8544_SETBIAS | LCD_BIAS, LCD_COMMAND);
  _LCD_Write(PCD8544_FUNCTIONSET, LCD_COMMAND);
  _LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
  _LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
  for (uint16_t c = 0; c < 504; c++)
    _LCD_Write(0x00, LCD_DATA);
  _LCD_Write(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL, LCD_COMMAND);

  cfont.font = 0;
}

void setContrastLCD(uint8_t contrast)
{
  _LCD_Write(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION, LCD_COMMAND);
  _LCD_Write(PCD8544_SETVOP | contrast, LCD_COMMAND);
  _LCD_Write(PCD8544_FUNCTIONSET, LCD_COMMAND);
}

void enableSleepLCD()
{
  _LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
  _LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
  for (uint16_t b = 0; b < 504; b++)
    _LCD_Write(0x00, LCD_DATA);
  _LCD_Write(PCD8544_FUNCTIONSET | PCD8544_POWERDOWN, LCD_COMMAND);
}

void disableSleepLCD(uint8_t contrast)
{
  _LCD_Write(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION, LCD_COMMAND);
  _LCD_Write(PCD8544_SETVOP | contrast, LCD_COMMAND);
  _LCD_Write(PCD8544_SETTEMP | LCD_TEMP, LCD_COMMAND);
  _LCD_Write(PCD8544_SETBIAS | LCD_BIAS, LCD_COMMAND);
  _LCD_Write(PCD8544_FUNCTIONSET, LCD_COMMAND);
  _LCD_Write(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL, LCD_COMMAND);
}

void clrScrLCD()
{
  _LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
  _LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
  for (uint16_t c = 0; c < 504; c++)
    _LCD_Write(0x00, LCD_DATA);
}

void  clrRowLCD(uint8_t row, uint8_t start_x = 0, uint8_t end_x = 83)
{
  _LCD_Write(PCD8544_SETYADDR | row, LCD_COMMAND);
  _LCD_Write(PCD8544_SETXADDR | start_x, LCD_COMMAND);
  for (uint8_t c = start_x; c <= end_x; c++) {
      _LCD_Write(0x00, LCD_DATA);
  }
  _LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
  _LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
}

void _print_char(unsigned char c, uint8_t x, uint8_t row)
{
    if (((x + cfont.x_size) <= 84) && (row + (cfont.y_size >> 3) <= 6))
    {
        for (uint8_t rowcnt = 0; rowcnt < (cfont.y_size >> 3); rowcnt++)
        {
            _LCD_Write(PCD8544_SETYADDR | (row + rowcnt), LCD_COMMAND);
            _LCD_Write(PCD8544_SETXADDR | x, LCD_COMMAND);

            if (c == 0x20 && cfont.offset == 0x2B)
                c = 0x2B;

            uint16_t font_idx = ((c - cfont.offset) * (cfont.x_size * (cfont.y_size >> 3))) + 4;

            for (uint16_t cnt = 0; cnt < cfont.x_size; cnt++)
            {
                switch (cfont.inverted) {
                case 0: _LCD_Write(fontbyte(font_idx + cnt + (rowcnt * cfont.x_size)), LCD_DATA); break;
                case 1: _LCD_Write(~(fontbyte(font_idx + cnt + (rowcnt * cfont.x_size))), LCD_DATA); break;
                }
            }
        }
        _LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
        _LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
    }
}

void printLCD(char *st, uint8_t x, uint8_t y)
{
  uint8_t stl, row;

  stl = strlen(st);

  switch (x) {
    case RIGHT: x = 84 - (stl * cfont.x_size); break;
    case CENTER: x = (84 - (stl * cfont.x_size)) >> 1; break;
  }
  row = y >> 3;

  for (int cnt = 0; cnt < stl; cnt++)
    _print_char(*st++, x + (cnt * (cfont.x_size)), row);
}

void setFontLCD(uint8_t* font)
{
  cfont.font = font;
  cfont.x_size = fontbyte(0);
  cfont.y_size = fontbyte(1);
  cfont.offset = fontbyte(2);
  cfont.numchars = fontbyte(3);
  cfont.inverted = 0;
}

void drawBitmapLCD(uint8_t x, uint8_t y, bitmapdatatype bitmap, uint8_t sx, uint8_t sy, boolean inv = 0)
{
  uint8_t steps = sx;
  uint8_t starty, rows;

  starty = y >> 3;

  if (sy % 8 == 0)
    rows = sy >> 3;
  else
    rows = (sy >> 3) + 1;

  for (uint8_t cy = 0; cy < rows; cy++)
  {
    _LCD_Write(PCD8544_SETYADDR | (starty + cy), LCD_COMMAND);
    _LCD_Write(PCD8544_SETXADDR | x, LCD_COMMAND);
    switch (inv) {
      case 0: for (uint8_t cx = 0; cx < steps; cx++) _LCD_Write(bitmapbyte(cx + (cy * sx)), LCD_DATA); break;
      case 1: for (uint8_t cx = 0; cx < steps; cx++) _LCD_Write(~(bitmapbyte(cx + (cy * sx))), LCD_DATA); break;
    }
  }
  _LCD_Write(PCD8544_SETYADDR, LCD_COMMAND);
  _LCD_Write(PCD8544_SETXADDR, LCD_COMMAND);
}
