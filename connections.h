//Соединения периферии с портами МК
//    PORTD (0 - D0 | 1 - D1 | 2 - D2 | 3 - D3 | 4 - D4 | 5 - D5 | 6 - D6 | 7 - D7) PIND
//  PORTB (0 - D8 | 1 - D9 | 2 - D10 | 3 - D11 | 4 - D12 | 5 - D13) PINB
//    PORTC (0 - A0 | 1 - A1 | 2 - A2 | 3 - A3 | 4 - A4 | 5 - A5) PINC

#define DDR_REG(portx)  (*(&portx-1))

//назначаем кнопки//
//пин кнопки ОК D7
#define OK_BIT   7 // D7
#define OK_PORT  PORTD
#define OK_PIN   PIND

#define OK_READ   (bitRead(OK_PIN, OK_BIT))
#define OK_SET   (bitSet(OK_PORT, OK_BIT))
#define OK_MODE  (bitClear((DDR_REG(OK_PORT)), OK_BIT))

#define OK_INIT  OK_SET; OK_MODE

//пин кнопки DOWN D6
#define DOWN_BIT   6 // D6
#define DOWN_PORT  PORTD
#define DOWN_PIN   PIND

#define DOWN_READ   (bitRead(DOWN_PIN, DOWN_BIT))
#define DOWN_SET   (bitSet(DOWN_PORT, DOWN_BIT))
#define DOWN_MODE   (bitClear((DDR_REG(DOWN_PORT)), DOWN_BIT))

#define DOWN_INIT  DOWN_SET; DOWN_MODE

//пин кнопки UP D8
#define UP_BIT   0 // D8
#define UP_PORT  PORTB
#define UP_PIN   PINB

#define UP_READ   (bitRead(UP_PIN, UP_BIT))
#define UP_SET   (bitSet(UP_PORT, UP_BIT))
#define UP_MODE   (bitClear((DDR_REG(UP_PORT)), UP_BIT))

#define UP_INIT  UP_SET; UP_MODE

//пин пищалки
#define BUZZ_BIT   4 // D4
#define BUZZ_PORT  PORTD

#define BUZZ_OFF   (bitClear(BUZZ_PORT, BUZZ_BIT))
#define BUZZ_ON   (bitSet(BUZZ_PORT, BUZZ_BIT))
#define BUZZ_READ (bitRead(BUZZ_PORT, BUZZ_BIT))
#define BUZZ_MODE   (bitSet((DDR_REG(BUZZ_PORT)), BUZZ_BIT))

#define BUZZ_INIT  BUZZ_OFF; BUZZ_MODE

//пин детектора
#define DET_1_BIT   2 // D2
#define DET_1_PORT  PORTD

#define DET_1_SET   (bitSet(DET_1_PORT, DET_1_BIT))
#define DET_1_MODE   (bitClear((DDR_REG(DET_1_PORT)), DET_1_BIT))

#define DET_1_INIT  DET_1_SET; DET_1_MODE

#define DET_2_BIT   3 // D3
#define DET_2_PORT  PORTD

#define DET_2_SET   (bitSet(DET_2_PORT, DET_2_BIT))
#define DET_2_MODE   (bitClear((DDR_REG(DET_2_PORT)), DET_2_BIT))

#define DET_2_INIT  DET_2_SET; DET_2_MODE

#define DET_3_BIT   5 // A5
#define DET_3_PORT  PORTC

#define DET_3_SET   (bitSet(DET_3_PORT, DET_3_BIT))
#define DET_3_MODE   (bitClear((DDR_REG(DET_3_PORT)), DET_3_BIT))

#define DET_3_INIT  DET_3_SET; DET_3_MODE

#define DET_4_BIT   4 // A4
#define DET_4_PORT  PORTC

#define DET_4_SET   (bitSet(DET_4_PORT, DET_4_BIT))
#define DET_4_MODE   (bitClear((DDR_REG(DET_4_PORT)), DET_4_BIT))

#define DET_4_INIT  DET_4_SET; DET_4_MODE

//пин преобразователя
#define CONV_BIT   5 // D5
#define CONV_PORT  PORTD

#define CONV_CLR   (bitClear(CONV_PORT, CONV_BIT))
#define CONV_MODE   (bitSet((DDR_REG(CONV_PORT)), CONV_BIT))

#define CONV_INIT  CONV_CLR; CONV_MODE

//пин индикации частиц D13
#define RAD_FLASH_BIT   5 // D13
#define RAD_FLASH_PORT  PORTB

#define RF_ON    (bitSet(RAD_FLASH_PORT, RAD_FLASH_BIT))
#define RF_OFF   (bitClear(RAD_FLASH_PORT, RAD_FLASH_BIT))
#define RF_READ  (bitRead(RAD_FLASH_PORT, RAD_FLASH_BIT))
#define RF_MODE  (bitSet((DDR_REG(RAD_FLASH_PORT)), RAD_FLASH_BIT))

#define RF_INIT  RF_OFF; RF_MODE

//пин подсветки A2
#define LIGHT_BIT   2 // A2
#define LIGHT_PORT  PORTC

#ifdef LIGHT_INV
#define LIGHT_ON    (bitSet(LIGHT_PORT, LIGHT_BIT))
#define LIGHT_OFF   (bitClear(LIGHT_PORT, LIGHT_BIT))
#else
#define LIGHT_ON    (bitClear(LIGHT_PORT, LIGHT_BIT))
#define LIGHT_OFF   (bitSet(LIGHT_PORT, LIGHT_BIT))
#endif
#define LIGHT_READ  (bitRead(LIGHT_PORT, LIGHT_BIT))
#define LIGHT_MODE  (bitSet((DDR_REG(LIGHT_PORT)), LIGHT_BIT))

#define LIGHT_INIT  LIGHT_OFF; LIGHT_MODE

//пин питания дисплея A3
#define PWR_LCD_BIT   3 // A3
#define PWR_LCD_PORT  PORTC

#define PWR_LCD_ON    bitClear(PWR_LCD_PORT, PWR_LCD_BIT)
#define PWR_LCD_MODE   bitSet((DDR_REG(PWR_LCD_PORT)), PWR_LCD_BIT)

#define PWR_LCD_INIT  PWR_LCD_ON; PWR_LCD_MODE

//обьявляем дисплей с указанием пинов подключения
//пин SCK дисплея A1
#define SCK_BIT  1
#define SCK_PORT PORTC
#define SCK_CLR  (bitClear(SCK_PORT, SCK_BIT))
#define SCK_MODE  (bitSet((DDR_REG(SCK_PORT)), SCK_BIT))

//пин MOSI дисплея A0
#define MOSI_BIT  0
#define MOSI_PORT PORTC
#define MOSI_CLR  (bitClear(MOSI_PORT, MOSI_BIT))
#define MOSI_MODE  (bitSet((DDR_REG(MOSI_PORT)), MOSI_BIT))

//пин DC дисплея D12
#define DC_BIT  4
#define DC_PORT PORTB
#define DC_CLR  (bitClear(DC_PORT, DC_BIT))
#define DC_MODE  (bitSet((DDR_REG(DC_PORT)), DC_BIT))

//пин RST дисплея D10
#define RST_BIT  2
#define RST_PORT PORTB
#define RST_CLR  (bitClear(RST_PORT, RST_BIT))
#define RST_MODE  (bitSet((DDR_REG(RST_PORT)), RST_BIT))

//пин CE дисплея D11
#define LCD_BIT   3 // D11
#define LCD_PORT  PORTB
#define LCD_ON    bitClear(LCD_PORT, LCD_BIT)
#define LCD_MODE   bitSet((DDR_REG(LCD_PORT)), LCD_BIT)

#define LCD_INIT  SCK_CLR; SCK_MODE; MOSI_CLR; MOSI_MODE; DC_CLR; DC_MODE; RST_CLR; RST_MODE; LCD_ON; LCD_MODE
