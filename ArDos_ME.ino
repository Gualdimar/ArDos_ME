#define VERSION "081120.1"
//----------------Библиотеки----------------
#include <Arduino.h>
#include <avr/delay.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

//---------------Конфигурации---------------
#include "connections.h"
#include "vars.h"
#include "config.h"
#include "LCD.h"
#include "resources.c"
#include "Fonts.c"

int atexit(void(* /*func*/)()) { //инициализация функций
    return 0;
}

int main()
{

#pragma region INIT
    //---------------Инициализация---------------
    initTimers(); //инициализация таймера millis();

#ifdef DEBUG
    PRR |= lowByte(_BV(7) | _BV(2)); //отключаем все лишнее (I2C | TIMER2 | SPI | UART)
    uart.begin(115200);
#else
    PRR |= lowByte(_BV(7) | _BV(2) | _BV(1)); //отключаем все лишнее (I2C | SPI | UART)
#endif
    ACSR |= 1 << ACD; //отключаем компаратор

    OK_INIT; //инициализация кнопки "ок"
    DOWN_INIT; //инициализация кнопки "вниз"
    UP_INIT; //инициализация кнопки "вверх"

    DET_1_INIT; //инициализация детектора частиц 1
    DET_2_INIT; //инициализация детектора частиц 2
    DET_3_INIT; //инициализация детектора частиц 3
    DET_4_INIT; //инициализация детектора частиц 4
    CONV_INIT; //инициализация преобразователя

    BUZZ_INIT; //инициализация бузера
    RF_INIT; //инициализация индикатора частиц

    LIGHT_INIT; //инициализация подсветки
    PWR_LCD_INIT; //инициализация питания дисплея
    LCD_INIT; //инициализация пинов дисплея
    PWR_LCD_ON; //включаем питание дисплея
    LCD_ON; //разрешаем работу с диплеем

    bat_adc = BAT_MAX;
    bat_level = 5;
    det_dbg_timeout = 3600;

    if (!OK_READ) flags.default_settings = 1; //если зажата кнопка ок при запуске, сбрасываем настройки
    if (!UP_READ && !DOWN_READ) flags.default_calibration = 1;

    config();
    flags.default_settings ? settings_update() : settings_read();
    flags.default_calibration ? calibration_update() : calibration_read();
    dose_read();

    InitLCD(LCD_CONTRAST); //инициализируем дисплей

    EICRA = 0b00001010; //настраиваем внешнее прерывание по спаду импульса на INT0 и INT1
    PCMSK2 = 0b10000000; // PCINT2 D7

    initDevice();

#pragma endregion
    //------------------------------------Основной цикл----------------------------------------------
    for (;;) {
        pump();
        power_manager();
        data_manager();
        alarm_manager();
        if (settings.errors_on) error_manager();
        button_manager();
        screen_manager();
    }
    return 0;
}
void initTimers() //инициализация таймера 0
{
    cli();
    //Timer0
    //устанавливаем режим шим
    bitSet(TCCR0A, WGM01);
    bitSet(TCCR0A, WGM00);

    //устанавливаем пределитель 64
    bitSet(TCCR0B, CS01);
    bitSet(TCCR0B, CS00);

    //утанавливаем прерывание по переполнению таймера 0
    bitSet(TIMSK0, TOIE0);

    //Timer1
    TCCR1A = (TCCR1A & 0xF0);
    TCCR1B = ((1 << WGM13) | (1 << WGM12) | 0b100);   // CTC mode + set prescaler
    ICR1 = 62499;          // Set timer top
    TIMSK1 |= (1 << OCIE1A);

    sei();
}
void initDevice() {
    LIGHT_ON;
    flags.bl_off = 0;

    drawBitmapLCD(0, 0, logo_img, 84, 24);//выводим лого
    setFontLCD(RusFont); //установка шрифта

    if (!flags.default_settings && !flags.default_calibration)
        printLCD("pfuheprf...", CENTER, 40); //загрузка...
    else if (flags.default_settings)
        printLCD("c,hjc yfcnhjtr", CENTER, 40); //сброс настроек
    else
        printLCD("c,hjc rfkb,h.", CENTER, 40); //сброс калибр.

    start_pump(); //первая накачка преобразователя

    clrRowLCD(4); //очистка строки 4
    clrRowLCD(5); //очистка строки 5

    printLCD("VT", CENTER, 32); //ME
    printLCD(VERSION, CENTER, 40); //версия ПО

    for (uint32_t t = millis() + (LOGO_TIME << 10); t > millis();) //ждём
        pump(); //накачка по обратной связи с АЦП

    PCICR = 0b00000010; // разрешаем внешнее прерывание PCINT1

    imp_buff_pos = 0;
    imp_buff[imp_buff_pos] = 0;
    avg_period = 0;
    avg_buff = 0;
    avg_avg = 0;

    detector_enable();

    flags.screen = 1;
    flags.statusbar_upd = 1;
    pointers.prev_pointer = NO_SCR;

    timers.bat_check = BAT_CHECK_TIME;

    timers.error = ERROR_REPEAT_TIME;

    clrScrLCD(); //очистка экрана
}
void detector_enable() {
    det_count = 0;
    det_own = 0;
    if (settings.det_1_on) {
        det_count += 1;
        det_own += settings.det_1_own;
        det_1 = 0;
    }
    if (settings.det_2_on) {
        det_count += 1;
        det_own += settings.det_2_own;
        det_2 = 0;
    }
    if (settings.det_3_on) {
        det_count += 1;
        det_own += settings.det_3_own;
        det_3 = 0;
    }
    if (settings.det_4_on) {
        det_count += 1;
        det_own += settings.det_4_own;
        det_4 = 0;
    }

    //det_off_th = DET_MAX_CNT / det_count;

    EIMSK = (0b00000000 | (settings.det_2_on << 1) | (settings.det_1_on)); //разрешаем внешнее прерывание INT0 и INT1
    PCMSK1 = (0b00000000 | (settings.det_4_on << 4) | (settings.det_3_on << 5)); // PCINT1 A4, A5
}
//---------------------------------Воспроизведение мелодии---------------------------------------
void play_sound(uint16_t arr[][3], uint8_t n, uint8_t s) //воспроизведение мелодии
{
    if (!flags.silent_mode) {
        static uint8_t pos; //переключатель мелодии
        static uint8_t sig; //переключатель мелодии
        static uint32_t sound_timer; //таймер мелодии

        if (sig != s) { //если сигнатура мелодии не равна старой
            pos = 0; //сбрасываем переключатель
            sig = s; //устанавливаем новую сигнатуру
        }
        if (millis() > sound_timer) { //если пришло время
            tone(4, pgm_read_word(&arr[pos][0]), pgm_read_word(&arr[pos][1])); //запускаем звук с задоной частотой и временем
            sound_timer = millis() + pgm_read_word(&arr[pos][2]); //устанавливаем паузу перед воспроизведением нового звука
            if (++pos > n - 1) { pos = 0; flags.sound = 0; } //переключпем на следующий семпл
        }
    }
}
ISR(TIMER1_COMPA_vect) {
    imp_buff_pos++;
    if (imp_buff_pos >= IMP_BUFF_LENGTH) imp_buff_pos = 0;
    imp_buff[imp_buff_pos] = 0;
    timers.dose_save++;
    if (timers.dose_scr > 0) timers.dose_scr++;
    timers.bat_check++;
    timers.auto_exit++;
    timers.backlight++;
    timers.error++;
    flags.data = 1;
    flags.screen = 1;
    if (flags.det_dbg) det_dbg.time++;

    det_1++;
    det_2++;
    det_3++;
    det_4++;
}
//-------------------------------Включение питания----------------------------------------------------
ISR(PCINT2_vect) //внешнее прерывание на пине PCINT2 - включение питания
{
}
#pragma region DETECT
//-------------------------------Детектирование частиц------------------------------------------------
ISR(INT0_vect) //внешнее прерывание на пине INT0 - считаем импульсы от счетчика 1
{
    detect();
    det_1 = 0;
    if (flags.det_dbg) det_dbg.d1++;
}
ISR(INT1_vect) //внешнее прерывание на пине INT1 - считаем импульсы от счетчика 2
{
    detect();
    det_2 = 0;
    if (flags.det_dbg) det_dbg.d2++;
}
ISR(PCINT1_vect) //внешнее прерывание на пине PCINT1 - считаем импульсы от счетчика 3 и 4
{
    if (!(PINC & (1 << DET_3_BIT))) {
        detect();
        det_3 = 0;
        if (flags.det_dbg) det_dbg.d3++;
   }
    else if (!(PINC & (1 << DET_4_BIT))) {
        detect();
        det_4 = 0;
        if (flags.det_dbg) det_dbg.d4++;
    }
}
void detect() {
    if (imp_buff[imp_buff_pos] != 65535) imp_buff[imp_buff_pos]++;

    if (settings.rfflash_on) {
        if (!flags.is_sleeping || settings.rf_nosleep) {
            RF_ON;
            rf_timer = millis() + RF_TIME;
        }
    }

    if (settings.det_sound && !flags.silent_mode) {
        if (!settings.det_sound_warning_only || rad_back >= settings.rad_th1) tone(4, BUZZ_FREQ, BUZZ_TIME);
    }
}
#pragma endregion
void power_manager() {

    if (rf_timer < millis()) RF_OFF;
    if (timers.bat_check >= BAT_CHECK_TIME) {
        bat_check();
        flags.statusbar_upd = 1;
        timers.bat_check = 0;
    }
    if (settings.bl_time && timers.backlight >= settings.bl_time) LIGHT_OFF;

    if (flags.sound) {
        switch (pointers.sound)
        {
        case SOUND_BATTERY:
            play_sound(bat_low_sound, SAMPLES_BAT_LOW, SOUND_BATTERY);
            break;
        case SOUND_ERROR:
            play_sound(error_sound, SAMPLES_ERROR, SOUND_ERROR);
            break;
        case SOUND_MEASURE:
            play_sound(measur_sound, SAMPLES_MEASUR, SOUND_BATTERY);
            break;
        case SOUND_ALARM:
            play_sound(alarm_sound, SAMPLES_ALARM, SOUND_ALARM);
            break;
        case SOUND_WARN:
            play_sound(warn_sound, SAMPLES_WARN, SOUND_WARN);
            break;
        }
    }
}
void poweroff() {
    //dose_update(0); ##TODO##
    timers.dose_save = 0;

    LIGHT_OFF;
    enableSleepLCD();
    bitClear(TIMSK1, OCIE1A);
    EIMSK = 0b00000000; //запрещаем внешнее прерывание INT0 и INT1
    PCICR = 0b00000100; // разрешаем внешнее прерывание PCINT2
    BUZZ_OFF;
    RF_OFF;
    while (1)
    {
        PRR |= lowByte(_BV(0)); //выключаем питание АЦП
        SMCR = (0x2 << 1) | (1 << SE);  //устанавливаем режим сна powerdown

        MCUCR = (0x03 << 5); //выкл bod
        MCUCR = (0x02 << 5);

        asm("sleep");  //с этого момента спим.

        _delay_ms(2000); //ждем 2 секунды
        if (!OK_READ) { //если кнопка не отжата
            SMCR = 0; // откл сон
            PRR &= ~lowByte(_BV(0)); //включаем питание АЦП
            if (VCC_read() < BAT_MIN)
                break;
            else {
                RF_ON;
                flags.sound = 1;
                while (flags.sound) play_sound(bat_low_sound, SAMPLES_BAT_LOW, SOUND_BATTERY);
                RF_OFF;
            }
        }
    }

    disableSleepLCD(settings.lcd_contrast); //включаем дисплей
    bitSet(TIMSK1, OCIE1A);
    initDevice();
}
void data_manager() {
    if (timers.dose_save >= settings.dose_save_time * 60)
    {
        dose_update(0);
        timers.dose_save = 0;
    }

    if (det_dbg.time == det_dbg_timeout) {
        flags.det_dbg = 0;
        if (pointers.cur_scr == DET_DEBUG_SCR) flags.selected = 1;
    }

    if (flags.data) {
        uint8_t position = imp_buff_pos == 0 ? IMP_BUFF_LENGTH - 1 : imp_buff_pos - 1;
        if (settings.dead_time_on && imp_buff[position] >= (settings.det_dead_time_th * det_count))
            imp_buff[position] = imp_buff[position] + imp_buff[position] * (1 - imp_buff[position] * settings.det_dead_time * 0.000001);

        avg_buff += imp_buff[position];
        if (++avg_period == IMP_BUFF_LENGTH) {
            uint8_t position2 = imp_buff_pos == (IMP_BUFF_LENGTH - 1) ? 0 : imp_buff_pos + 1;
            avg_buff -= imp_buff[position2];
            avg_period--;
        }

        uint32_t abb = avg_buff * 1000UL;
        uint32_t doi = (uint32_t)avg_period * det_own;

        rad_back = settings.own_on ? abb > doi ? abb - doi : 0 : abb;

        float old_accur = accur;
        accur = 1 / sqrt(rad_back*0.001);

        rad_back = (((uint32_t)rad_back * settings.count_time) / (avg_period * det_count) + 500) * 0.001;

        if (rad_back > 9999499) rad_back = 9999499;
        if (rad_back > rad_max && avg_period > 10) rad_max = rad_back;

        imp_dose += imp_buff[position];
        dose_time++;
        if (imp_dose > 0)
            dose = settings.own_on ? (imp_dose * 1000UL - (uint32_t)dose_time*det_own)*settings.count_time / 3600000UL : (uint32_t)imp_dose*settings.count_time / 3600UL;

        if (flags.measure) {
            measure.time++;
            measure.imp += imp_buff[position];
            abb = measure.imp * 1000UL;
            doi = (uint32_t)measure.time * det_own;
            measure.rad = settings.own_on ? abb > doi ? abb - doi : 0 : abb;
            measure.accur =(100 * settings.sigma / sqrt(measure.rad*0.001)) + 0.5;
            if (!measure.accur) measure.accur = 255;
            measure.rad = (((uint32_t)measure.rad * settings.count_time) / (measure.time * det_count) + 500) * 0.001;
            switch (settings.measure_time)
            {
            case 0:
                if (measure.accur <= settings.measure_th) {
                    flags.measure = 0;
                    if (settings.measure_sound) {
                        flags.sound = 1;
                        pointers.sound = SOUND_MEASURE;
                    }
                }
                break;
            case 1:
                if ((measure.time / 60) >= settings.measure_th) {
                    flags.measure = 0;
                    if (settings.measure_sound) {
                        flags.sound = 1;
                        pointers.sound = SOUND_MEASURE;
                    }
                }
                break;
            }
        }

        flags.data = 0;

        uint32_t big_rad = rad_back * 1000UL;
        if (avg_avg != 0) {
            uint32_t pos_th = avg_avg * old_accur * 3;
            uint32_t neg_th = avg_avg > pos_th ? avg_avg - pos_th : 0;
            pos_th += avg_avg;
            if (big_rad > pos_th || big_rad < neg_th) {
                avg_period = 1;
                avg_buff = imp_buff[position];
                avg_avg = 0;
                return;
            }
            float koef = (avg_period > 99) ? 0.01 : 1.00 / avg_period;
            avg_avg = big_rad * koef + avg_avg * (1 - koef);
        }
        else
            avg_avg = big_rad;
    }
}
void error_manager() {
    if (settings.det_1_on && det_1 > ERROR_DET_TIME && !(error & ERR_NOIMP_D1))
        error += ERR_NOIMP_D1;
    if (settings.det_2_on && det_2 > ERROR_DET_TIME && !(error & ERR_NOIMP_D1))
        error += ERR_NOIMP_D2;
    if (settings.det_3_on && det_3 > ERROR_DET_TIME && !(error & ERR_NOIMP_D1))
        error += ERR_NOIMP_D3;
    if (settings.det_4_on && det_4 > ERROR_DET_TIME && !(error & ERR_NOIMP_D1))
        error += ERR_NOIMP_D4;
    if (error && timers.error >= ERROR_REPEAT_TIME) {
        if (pointers.cur_scr != ERROR_SCR) {
            pointers.prev_scr = pointers.cur_scr;
            pointers.cur_scr = ERROR_SCR;
            pointers.prev_pointer = NO_SCR;
            flags.screen = 1;
            if (settings.errors_sound) {
                flags.sound = 1;
                if(error < ERR_LOW_BAT) pointers.sound = SOUND_ERROR;
                else  pointers.sound = SOUND_BATTERY;
            }
        }
    }
}
void alarm_manager() {
    if (settings.dose_alarm_on) {
        if ((dose >= settings.dose_th1 && !flags.dose_alarm1_off) || (dose >= settings.dose_th2 && !flags.dose_alarm2_off)) {
            if (pointers.cur_scr != ALARM_SCR) {
                pointers.prev_pointer = pointers.pointer;
                pointers.prev_scr = pointers.cur_scr;
                pointers.cur_scr = ALARM_SCR;
                flags.screen = 1;
            }
            flags.sound = 1;
            if (dose >= settings.dose_th2) {
                pointers.pointer = ALARM_DOSE2;
                pointers.sound = SOUND_ALARM;
                //flags.play_alarm_sound = 1;
            }
            else {
                pointers.pointer = ALARM_DOSE1;
                pointers.sound = SOUND_WARN;
                //flags.play_warn_sound = 1;
            }
        }
    }
    if (settings.rad_alarm_on) {
        if ((rad_back >= settings.rad_th1 && !flags.rad_alarm1_off) || (rad_back >= settings.rad_th2 && !flags.rad_alarm2_off)) {
            if (pointers.cur_scr != ALARM_SCR) {
                pointers.prev_pointer = pointers.pointer;
                pointers.prev_scr = pointers.cur_scr;
                pointers.cur_scr = ALARM_SCR;
                flags.screen = 1;
            }
            flags.sound = 1;
            if (rad_back >= settings.rad_th2) {
                pointers.pointer = ALARM_RAD2;
                pointers.sound = SOUND_ALARM;
                //flags.play_alarm_sound = 1;
            }
            else {
                pointers.pointer = ALARM_RAD1;
                pointers.sound = SOUND_WARN;
                //flags.play_warn_sound = 1;
            }
        }
    }

    if (settings.alarm_autooff) {
        if (pointers.cur_scr == ALARM_SCR) {
            switch (pointers.pointer)
            {
            case ALARM_RAD1:
                if (rad_back < settings.rad_th1) {
                    pointers.cur_scr = pointers.prev_scr;
                    pointers.pointer = pointers.prev_pointer;
                    pointers.prev_pointer = NO_SCR;
                    flags.screen = 1;
                    flags.statusbar_upd = 1;
                    flags.sound = 0;
                    //flags.play_warn_sound = 0;
                }
                break;
            case ALARM_RAD2:
                if (rad_back < settings.rad_th2) {
                    if (flags.rad_alarm1_off) {
                        pointers.cur_scr = pointers.prev_scr;
                        pointers.pointer = pointers.prev_pointer;
                        pointers.prev_pointer = NO_SCR;
                        flags.sound = 0;
                    }
                    else pointers.pointer = ALARM_RAD1;
                    flags.screen = 1;
                    flags.statusbar_upd = 1;
                    //flags.play_alarm_sound = 0;
                }
                break;
            }
        }
    }

    if (flags.rad_alarm1_off && rad_back < settings.rad_th1) {
        flags.rad_alarm1_off = 0;
        flags.statusbar_upd = 1;
    }
    if (flags.rad_alarm2_off && rad_back < settings.rad_th2) {
        flags.rad_alarm2_off = 0;
        flags.statusbar_upd = 1;
    }
    if (flags.dose_alarm1_off && dose < settings.dose_th1) {
        flags.dose_alarm1_off = 0;
        flags.statusbar_upd = 1;
    }
    if (flags.dose_alarm2_off && dose < settings.dose_th2) {
        flags.dose_alarm2_off = 0;
        flags.statusbar_upd = 1;
    }
}
#pragma region BUTTONS
void button_manager() {
    uint8_t key_check = check_keys();

    if (flags.btn_skip)
        return;

    if (key_check) {
        flags.screen = 1;

        switch (pointers.cur_scr)
        {
        case ERROR_SCR:
            error_button(&key_check);
            break;
        case ALARM_SCR:
            error_button(&key_check);
            break;
        case CONFIRMATION_SCR:
            confirmation_button(&key_check);
            break;
        case FAST_MENU_SCR:
            fast_menu_button(&key_check);
            break;
        default:
            switch (key_check)
            {
            case 1://ok clicked
                if (pointers.cur_scr == MEASURE_SCR) {
                    if (pointers.pointer < MEASURE_RESULT && !flags.measure) flags.measure = 1;
                }
                else if (pointers.cur_scr > SETT_MAIN_SCR) {
                    flags.selected = !flags.selected;
                    if (pointers.cur_scr == DET_DEBUG_SCR) flags.det_dbg = !flags.det_dbg;
                }
                else if (pointers.cur_scr == SETT_MAIN_SCR)
                {
                    pointers.settscr_pointer = pointers.pointer;
                    pointers.pointer = 0;
                    switch (pointers.settscr_pointer)
                    {
                    case 0:
                        pointers.cur_scr = SETT_GEMERAL_SCR;
                        break;
                    case 1:
                        pointers.cur_scr = SETT_ALARM_SCR;
                        break;
                    case 2:
                        pointers.cur_scr = SETT_SOUND_SCR;
                        break;
                    case 3:
                        pointers.cur_scr = SETT_DET_SCR;
                        break;
                    case 4:
                        pointers.cur_scr = DEBUG_SCR;
                        pointers.prev_pointer = NO_SCR;
                        flags.statusbar_upd = 1;
                        clrScrLCD();
                        break;
                    case 5:
                        pointers.cur_scr = DET_DEBUG_SCR;
                        pointers.prev_pointer = NO_SCR;
                        flags.selected = !flags.det_dbg;
                        flags.statusbar_upd = 1;
                        break;
                    }
                }
                break;
            case 2://down clicked
                if (pointers.cur_scr == MEASURE_SCR) {
                    if (pointers.pointer < MEASURE_RESULT && flags.measure) flags.measure = 0;
                }
                else if (!flags.selected) {
                    pointers.prev_pointer = pointers.pointer++;
                    pointerConstrain();
                }
                else
                    settings_value_down();
                break;
            case 3://up clicked
                if (pointers.cur_scr == MEASURE_SCR)
                {
                    pointers.mainscr_pointer = pointers.pointer;
                    pointers.prev2_scr = pointers.cur_scr;
                    pointers.pointer = CONFIRM_MEASURE_NEXT;
                    pointers.cur_scr = CONFIRMATION_SCR;
                    pointers.prev_pointer = NO_SCR;
                    flags.confirmation = 0;
                }
                else if (!flags.selected) {
                    pointers.prev_pointer = pointers.pointer--;
                    pointerConstrain();
                }
                else
                    settings_value_up();

                break;
            case 4://ok holding
                if (pointers.cur_scr == MEASURE_SCR)
                {
                    pointers.mainscr_pointer = pointers.pointer;
                    pointers.prev2_scr = pointers.cur_scr;
                    pointers.pointer = CONFIRM_MEASURE_EXIT;
                    pointers.cur_scr = CONFIRMATION_SCR;
                    pointers.prev_pointer = NO_SCR;
                    flags.confirmation = 0;
                }
                else if (pointers.cur_scr < SETT_MAIN_SCR)
                {
                    pointers.mainscr_pointer = pointers.pointer;
                    pointers.pointer = 0;
                    pointers.prev2_scr = pointers.cur_scr;
                    pointers.cur_scr = SETT_MAIN_SCR;
                    flags.statusbar_upd = 1;
                }
                else if (pointers.cur_scr > SETT_MAIN_SCR) {
                    if (pointers.cur_scr >= DEBUG_SCR)
                        flags.statusbar_upd = 1;
                    pointers.pointer = pointers.settscr_pointer;
                    pointers.cur_scr = SETT_MAIN_SCR;
                    flags.selected = 0;
                }
                else
                    settings_check();
                flags.btn_skip = 1;
                break;
            case 5://down holding
                if (!flags.selected) {
                    flags.btn_skip = 1;

                    switch (pointers.cur_scr)
                    {
                    case MAIN_SCR:
                        uint8_t position;
                        switch (pointers.pointer)
                        {
                        case MONITORING_SCR:
                            position = imp_buff_pos == 0 ? IMP_BUFF_LENGTH - 1 : imp_buff_pos - 1;
                            avg_period = 1;
                            avg_buff = imp_buff[position];
                            avg_avg = 0;
                            rad_max = 0;
                            break;

                        case DOSE_SCR:
                            pointers.mainscr_pointer = pointers.pointer;
                            pointers.prev2_scr = pointers.cur_scr;
                            pointers.pointer = flags.dose_alltime ? CONFIRM_ALLTIME_DOSE : CONFIRM_DOSE;
                            pointers.cur_scr = CONFIRMATION_SCR;
                            pointers.prev_pointer = NO_SCR;
                            flags.confirmation = 0;
                            break;
                        }
                        break;
                    case MEASURE_SCR:
                        pointers.mainscr_pointer = pointers.pointer;
                        pointers.prev2_scr = pointers.cur_scr;
                        pointers.pointer = CONFIRM_MEASURE_RESET;
                        pointers.cur_scr = CONFIRMATION_SCR;
                        pointers.prev_pointer = NO_SCR;
                        flags.confirmation = 0;
                        break;
                    case DET_DEBUG_SCR:
                        det_dbg.d1 = 0;
                        det_dbg.d2 = 0;
                        det_dbg.d3 = 0;
                        det_dbg.d4 = 0;
                        det_dbg.time = 0;
                        break;
                    }
                }
                else
                    settings_value_down();
                break;
            case 6://up holding
                if (!flags.selected) {
                    pointers.prev_scr = pointers.cur_scr;
                    pointers.cur_scr = FAST_MENU_SCR;
                    flags.statusbar_upd = 1;
                    flags.btn_skip = 1;
                }
                else
                    settings_value_up();
                break;
            }
            break;
        }
        if (pointers.cur_scr == MAIN_SCR) {
            if (pointers.pointer != pointers.prev_pointer) flags.statusbar_upd = 1;
            if (pointers.pointer == DOSE_SCR) {
                if (pointers.pointer != pointers.prev_pointer) timers.dose_scr = DOSE_SCR_UPD_TIME + 1;
                else if (key_check == 1) {
                    flags.dose_alltime = !flags.dose_alltime;
                    timers.dose_scr = DOSE_SCR_UPD_TIME + 1;
                    pointers.prev_pointer = NO_SCR;
                }
            }
            else timers.dose_scr = 0;
        }
    }
}
void fast_menu_button(uint8_t *button) {
    flags.statusbar_upd = 1;
    switch (*button)
    {
    case 2:
        LIGHT_READ ? LIGHT_ON : LIGHT_OFF;
        flags.bl_off = LIGHT_READ;
        break;
    case 3:
        if (pointers.prev_scr == MAIN_SCR) pointers.measure_pointer = pointers.pointer;
        pointers.cur_scr = MEASURE_SCR;
        pointers.pointer = 0;
        pointers.prev_pointer = NO_SCR;
        flags.statusbar_upd = 1;
        measure.accur = 255;
        measure.imp = 0;
        measure.rad = 0;
        measure.time = 0;
        flags.measure = 0;
        return;
        break;
    case 4:
        poweroff();
        break;
    case 5:
        settings.rfflash_on = !settings.rfflash_on;
        flags.btn_skip = 1;
        break;
    case 6:
        flags.silent_mode = !flags.silent_mode;
        flags.btn_skip = 1;
        break;
    }
    pointers.cur_scr = pointers.prev_scr;
    pointers.prev_pointer = NO_SCR;
}
void confirmation_button(uint8_t *button) {
    switch (*button)
    {
    case 1:
        switch (pointers.pointer)
        {
        case CONFIRM_SETTINGS:
            if(flags.confirmation) settings_update();
            flags.statusbar_upd = 1;
            break;
        case CONFIRM_DOSE:
            if (flags.confirmation)
            {
                //dose_update(0);
                dose = 0;
                dose_time = 0;
                imp_dose = 0;
                dose_old = 0;
                timers.dose_scr = DOSE_SCR_UPD_TIME + 1;
                timers.dose_save = 0;
            }
            break;
        case CONFIRM_ALLTIME_DOSE:
            if (flags.confirmation)
            {
                dose_update(1);
                timers.dose_scr = DOSE_SCR_UPD_TIME + 1;
            }
            break;
        case CONFIRM_MEASURE_RESET:
            if (flags.confirmation)
            {
                flags.measure = 0;
                pointers.mainscr_pointer = 0;
                measure.accur = 255;
                measure.imp = 0;
                measure.rad = 0;
                measure.time = 0;
            }
            break;
        case CONFIRM_MEASURE_EXIT:
            if (flags.confirmation)
            {
                flags.measure = 0;
                pointers.mainscr_pointer = pointers.measure_pointer;
                pointers.prev2_scr = MAIN_SCR;
                flags.statusbar_upd = 1;
            }
            break;
        case CONFIRM_MEASURE_NEXT:
            if (flags.confirmation)
            {
                flags.measure = 0;
                if (pointers.mainscr_pointer != MEASURE_2) {
                    measure.imp1 = measure.imp;
                    measure.rad1 = measure.rad;
                    measure.accur = 255;
                    measure.imp = 0;
                    measure.rad = 0;
                    measure.time = 0;
                }
                if (pointers.mainscr_pointer < MEASURE_RESULT) pointers.mainscr_pointer++;
                else pointers.mainscr_pointer = 0;
            }
            break;
        }
        pointers.cur_scr = pointers.prev2_scr;
        pointers.pointer = pointers.mainscr_pointer;
        pointers.prev_pointer = NO_SCR;
        break;
    case 2:
        flags.confirmation = !flags.confirmation;
        break;
    case 3:
        flags.confirmation = !flags.confirmation;
        break;
    }
}
void error_button(uint8_t *button) {
    switch (*button)
    {
    case 1:
        switch (pointers.cur_scr)
        {
        case ERROR_SCR:
            error = error >= ERR_LOW_BAT ? error - ERR_LOW_BAT : error >= ERR_OVERLOAD ? error - ERR_OVERLOAD : error >= ERR_SHORT ? error - ERR_SHORT : 0;
            timers.error = 0;
            break;
        case ALARM_SCR:
            flags.dose_alarm1_off = 1;
            if (pointers.pointer == ALARM_RAD2 || pointers.pointer == ALARM_DOSE2)
                flags.dose_alarm2_off = 1;
            pointers.pointer = pointers.prev_pointer;
            flags.sound = 0;
            break;
        }
        pointers.cur_scr = pointers.prev_scr;
        pointers.prev_pointer = NO_SCR;
        flags.statusbar_upd = 1;
        break;
    }
}
void pointerConstrain() {
    uint8_t items;
    switch (pointers.cur_scr)
    {
    case MAIN_SCR:
        items = MAIN_ITEMS - 1;
        break;
    case SETT_MAIN_SCR:
        items = settings.debug_on ? SETT_ITEMS - 1 : SETT_ITEMS - 3;
        break;
    case SETT_GEMERAL_SCR:
        items = GENERAL_ITEMS - 1;
        break;
    case SETT_ALARM_SCR:
        items = ALARM_ITEMS - 1;
        break;
    case SETT_SOUND_SCR:
        items = SOUND_ITEMS - 1;
        break;
    case SETT_DET_SCR:
        items = DET_ITEMS - 1;
        break;
    case DEBUG_SCR:
        items = DEBUG_ITEMS - 1;
        break;
    default:
        items = 0;
        break;
    }
    if (pointers.pointer > items)
        pointers.pointer = pointers.prev_pointer ? 0 : items;
}
void settings_value_down() {
    switch (pointers.cur_scr)
    {
    case SETT_GEMERAL_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.sigma == 1 ? settings.sigma = 3 : settings.sigma--;
            break;
        case 1:
            settings.measure_time = !settings.measure_time;
            break;
        case 2:
            settings.measure_th == 1 ? settings.measure_th = 255 : settings.measure_th--;
            break;
        case 3:
            settings.bl_time--;
            break;
        case 4:
            settings.lcd_contrast--;
            setContrastLCD(settings.lcd_contrast);
            break;
        case 5:
            settings.auto_exit_time == 5 ? settings.auto_exit_time = 0 : settings.auto_exit_time--;
            break;
        case 6:
            settings.dose_save_time == 1 ? settings.dose_save_time = 255 : settings.dose_save_time--;
            break;
        case 7:
            settings.sleep_time == 5 ? settings.sleep_time = 0 : settings.sleep_time--;
            break;
        case 8:
            settings.rf_nosleep = !settings.rf_nosleep;
            break;
        case 9:
            settings.rfflash_on = !settings.rfflash_on;
            break;
        case 10:
            settings.search_lr = !settings.search_lr;
            break;
        case 11:
            settings.nanosv = !settings.nanosv;
            break;
        case 12:
            settings.errors_on = !settings.errors_on;
            break;
        case 13:
            settings.debug_on = !settings.debug_on;
            break;
        }
        break;
    case SETT_ALARM_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.alarm_autooff = !settings.alarm_autooff;
            break;
        case 1:
            settings.rad_alarm_on = !settings.rad_alarm_on;
            break;
        case 2:
            settings.rad_th1--;
            break;
        case 3:
            settings.rad_th2--;
            break;
        case 4:
            settings.dose_alarm_on = !settings.dose_alarm_on;
            break;
        case 5:
            settings.dose_th1--;
            break;
        case 6:
            settings.dose_th2--;
            break;
        }
        flags.statusbar_upd = 1;
        break;
    case SETT_SOUND_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.button_sound = !settings.button_sound;
            break;
        case 1:
            settings.det_sound = !settings.det_sound;
            flags.statusbar_upd = 1;
            break;
        case 2:
            settings.det_sound_warning_only = !settings.det_sound_warning_only;
            break;
        case 3:
            settings.measure_sound = !settings.measure_sound;
            break;
        case 4:
            settings.errors_sound = !settings.errors_sound;
            break;
        }
        break;
    case SETT_DET_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.det_1_on = !settings.det_1_on;
            detector_enable();
            break;
        case 1:
            settings.det_2_on = !settings.det_2_on;
            detector_enable();
            break;
        case 2:
            settings.det_3_on = !settings.det_3_on;
            detector_enable();
            break;
        case 3:
            settings.det_4_on = !settings.det_4_on;
            detector_enable();
            break;
        case 4:
            settings.own_on = !settings.own_on;
            break;
        case 5:
            settings.det_1_own--;
            detector_enable();
            break;
        case 6:
            settings.det_2_own--;
            detector_enable();
            break;
        case 7:
            settings.det_3_own--;
            detector_enable();
            break;
        case 8:
            settings.det_4_own--;
            detector_enable();
            break;
        case 9:
            settings.dead_time_on = !settings.dead_time_on;
            break;
        case 10:
            settings.det_dead_time == 1 ? settings.det_dead_time = 255 : settings.det_dead_time--;
            break;
        case 11:
            settings.det_dead_time_th == 1 ? settings.det_dead_time_th = 255 : settings.det_dead_time_th--;
            break;
        case 12:
            settings.count_time == 1 ? settings.count_time = 255 : settings.count_time--;
            break;
        case 13:
            settings.high_count_det_off = !settings.high_count_det_off;
            break;
        }
        break;
    case DEBUG_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.opornoe--;
            break;
        case 1:
            settings.puls--;
            break;
        case 2:
            settings.k_delitel--;
            break;
        case 3:
            settings.ADC_value--;
            break;
        }
        break;
    case DET_DEBUG_SCR:
        det_dbg_timeout == 0 ? det_dbg_timeout = 9999 : det_dbg_timeout--;
    }
}
void settings_value_up() {
    switch (pointers.cur_scr)
    {
    case SETT_GEMERAL_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.sigma == 3 ? settings.sigma = 1 : settings.sigma++;
            break;
        case 1:
            settings.measure_time = !settings.measure_time;
            break;
        case 2:
            settings.measure_th == 255 ? settings.measure_th = 1 : settings.measure_th++;
            break;
        case 3:
            settings.bl_time++;
            break;
        case 4:
            settings.lcd_contrast++;
            setContrastLCD(settings.lcd_contrast);
            break;
        case 5:
            settings.auto_exit_time == 0 ? settings.auto_exit_time = 5 : settings.auto_exit_time++;
            break;
        case 6:
            settings.dose_save_time == 255 ? settings.dose_save_time = 1 : settings.dose_save_time++;
            break;
        case 7:
            settings.sleep_time == 0 ? settings.sleep_time = 5 : settings.sleep_time++;
            break;
        case 8:
            settings.rf_nosleep = !settings.rf_nosleep;
            break;
        case 9:
            settings.rfflash_on = !settings.rfflash_on;
            break;
        case 10:
            settings.search_lr = !settings.search_lr;
            break;
        case 11:
            settings.nanosv = !settings.nanosv;
            break;
        case 12:
            settings.errors_on = !settings.errors_on;
            break;
        case 13:
            settings.debug_on = !settings.debug_on;
            break;
        }
        break;
    case SETT_ALARM_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.alarm_autooff = !settings.alarm_autooff;
            break;
        case 1:
            settings.rad_alarm_on = !settings.rad_alarm_on;
            break;
        case 2:
            settings.rad_th1++;
            break;
        case 3:
            settings.rad_th2++;
            break;
        case 4:
            settings.dose_alarm_on = !settings.dose_alarm_on;
            break;
        case 5:
            settings.dose_th1++;
            break;
        case 6:
            settings.dose_th2++;
            break;
        }
        flags.statusbar_upd = 1;
        break;
    case SETT_SOUND_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.button_sound = !settings.button_sound;
            break;
        case 1:
            settings.det_sound = !settings.det_sound;
            flags.statusbar_upd = 1;
            break;
        case 2:
            settings.det_sound_warning_only = !settings.det_sound_warning_only;
            break;
        case 3:
            settings.measure_sound = !settings.measure_sound;
            break;
        case 4:
            settings.errors_sound = !settings.errors_sound;
            break;
        }
        break;
    case SETT_DET_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.det_1_on = !settings.det_1_on;
            detector_enable();
            break;
        case 1:
            settings.det_2_on = !settings.det_2_on;
            detector_enable();
            break;
        case 2:
            settings.det_3_on = !settings.det_3_on;
            detector_enable();
            break;
        case 3:
            settings.det_4_on = !settings.det_4_on;
            detector_enable();
            break;
        case 4:
            settings.own_on = !settings.own_on;
            break;
        case 5:
            settings.det_1_own++;
            detector_enable();
            break;
        case 6:
            settings.det_2_own++;
            detector_enable();
            break;
        case 7:
            settings.det_3_own++;
            detector_enable();
            break;
        case 8:
            settings.det_4_own++;
            detector_enable();
            break;
        case 9:
            settings.dead_time_on = !settings.dead_time_on;
            break;
        case 10:
            settings.det_dead_time == 255 ? settings.det_dead_time = 1 : settings.det_dead_time++;
            break;
        case 11:
            settings.det_dead_time_th == 255 ? settings.det_dead_time_th = 1 : settings.det_dead_time_th++;
            break;
        case 12:
            settings.count_time == 255 ? settings.count_time = 1 : settings.count_time++;
            break;
        case 13:
            settings.high_count_det_off = !settings.high_count_det_off;
            break;
        }
        break;
    case DEBUG_SCR:
        switch (pointers.pointer)
        {
        case 0:
            settings.opornoe++;
            break;
        case 1:
            settings.puls++;
            break;
        case 2:
            settings.k_delitel++;
            break;
        case 3:
            settings.ADC_value++;
            break;
        }
        break;
    case DET_DEBUG_SCR:
        det_dbg_timeout == 9999 ? det_dbg_timeout = 0 : det_dbg_timeout++;
    }
}
uint8_t check_keys() //проверить клавиатуру
{
    static uint8_t btn_set; //флаг признака действия
    static uint8_t btn_switch; //флаг мультиопроса кнопок

    switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
    case 0:
        if (!OK_READ) { //если нажата кл. ок
            btn_switch = 1; //выбираем клавишу опроса
            flags.btn_state = 0; //обновляем текущее состояние кнопки
        }
        else if (!DOWN_READ) { //если нажата кл. вниз
            btn_switch = 2; //выбираем клавишу опроса
            flags.btn_state = 0; //обновляем текущее состояние кнопки
        }
        else if (!UP_READ) { //если нажата кл. вверх
            btn_switch = 3; //выбираем клавишу опроса
            flags.btn_state = 0; //обновляем текущее состояние кнопки
        }
        else flags.btn_state = 1; //обновляем текущее состояние кнопки
        break;
    case 1: flags.btn_state = OK_READ; break; //опрашиваем клавишу ок
    case 2: flags.btn_state = DOWN_READ; break; //опрашиваем клавишу вниз
    case 3: flags.btn_state = UP_READ; break; //опрашиваем клавишу вверх
    }

    switch (flags.btn_state)
    {
    case 0:
        switch (flags.btn_check)
        {
        case 0: //кнопка нажата
            if (btn_tmr < millis()) {
                flags.btn_check = 1;
                btn_tmr = millis() + BTN_HOLD_MS;
                if (!flags.bl_off) LIGHT_ON;
                timers.auto_exit = 0;
                timers.backlight = 0;
            }
            break;
        case 1: //кнопка удерживается
            if (btn_tmr < millis()) btn_set = 2;
            break;
        }
        break;
    case 1:
        switch (flags.btn_check)
        {
        case 0:
            if (!btn_set) btn_switch = 0; //сбрасываем мультиопрос кнопок
            break;
        case 1: //кнопка отпущенна
            if (btn_tmr > millis()) btn_set = 1;
            flags.btn_check = 0;
            flags.btn_skip = 0;
            btn_tmr = millis() + BTN_DEBOUNCE_MS;
            if (settings.button_sound && !flags.silent_mode) tone(4, FREQ_BEEP, TIME_BEEP); //щелчок пищалкой
            break;
        }
        break;
    }

    switch (btn_set) { //переключаемся в зависимости от признака нажатия
    case 0: return 0; //клавиша не нажата, возвращаем 0
    case 1:
        btn_set = 0; //сбрасываем признак нажатия
        switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
        case 1: return 1; //ok press, возвращаем 1
        case 2: return 2; //down press, возвращаем 2
        case 3: return 3; //up press, возвращаем 3
        }
        break;

    case 2:
        btn_set = 0; //сбрасываем признак нажатия
        switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
        case 1: return 4; //ok hold, возвращаем 4
        case 2: return 5; //down hold, возвращаем 5
        case 3: return 6; //up hold, возвращаем 6
        }
        break;
    }
}
#pragma endregion
#pragma region SCREEN
void screen_manager() {

    if (settings.auto_exit_time && pointers.cur_scr > MEASURE_SCR && timers.auto_exit >= settings.auto_exit_time) {
        if (pointers.cur_scr == FAST_MENU_SCR) pointers.cur_scr = pointers.prev_scr;
        else {
            pointers.cur_scr = pointers.prev2_scr;
            pointers.pointer = pointers.mainscr_pointer;
        }
        pointers.prev_pointer = NO_SCR;
        flags.statusbar_upd = 1;
    }

    if (flags.statusbar_upd) statusbar();
    if (flags.screen) {
        switch (pointers.cur_scr)
        {
        case MAIN_SCR:
            switch (pointers.pointer)
            {
            case MONITORING_SCR:
                monitoring_scr();
                break;
            case SEARCH_SCR:
                search_scr();
                break;
            case DOSE_SCR:
                if (timers.dose_scr > DOSE_SCR_UPD_TIME) {
                    timers.dose_scr = 1;
                    dose_scr();
                }
                break;
            }
            break;
        case MEASURE_SCR:
            measur_scr();
            break;
        case FAST_MENU_SCR:
            fast_menu_scr();
            break;
        case SETT_MAIN_SCR:
            settings_main_scr();
            break;
        case DEBUG_SCR:
            debug_scr();
            break;
        case DET_DEBUG_SCR:
            det_debug_scr();
            break;
        case CONFIRMATION_SCR:
            confirmation_scr();
            break;
        case ERROR_SCR:
            error_scr();
            break;
        case ALARM_SCR:
            alarm_scr();
            break;
        default:
            settings_scr();
            break;
        }
        flags.screen = 0;
    }
}
void statusbar() {
    drawBitmapLCD(0, 0, statusbar_bg, 84, 8); //устанавлваем фон
    drawBitmapLCD(70, 0, bat_alt_img, bat_level * 2, 8); //отображаем состояние батареи

    if (flags.silent_mode)
        drawBitmapLCD(53, 0, cross_img, 5, 8);
    else if (settings.det_sound)
        drawBitmapLCD(53, 0, sound_img, 5, 8);

    if (!settings.rad_alarm_on && !settings.dose_alarm_on)
        drawBitmapLCD(64, 0, cross_img, 5, 8);
    else {
        if (settings.rad_alarm_on) {
            if (rad_back > settings.rad_th2) {
                drawBitmapLCD(64, 0, excl_img, 1, 8);
                drawBitmapLCD(68, 0, excl_img, 1, 8);
            }
            else if (rad_back > settings.rad_th1)
                drawBitmapLCD(64, 0, excl_img, 1, 8);
            else
                drawBitmapLCD(64, 0, alarm_img, 1, 8);
        }
        if (settings.dose_alarm_on) {
            if (dose > settings.dose_th2) {
                drawBitmapLCD(66, 0, excl_img, 1, 8);
                drawBitmapLCD(68, 0, excl_img, 1, 8);
            }
            else if (dose > settings.dose_th1)
                drawBitmapLCD(66, 0, excl_img, 1, 8);
            else
                drawBitmapLCD(66, 0, alarm_img, 1, 8);
        }
    }

    switch (pointers.cur_scr)
    {
    case MAIN_SCR:
        switch (pointers.pointer)
        {
        case MONITORING_SCR:
            drawBitmapLCD(2, 0, backgr_img, 15, 8);
            break;  //режим текущего фона
        case SEARCH_SCR:
            drawBitmapLCD(2, 0, serch_img, 24, 8);
            break;  //режим накопленной дозы
        case DOSE_SCR:
            drawBitmapLCD(2, 0, dose_img, 20, 8);
            break;  //режим поиска
        }
        break;
    case MEASURE_SCR:
        drawBitmapLCD(2, 0, measur_img, 25, 8);
        break;
    case FAST_MENU_SCR:
        drawBitmapLCD(2, 0, fast_menu_img, 21, 8);
        break;
    case SETT_MAIN_SCR:
        drawBitmapLCD(2, 0, settings_img, 45, 8);
        break;
    case DEBUG_SCR:
        drawBitmapLCD(2, 0, debug_img, 36, 8);
        break;
    case DET_DEBUG_SCR:
        drawBitmapLCD(2, 0, debug_img, 36, 8);
        break;
    case CONFIRMATION_SCR:
        switch (pointers.pointer)
        {
        case CONFIRM_SETTINGS:
            drawBitmapLCD(2, 0, settings_img, 45, 8);
            break;  //режим текущего фона
        case CONFIRM_DOSE:
            drawBitmapLCD(2, 0, dose_img, 20, 8);
            break;  //режим накопленной дозы
        case CONFIRM_ALLTIME_DOSE:
            drawBitmapLCD(2, 0, dose_img, 20, 8);
            break;  //режим поиска
        default:
            drawBitmapLCD(2, 0, measur_img, 25, 8);
            break;
        }
        break;
    default:
        drawBitmapLCD(2, 0, settings_img, 45, 8);
        break;
    }

    flags.statusbar_upd = 0;
}
void monitoring_scr() {
    static uint8_t avg_scale;

    char str[10];
    uint32_t result;

    if (pointers.prev_pointer != MONITORING_SCR) {
        clrRowLCD(1);
        clrRowLCD(2);
        clrRowLCD(3);
        clrRowLCD(4);
        printLCD("NJXYJCNM&", LEFT, 32); //ТОЧНОСТЬ:
        printLCD("VFRC.&", LEFT, 40); //МАКС.:
        avg_scale = 0;
        pointers.prev_pointer = MONITORING_SCR;
    }

    if (avg_scale > avg_period) {
        clrRowLCD(3); //очистка строки 4
        avg_scale = avg_period;
    }
    while (((avg_scale << 1) + (avg_scale >> 1)) < avg_period)
    {
        drawBitmapLCD((avg_scale + 2), 24, l1, 1, 6);
        avg_scale++;
    }

    uint8_t ad = accur * 100 * settings.sigma + 0.5;
    formatAndPrint("%3u%%", ad, 10, RIGHT, 32);


    switch (settings.nanosv)
    {
    case 0:
        if (rad_max < 1000) {
            formatAndPrint("%3uvrH|x", rad_max, 10, RIGHT, 40);
        }
        else {
            result = (rad_max + 500) * 0.001;
            formatAndPrint("%4uvH|x", result, 10, RIGHT, 40);
        }

        if (rad_back < 10000) {
            snprintf(str, 10, "%4u", rad_back);
            printLCD(" vrH|x", RIGHT, 16);
        }
        else {
            result = (rad_back + 500) * 0.001;
            snprintf(str, 10, "%4u", result);
            printLCD(" vH|x ", RIGHT, 16);
        }
        break;
    case 1:
        if (rad_max < 1000) {
            result = rad_max * 10;
            formatAndPrint("%4uyPd|x", result, 10, RIGHT, 40);
        }
        else if (rad_max < 99950){
            result = (rad_max + 50) * 0.01;
            formatAndPrint("%3uvrPd|x", result, 10, RIGHT, 40);
        }
        else {
            result = (rad_max + 50000) * 0.00001;
            formatAndPrint("%4uvPd|x", result, 10, RIGHT, 40);
        }

        if (rad_back < 1000) {
            result = rad_back * 10;
            snprintf(str, 10, "%4u", result);
            printLCD(" yPd|x", RIGHT, 16);
        }
        else if (rad_back < 999950) {
            result = (rad_back + 50) * 0.01;
            snprintf(str, 10, "%4u", result);
            printLCD("vrPd|x", RIGHT, 16);
        }
        else {
            result = (rad_back + 50000) * 0.00001;
            snprintf(str, 10, "%4u", result);
            printLCD(" vPd|x", RIGHT, 16);
        }
        break;
    }
    setFontLCD(MediumNumbers);
    printLCD(str, 1, 8);
    setFontLCD(RusFont);
}
void search_scr() {
    pointers.prev_pointer = SEARCH_SCR;
    
    uint8_t position = imp_buff_pos == 0 ? IMP_BUFF_LENGTH - 1 : imp_buff_pos - 1;
    formatAndPrint("%9ubvg|c", imp_buff[position], 15, RIGHT, 8);

    uint8_t n;
    uint8_t x;
    for (uint8_t i = 0; i < 82; i++) {
        n = 1 + i;
        position = imp_buff_pos < n ? IMP_BUFF_LENGTH - n + imp_buff_pos : imp_buff_pos - n;
        x = settings.search_lr ? 0 + i : 82 - i;
        imp_buff[position] < 32 ? drawBitmapLCD(x, 16, search_lines[imp_buff[position]], 1, 32) : drawBitmapLCD(x, 16, search_lines[31], 1, 32);
    }

    drawBitmapLCD(0, 16, search_bg_left, 2, 32);
    drawBitmapLCD(82, 16, search_bg_right, 2, 32);
}
void dose_scr() {
    char str[13];
    uint32_t result;

    if (pointers.prev_pointer != DOSE_SCR) {
        clrRowLCD(1);
        clrRowLCD(2);
        clrRowLCD(3);
        clrRowLCD(4);
        pointers.prev_pointer = DOSE_SCR;
    }

    uint32_t dose_alltime = eeprom_read_dword(dose_curr_offset);
    uint8_t len = map(timers.dose_save, 0, settings.dose_save_time * 60, 2, 83);

    switch (flags.dose_alltime)
    {
    case 0:

        clrRowLCD(3);
        for (uint8_t i = 2; i < len; i++)
            drawBitmapLCD(i, 24, l1, 1, 6);

        doseTimePrint(&dose_time, 32);

        printLCD("DCTUJ&", LEFT, 40); //ВСЕГО:

        switch (settings.nanosv)
        {
        case 0:
            if (dose_alltime < 100000) {
                formatAndPrint("%5uvrH", dose_alltime, 13, RIGHT, 40);
            }
            else {
                result = (dose_alltime + 500)*0.001;
                formatAndPrint("%6uvH", result, 13, RIGHT, 40);
            }

            if (dose < 100000) {
                snprintf(str, 13, "%5u", dose);
                printLCD(" vrH", RIGHT, 16);
            }
            else {
                result = (dose + 500)*0.001;
                snprintf(str, 13, "%5u", result);
                printLCD(" vH ", RIGHT, 16);
            }
            break;
        case 1:
            if (dose_alltime < 10000) {
                result = dose_alltime * 10;
                formatAndPrint("%5uyPd", result, 13, RIGHT, 40);
            }
            else if (dose_alltime < 999950){
                result = (dose_alltime + 50)*0.01;
                formatAndPrint("%4uvrPd", result, 13, RIGHT, 40);
            }
            else {
                result = (dose_alltime + 50000)*0.00001;
                formatAndPrint("%5uvPd", result, 13, RIGHT, 40);
            }

            if (dose < 10000) {
                result = dose * 10;
                snprintf(str, 13, "%5u", result);
                printLCD(" yPd", RIGHT, 16);
            }
            else if (dose < 9999950) {
                result = (dose + 50)*0.01;
                snprintf(str, 13, "%5u", result);
                printLCD("vrPd", RIGHT, 16);
            }
            else {
                result = (dose + 50000)*0.00001;
                snprintf(str, 13, "%5u", result);
                printLCD(" vPd", RIGHT, 16);
            }
            break;
        }
        break;
    case 1:
        uint32_t dose_alltime_time = eeprom_read_dword(dose_curr_offset + 4);
        dose_alltime_time &= ~((uint32_t)dose_mask << 30);
        clrRowLCD(5);
        printLCD("DCTUJ PF&", CENTER, 32); //ВСЕГО ЗА:

        doseTimePrint(&dose_alltime_time, 40);

        switch (settings.nanosv)
        {
        case 0:
            if (dose_alltime < 100000) {
                snprintf(str, 13, "%5u", dose_alltime);
                printLCD(" vrH", RIGHT, 16);
            }
            else {
                result = (dose_alltime + 500)*0.001;
                snprintf(str, 13, "%5u", result);
                printLCD(" vH ", RIGHT, 16);
            }
            break;
        case 1:
            if (dose_alltime < 10000) {
                result = dose_alltime * 10;
                snprintf(str, 13, "%5u", result);
                printLCD(" yPd", RIGHT, 16);
            }
            else if (dose_alltime < 9999950) {
                result = (dose_alltime + 50)*0.01;
                snprintf(str, 13, "%5u", result);
                printLCD("vrPd", RIGHT, 16);
            }
            else {
                result = (dose_alltime + 50000)*0.00001;
                snprintf(str, 13, "%5u", result);
                printLCD(" vPd", RIGHT, 16);
            }
            break;
        }
        timers.dose_scr = 0;
        break;
    }
    setFontLCD(MediumNumbers);
    printLCD(str, 1, 8);
    setFontLCD(RusFont);
}
void doseTimePrint(uint32_t* time, uint8_t y) {
    char str[13];
    uint8_t d, h, m, s;

    d = (*time) / 86400;
    h = ((*time) / 3600) % 24;
    m = ((*time) / 60) % 60;
    s = (*time) % 60;

    snprintf(str, 13, "%2ul %02u&%02u&%02u", d, h, m, s);
    printLCD(str, CENTER, y);
}
void measur_scr() {
    char str[15];
    uint32_t result;

    if (pointers.pointer < MEASURE_RESULT) {
        if (pointers.prev_pointer == NO_SCR) {
            clrRowLCD(1);
            clrRowLCD(2);
            clrRowLCD(4);
            clrRowLCD(5);
            formatAndPrint("#%u", pointers.pointer + 1, 3, RIGHT, 8);
            settings.measure_time ? formatAndPrint("%uvby", settings.measure_th, 7, LEFT, 32) : formatAndPrint("%u%%", settings.measure_th, 4, LEFT, 32);
            pointers.prev_pointer = 0;
        }

        clrRowLCD(3);
        clrRowLCD(5);
        uint8_t bar;
        uint8_t x_a;
        uint8_t x_t;
        if (settings.measure_time) {
            bar = constrain(map(measure.time, 0, settings.measure_th * 60, 2, 83), 2, 83);
            x_a = RIGHT;
            x_t = LEFT;
        }
        else {
            bar = constrain(map(measure.accur, settings.measure_th << 1, settings.measure_th, 2, 83), 2, 83);
            x_a = LEFT;
            x_t = RIGHT;
        }
        uint8_t h = measure.time / 3600;
        uint8_t m = (measure.time / 60) % 60;
        uint8_t s = measure.time % 60;
        snprintf(str, 9, "%02u&%02u&%02u", h, m, s);
        printLCD(str, x_t, 40);
        formatAndPrint("$%u%%", measure.accur, 6, x_a, 40);
        formatAndPrint("%5ubvg", measure.imp, 9, RIGHT, 32);
        for (uint8_t i = 2; i < bar; i++)
            drawBitmapLCD(i, 24, l1, 1, 6);

        switch (settings.nanosv)
        {
        case 0:
            if (measure.rad < 10000) {
                snprintf(str, 10, "%4u", measure.rad);
                printLCD(" vrH|x", RIGHT, 16);
            }
            else {
                result = (measure.rad + 500) * 0.001;
                snprintf(str, 10, "%4u", result);
                printLCD(" vH|x ", RIGHT, 16);
            }
            break;
        case 1:
            if (measure.rad < 1000) {
                result = measure.rad * 10;
                snprintf(str, 10, "%4u", result);
                printLCD(" yPd|x", RIGHT, 16);
            }
            else if (measure.rad < 999950) {
                result = (measure.rad + 50) * 0.01;
                snprintf(str, 10, "%4u", result);
                printLCD("vrPd|x", RIGHT, 16);
            }
            else {
                result = (measure.rad + 50000) * 0.00001;
                snprintf(str, 10, "%4u", result);
                printLCD(" vPd|x", RIGHT, 16);
            }
            break;
        }
        setFontLCD(MediumNumbers);
        printLCD(str, 1, 8);
        setFontLCD(RusFont);
    }
    else if (pointers.prev_pointer == NO_SCR) {
        result = measure.rad1 - measure.rad;
        formatMeasure(result, 8);
        formatMeasure(measure.rad1, 16);
        formatMeasure(measure.rad,32);
        formatAndPrint("%10u bvg", measure.imp1, 15, RIGHT, 24);
        formatAndPrint("%10u bvg", measure.imp, 15, RIGHT, 40);
        printLCD("#1", LEFT, 16);
        printLCD("#2", LEFT, 32);
        pointers.prev_pointer = 0;
    }
}
void formatMeasure(uint32_t value, uint8_t y) {
    switch (settings.nanosv)
    {
    case 0:
        if (value < 10000) {
            formatAndPrint("%8u vrH|x", value, 15, RIGHT, y);
        }
        else {
            value = (value + 500) * 0.001;
            formatAndPrint("%9u vH|x", value, 15, RIGHT, y);
        }
        break;
    case 1:
        if (value < 1000) {
            value = value * 10;
            formatAndPrint("%8u yPd|x", value, 15, RIGHT, y);
        }
        else if (value < 99950) {
            value = (value + 50) * 0.01;
            formatAndPrint("%7u vrPd|x", value, 15, RIGHT, y);
        }
        else {
            value = (value + 50000) * 0.00001;
            formatAndPrint("%8u vPd|x", value, 15, RIGHT, y);
        }
        break;
    }
}
void fast_menu_scr() {
    drawBitmapLCD(0, 8, fast_measur_img, 28, 32);
    drawBitmapLCD(28, 8, fast_power_img, 28, 32);
    drawBitmapLCD(56, 8, fast_light_img, 28, 32);
    drawBitmapLCD(0, 40, fast_down_img, 84, 8);
}
void settings_main_scr() {

    uint8_t y;
    uint8_t p = pointers.pointer;
    uint16_t ptr;
    cfont.inverted = 1;
    for (uint8_t i = 0; i < 6; i++)
    {
        y = (i << 3) + 8;
        ptr = pgm_read_word(&(sett_main_items[pointers.pointer]));
        printMenuItem(ptr, LEFT, y, 0);
        pointers.prev_pointer = pointers.pointer++;
        pointerConstrain();
        cfont.inverted = 0;
    }
    pointers.pointer = p;
}
void settings_scr() {
    uint8_t p = pointers.pointer;
    uint8_t y;
    uint16_t ptr;
    char str[15];

    clrRowLCD(1);
    ptr = pgm_read_word(&(sett_main_items[pointers.settscr_pointer]));
    printMenuItem(ptr, CENTER, 8, 1);

    if (!flags.selected) cfont.inverted = 1;

    for (uint8_t i = 0; i < 2; i++)
    {
        switch (pointers.cur_scr)
        {
        case SETT_GEMERAL_SCR:
            ptr = pgm_read_word(&(sett_gen_items[pointers.pointer]));
            sett_gen_m(str);
            break;
        case SETT_ALARM_SCR:
            ptr = pgm_read_word(&(sett_alarm_items[pointers.pointer]));
            sett_alarm_m(str);
            break;
        case SETT_SOUND_SCR:
            ptr = pgm_read_word(&(sett_sound_items[pointers.pointer]));
            sett_sound_m(str);
            break;
        case SETT_DET_SCR:
            ptr = pgm_read_word(&(sett_det_items[pointers.pointer]));
            sett_det_m(str);
            break;
        }

        y = (i << 4) + 16;
        printMenuItem(ptr, LEFT, y, 0);

        if (flags.selected && !i) cfont.inverted = 1;

        printLCD(str, LEFT, y + 8);
        pointers.prev_pointer = pointers.pointer++;
        pointerConstrain();

        cfont.inverted = 0;

    }
    pointers.pointer = p;
}
void sett_gen_m(char* str) {
    switch (pointers.pointer)
    {
    case 0:
        snprintf_P(str, 15, &(sett_value10[0]), settings.sigma);
        break;
    case 1:
        settings.measure_time ? strncpy_P(str, &(sett_value9[0]), 15) : strncpy_P(str, &(sett_value2[0]), 15);
        break;
    case 2:
        settings.measure_time ? snprintf_P(str, 15, &(sett_value11[0]), settings.measure_th) : snprintf_P(str, 15, &(sett_value12[0]), settings.measure_th);
        break;
    case 3:
        settings.bl_time ? (char*)snprintf_P(str, 15, &(sett_value11[0]), settings.bl_time) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 4:
        snprintf_P(str, 15, &(sett_value10[0]), settings.lcd_contrast);
        break;
    case 5:
        settings.auto_exit_time ? (char*)snprintf_P(str, 15, &(sett_value11[0]), settings.auto_exit_time) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 6:
        snprintf_P(str, 15, &(sett_value13[0]), settings.dose_save_time);
        break;
    case 7:
        settings.sleep_time ? (char*)snprintf_P(str, 15, &(sett_value11[0]), settings.sleep_time) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 8:
        settings.rf_nosleep ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 9:
        settings.rfflash_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 10:
        settings.search_lr ? strncpy_P(str, &(sett_value5[0]), 15) : strncpy_P(str, &(sett_value6[0]), 15);
        break;
    case 11:
        settings.nanosv ? strncpy_P(str, &(sett_value3[0]), 15) : strncpy_P(str, &(sett_value4[0]), 15);
        break;
    case 12:
        settings.errors_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 13:
        settings.debug_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    }
}
void sett_alarm_m(char* str) {
    switch (pointers.pointer)
    {
    case 0:
        settings.alarm_autooff ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 1:
        settings.rad_alarm_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 2:
        settings.nanosv ? snprintf_P(str, 15, &(sett_value14[0]), settings.rad_th1) : snprintf_P(str, 15, &(sett_value15[0]), settings.rad_th1);
        break;
    case 3:
        settings.nanosv ? snprintf_P(str, 15, &(sett_value14[0]), settings.rad_th2) : snprintf_P(str, 15, &(sett_value15[0]), settings.rad_th2);
        break;
    case 4:
        settings.dose_alarm_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 5:
        settings.nanosv ? snprintf_P(str, 15, &(sett_value16[0]), settings.dose_th1) : snprintf_P(str, 15, &(sett_value17[0]), settings.dose_th1);
        break;
    case 6:
        settings.nanosv ? snprintf_P(str, 15, &(sett_value16[0]), settings.dose_th2) : snprintf_P(str, 15, &(sett_value17[0]), settings.dose_th2);
        break;
    }
}
void sett_sound_m(char* str) {
    switch (pointers.pointer)
    {
    case 0:
        settings.button_sound ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 1:
        settings.det_sound ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 2:
        settings.det_sound_warning_only ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 3:
        settings.measure_sound ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 4:
        settings.errors_sound ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    }
}
void sett_det_m(char* str) {
    switch (pointers.pointer)
    {
    case 0:
        settings.det_1_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 1:
        settings.det_2_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 2:
        settings.det_3_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 3:
        settings.det_4_on ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    case 4:
        settings.own_on ? strncpy_P(str, &(sett_value7[0]), 15) : strncpy_P(str, &(sett_value8[0]), 15);
        break;
    case 5:
        snprintf_P(str, 15, &(sett_value18[0]), settings.det_1_own);
        break;
    case 6:
        snprintf_P(str, 15, &(sett_value18[0]), settings.det_2_own);
        break;
    case 7:
        snprintf_P(str, 15, &(sett_value18[0]), settings.det_3_own);
        break;
    case 8:
        snprintf_P(str, 15, &(sett_value18[0]), settings.det_4_own);
        break;
    case 9:
        settings.dead_time_on ? strncpy_P(str, &(sett_value7[0]), 15) : strncpy_P(str, &(sett_value8[0]), 15);
        break;
    case 10:
        snprintf_P(str, 15, &(sett_value19[0]), settings.det_dead_time);
        break;
    case 11:
        snprintf_P(str, 15, &(sett_value20[0]), settings.det_dead_time_th);
        break;
    case 12:
        snprintf_P(str, 15, &(sett_value11[0]), settings.count_time);
        break;
    case 13:
        settings.high_count_det_off ? strncpy_P(str, &(sett_value0[0]), 15) : strncpy_P(str, &(sett_value1[0]), 15);
        break;
    }
}
void debug_scr() {
    timers.bat_check = BAT_CHECK_TIME;
    timers.auto_exit = 0;

    uint16_t ptr;
    uint8_t x, y;
    char str[4];
    float vcc_bat = (float)(settings.opornoe * 255) / (bat_adc * 100); //состояние батареи
    float opornoe = settings.opornoe * 0.01;
    uint16_t vcc_hv = (uint32_t)hv_adc * settings.opornoe * settings.k_delitel / 25500; //считем высокое перед выводом

    if (pointers.prev_pointer == NO_SCR) {
        clrRowLCD(3);
        printLCD("<FN", LEFT, 8); //БАТ
        printLCD("FWG", 46, 8); //АЦП
        printLCD("DD", LEFT, 16); //ВВ
        printLCD("FWG", 46, 16); //АЦП
        printLCD("CRH", LEFT, 24); //СКР
        pointers.prev_pointer = 0;
    }
    
    dtostrf(vcc_bat, 4, 2, str);
    printLCD(str, 20, 8); //напряжение акб
    formatAndPrint("%3u", bat_adc, 4, RIGHT, 8); //значение ацп акб
    formatAndPrint("%3u", vcc_hv, 4, 18, 16); //напряжение высокого
    formatAndPrint("%3u", hv_adc, 4, RIGHT, 16); // значение ацп вв
    formatAndPrint("%3u", speed_nak, 4, 20, 24); //скорость накачки

    for (uint8_t i = 0; i < DEBUG_ITEMS; i++)
    {
        if (i == pointers.pointer && !flags.selected) cfont.inverted = 1;

        ptr = pgm_read_word(&(sett_debug_items[i]));
        x = i > 1 ? 46 : LEFT;
        y = i == 1 || i == 3 ? 40 : 32;
        printMenuItem(ptr, x, y, 1);

        cfont.inverted = (i == pointers.pointer && flags.selected) ? 1 : 0;

        x = i > 1 ? RIGHT : 20;

        switch (i)
        {
        case 0:
            dtostrf(opornoe, 4, 2, str);
            printLCD(str, x, y);
            break;
        case 1:
            formatAndPrint("%3u", settings.puls, 4, x, y);
            break;
        case 2:
            formatAndPrint("%3u", settings.k_delitel, 4, x, y);
            break;
        case 3:
            formatAndPrint("%3u", settings.ADC_value, 4, x, y);
            break;
        }
        cfont.inverted = 0;
    }
    speed_nak = 0;
}
void det_debug_scr() {
    timers.auto_exit = 0;

    if (pointers.prev_pointer == NO_SCR) {
        for (uint8_t i = 8; i < 33; i += 8)
            printLCD(" bvg ", RIGHT, i);
        printLCD(" ctr", RIGHT, 40);
        pointers.prev_pointer = 0;
    }

    formatAndPrint(" %8u", det_dbg.d1, 10, LEFT, 8);
    formatAndPrint(" %8u", det_dbg.d2, 10, LEFT, 16);
    formatAndPrint(" %8u", det_dbg.d3, 10, LEFT, 24);
    formatAndPrint(" %8u", det_dbg.d4, 10, LEFT, 32);
    formatAndPrint(" %4u", det_dbg_timeout, 10, LEFT, 40);
    formatAndPrint(" %4u", det_dbg.time, 10, 30, 40);
}
void confirmation_scr() {
    
    clrRowLCD(4);

    if (pointers.prev_pointer == NO_SCR) {
        clrRowLCD(1);
        clrRowLCD(2);
        clrRowLCD(3);
        clrRowLCD(5);
        switch (pointers.pointer)
        {
        case CONFIRM_SETTINGS:
            printLCD("Cj[hfybnm", CENTER, 8); //Сохранить
            printLCD("yfcnhjqrb?", CENTER, 16); //настройки?
            break;
        case CONFIRM_DOSE:
            printLCD("C,hjcbnm", CENTER, 8); //Сбросить
            printLCD("ntreoe/ ljpe?", CENTER, 16); //текущщую дозу?
            break;
        case CONFIRM_ALLTIME_DOSE:
            printLCD("C,hjcbnm", CENTER, 8); //Сбросить
            printLCD("j,oe/ ljpe?", CENTER, 16); //общую дозу?
            break;
        case CONFIRM_MEASURE_RESET:
            printLCD("C,hjcbnm", CENTER, 8); //Сбросить
            printLCD("pfvth&", CENTER, 16); //замер?
            break;
        case CONFIRM_MEASURE_EXIT:
            printLCD("Jcnfyjdbnm", CENTER, 8); //Остановить
            printLCD("b dsqnb&", CENTER, 16); //и выйти?
            break;
        case CONFIRM_MEASURE_NEXT:
            printLCD("Cktle/obq", CENTER, 8); //Следующий
            printLCD("pfvth&", CENTER, 16); //замер?
            break;
        }
        pointers.prev_pointer = 0;
    }

    if (!flags.confirmation) cfont.inverted = 1;
    printLCD("  ytn  ", LEFT, 32);
    if (flags.confirmation) cfont.inverted = 1;
    else cfont.inverted = 0;
    printLCD("  lf  ", RIGHT, 32);
    cfont.inverted = 0;
}
void error_scr() {
    timers.auto_exit = 0;

    if (pointers.prev_pointer == NO_SCR) {

        clrScrLCD();

        cfont.inverted = 1;
        printLCD("    JIB<RF!   ", CENTER, 0); //ОШИБКА!
        cfont.inverted = 0;

        if (error >= ERR_LOW_BAT) {
            printLCD("Pfhzlbnt", CENTER, 16); //Зарядите
            printLCD("FR<!", CENTER, 24); //АКБ!
        }
        else if (error >= ERR_OVERLOAD) {
            printLCD("Gthtuheprf", CENTER, 16); //Перегрузка
            printLCD("ghtj,hfpjdfn!", CENTER, 24); //преобразоват!
            formatAndPrint("FWG& %u", hv_adc, 15, CENTER, 32); //АЦП:
        }
        else if (error >= ERR_SHORT) {
            printLCD("RP", CENTER, 16); //КЗ
            printLCD("ghtj,hfpjdfn!", CENTER, 24); //преобразователя!
            formatAndPrint("FWG& %u", hv_adc, 15, CENTER, 32); //АЦП:
        }
        else {
            uint16_t det_num = 0;
            printLCD("Jncencnde/n", CENTER, 16); //Отсутствуют
            printLCD("bvgekmcs jn", CENTER, 24); //импульсы от
            if (error & ERR_NOIMP_D1)
                det_num = det_num * 10 + 1;
            if (error & ERR_NOIMP_D2)
                det_num = det_num * 10 + 2;
            if (error & ERR_NOIMP_D3)
                det_num = det_num * 10 + 3;
            if (error & ERR_NOIMP_D4)
                det_num = det_num * 10 + 4;
            formatAndPrint("lfnxbrjd& %u", det_num, 15, CENTER, 32); //датчиков: 
        }
        pointers.prev_pointer = 0;
    }
}
void alarm_scr() {
    timers.auto_exit = 0;

    clrScrLCD();

    cfont.inverted = 1;
    printLCD("  JGFCYJCNM!  ", CENTER, 0); //ОПАСНОСТЬ!
    cfont.inverted = 0;
    printLCD("Ghtdsity", CENTER, 16); //Превышен
    printLCD("gjhju ", 12, 32); //порог

    pointers.pointer % 2 ? printLCD("gthdsq", CENTER, 24) : printLCD("dnjhjq", CENTER, 24); //первый : второй


    if (pointers.pointer < ALARM_DOSE1) {
        printLCD("ajyf", 48, 32); //фона
        formatAndPrint("AJY %uvrH|x", rad_back, 15, CENTER, 40); //ФОН
    }
    else
    {
        printLCD("ljps", 48, 32); //дозы
        formatAndPrint("LJPF %uvrH", dose, 15, CENTER, 40); //ДОЗА
    }
}
void printMenuItem(uint16_t ptr, uint8_t x, uint8_t y, boolean nofill) {
    char str[15];
    boolean fill = 0;

    for (uint8_t i = 0; i < 14; i++) {
        if (!fill)
        {
            str[i] = pgm_read_byte(ptr++);
            if (str[i] == NULL) {
                if (nofill) break;
                else {
                    fill = 1;
                    str[i] = 0x20;
                }
            }
        }
        else str[i] = 0x20;
    }

    str[14] = NULL;

    printLCD(str, x, y);
}
void formatAndPrint(char* fmt, uint32_t value, uint8_t length, uint8_t x, uint8_t y) {
    char str[15];
    snprintf(str, length, fmt, value);
    printLCD(str, x, y);
}
#pragma endregion
#pragma region PUMP
//-----------------------------------Первая накачка--------------------------------------------
void start_pump() //первая накачка
{
    uint32_t i = millis() + START_PUMP_TIME; //таймер авто-выхода

    for (hv_adc = Read_HV(); hv_adc < settings.ADC_value; hv_adc = Read_HV()) { //значение АЦП при котором на выходе 400В

        CONV_PORT |= (1 << CONV_BIT); //пин накачки
        for (uint8_t c = settings.puls; c > 0; c--) asm("nop"); //ждем
        CONV_PORT &= ~(1 << CONV_BIT); //пин накачки

        if (i < millis()) break; //если время вышло, останавливаем накачку

        for (uint8_t i = 0; i < map(hv_adc, 0, settings.ADC_value, 0, 64); i++) { //рисуем прогресс бар накачки преобразователя
            drawBitmapLCD(i + 10, 32, l0, 1, 8);
        }
    }
}
//----------------------------Накачка по обратной связи с АЦП---------------------------------
void pump() //накачка по обратной связи с АЦП
{
    uint8_t i = 0; //счетчик циклов переполнения

    for (hv_adc = Read_HV(); hv_adc < settings.ADC_value; hv_adc = Read_HV()) { //значение АЦП при котором на выходе 400В

        CONV_PORT |= (1 << CONV_BIT); //пин накачки
        for (uint8_t c = settings.puls; c > 0; c--) asm("nop"); //ждем
        CONV_PORT &= ~(1 << CONV_BIT); //пин накачки

        speed_nak++; //считаем скорость накачки

        if (++i >= CYCLE_OVERFLOW) { //защита от зацикливания
            if (hv_adc < 10 && !(error & ERR_SHORT)) error += ERR_SHORT; //устанавливаем ошибку кз преобразователя
            else if (!(error & ERR_SHORT))error += ERR_OVERLOAD; //устанавливаем ошибку перегрузка преобразователя
            break;
        }
    }
}
//------------------------Чтение напряжения преобразователя-----------------------------------
uint16_t Read_HV() //чтение напряжения преобразователя
{
    uint16_t result = 0; //результат опроса АЦП внутреннего опорного напряжения

    ADMUX = 0b11100110; //выбор внутреннего опорного 1,1В и А6
    ADCSRA = 0b11100111; //настройка АЦП

    for (uint8_t i = 0; i < 8; i++) { //делаем 8 замеров
        while ((ADCSRA & 0x10) == 0); //ждем флага прерывания АЦП
        ADCSRA |= 0x10; //сбрасываем флаг прерывания
        result += ADCH; //прибавляем замер в буфер
    }
    result >>= 3; //находим среднее значение
    return result; //возвращаем результат опроса АЦП
}
#pragma endregion
#pragma region BATTERY
//-----------------------------------Опрос батареи-----------------------------------------------
void bat_check() //опрос батареи
{
    bat_adc = VCC_read(); //состояние батареи

    if (bat_adc == 255) bat_adc = BAT_MAX; //защита от ложных данных

    if (bat_adc < BAT_MIN) { //мин.напр. 3v макс.нап. 4,2v
        if (bat_adc < BAT_MAX) bat_level = 5;  //если батарея заряжена
        else { //иначе расчитывает указатель заряда батареи
            bat_level = map(bat_adc, BAT_MIN, BAT_MAX, 0, 5); //переводим значение в полосы акб
            bat_level = constrain(bat_level, 0, 5); //ограничиваем
            if (bat_adc >= BAT_WARN)
                error += ERR_LOW_BAT;
        }
    }
    else poweroff();
}
//--------------------------------Чтение напряжения батареи-------------------------------------
uint8_t VCC_read()  //чтение напряжения батареи
{
    ADMUX = 0b01101110; //выбор внешнего опорного+BG
    ADCSRA = 0b11100111; //настройка АЦП
    _delay_ms(1);
    while ((ADCSRA & 0x10) == 0); //ждем флага прерывания АЦП
    ADCSRA |= 0x10; //сбрасываем флаг прерывания
    return ADCH;
}
#pragma endregion
#pragma region EEPROM
void settings_check() {
    settings_struct buffer = EEPROM.get(EEPROM_SETTINGS_STRUCT, buffer);

    if (settings.ADC_value != buffer.ADC_value ||
        settings.alarm_autooff != buffer.alarm_autooff ||
        settings.auto_exit_time != buffer.auto_exit_time ||
        settings.errors_sound != buffer.errors_sound ||
        settings.bl_time != buffer.bl_time ||
        settings.button_sound != buffer.button_sound ||
        settings.count_time != buffer.count_time ||
        settings.dead_time_on != buffer.dead_time_on ||
        settings.debug_on != buffer.debug_on ||
        settings.det_1_on != buffer.det_1_on ||
        settings.det_1_own != buffer.det_1_own ||
        settings.det_2_on != buffer.det_2_on ||
        settings.det_2_own != buffer.det_2_own ||
        settings.det_3_on != buffer.det_3_on ||
        settings.det_3_own != buffer.det_3_own ||
        settings.det_4_on != buffer.det_4_on ||
        settings.det_4_own != buffer.det_4_own ||
        settings.det_dead_time != buffer.det_dead_time ||
        settings.det_dead_time_th != buffer.det_dead_time_th ||
        settings.det_sound != buffer.det_sound ||
        settings.det_sound_warning_only != buffer.det_sound_warning_only ||
        settings.dose_alarm_on != buffer.dose_alarm_on ||
        settings.dose_save_time != buffer.dose_save_time ||
        settings.dose_th1 != buffer.dose_th1 ||
        settings.dose_th2 != buffer.dose_th2 ||
        settings.errors_on != buffer.errors_on ||
        settings.high_count_det_off != buffer.high_count_det_off ||
        settings.k_delitel != buffer.k_delitel ||
        settings.lcd_contrast != buffer.lcd_contrast ||
        settings.measure_sound != buffer.measure_sound ||
        settings.measure_th != buffer.measure_th ||
        settings.measure_time != buffer.measure_time ||
        settings.nanosv != buffer.nanosv ||
        settings.opornoe != buffer.opornoe ||
        settings.own_on != buffer.own_on ||
        settings.puls != buffer.puls ||
        settings.rad_alarm_on != buffer.rad_alarm_on ||
        settings.rad_th1 != buffer.rad_th1 ||
        settings.rad_th2 != buffer.rad_th2 ||
        settings.rfflash_on != buffer.rfflash_on ||
        settings.rf_nosleep != buffer.rf_nosleep ||
        settings.search_lr != buffer.search_lr ||
        settings.sigma != buffer.sigma ||
        settings.sleep_time != buffer.sleep_time) {

        pointers.cur_scr = CONFIRMATION_SCR;
        pointers.pointer = CONFIRM_SETTINGS;
        flags.confirmation = 0;
    }
    else
    {
        pointers.cur_scr = pointers.prev2_scr;
        pointers.pointer = pointers.mainscr_pointer;
        flags.statusbar_upd = 1;
    }
    pointers.prev_pointer = NO_SCR;
}
//------------------------------------Чтение настроек----------------------------------------------
void settings_read()
{
    if (eeprom_read_byte(EEPROM_SETTINGS_VERSION) == SETTINGS_STRUCT_VERSION)
        EEPROM.get(EEPROM_SETTINGS_STRUCT, settings);
    else 
        settings_update();
}
//------------------------------------Чтение дозы----------------------------------------------
void dose_read()
{
    dose_mask = eeprom_read_byte(EEPROM_DOSE_DIRTY_BIT);
    if (dose_mask > 3 || dose_mask < 2) {
        dose_mask = 2;
        eeprom_update_byte(EEPROM_DOSE_DIRTY_BIT, dose_mask);
    }
    else {
        uint8_t buff;
        for (dose_curr_offset = EEPROM_DOSE + 15; dose_curr_offset < 1024; dose_curr_offset += 8)
        {
            buff = eeprom_read_byte(dose_curr_offset);
            if (buff == 0xFF || (buff >> 6) != dose_mask) {
                dose_curr_offset -= 15;
                return;
            }
        }
    }
    dose_curr_offset = EEPROM_DOSE;
    eeprom_update_dword(dose_curr_offset, 0);
    uint32_t buff = 0 | ((uint32_t)dose_mask << 30);
    eeprom_update_dword(dose_curr_offset + 4, buff);
}
//---------------------------------------Чтение калибровок--------------------------------------------------
void calibration_read()
{
    if (eeprom_read_byte(EEPROM_CAL_FLAG) == 101) {
        settings.puls = eeprom_read_byte(EEPROM_PULS);
        settings.opornoe = eeprom_read_float(EEPROM_OPORNOE) * 100;
        settings.ADC_value = eeprom_read_byte(EEPROM_ADC);
        settings.k_delitel = eeprom_read_word(EEPROM_KDEL);
    }
    else calibration_update();
}
//------------------------------------Обновление настроек----------------------------------------------
void settings_update()
{
#ifndef DISABLE_EEPROM_WRITES
    EEPROM.put(EEPROM_SETTINGS_STRUCT, settings);
    eeprom_update_byte(EEPROM_SETTINGS_VERSION, SETTINGS_STRUCT_VERSION);
#endif // !DISABLE_EEPROM
}
//---------------------------------------Обновление статистики--------------------------------------------------
void dose_update(boolean reset)
{
#ifndef DISABLE_EEPROM_WRITES
    uint32_t buff_time, buff_dose;
    if (!reset)
    {
        buff_dose = eeprom_read_dword(dose_curr_offset);
        buff_dose += dose - dose_old;
        dose_old = dose;

        buff_time = eeprom_read_dword(dose_curr_offset + 4);
        buff_time &= ~((uint32_t)dose_mask << 30);
        buff_time += dose_time - dose_time_old;
        if (buff_time > 0x3FFFFFFF) buff_time = 0x3FFFFFFF;
        dose_time_old = dose_time;
    }
    else {
        buff_dose = 0;
        dose_old = 0;
        buff_time = 0;
        dose_time_old = 0;
    }

    dose_curr_offset += 8;
    if (dose_curr_offset > 1016) {
        dose_curr_offset = EEPROM_DOSE;
        dose_mask = dose_mask == 2 ? 3 : 2;
        eeprom_update_byte(EEPROM_DOSE_DIRTY_BIT, dose_mask);
    }

    buff_time |= ((uint32_t)dose_mask << 30);

    eeprom_update_dword(dose_curr_offset, buff_dose);
    eeprom_update_dword(dose_curr_offset + 4, buff_time);

#endif // !DISABLE_EEPROM
}
//---------------------------------------Обновление калибровок--------------------------------------------------
void calibration_update()
{
#ifndef DISABLE_EEPROM_WRITES
    eeprom_update_byte(EEPROM_PULS, settings.puls);
    eeprom_update_float(EEPROM_OPORNOE, (float)(settings.opornoe * 0.01));
    eeprom_update_byte(EEPROM_ADC, settings.ADC_value);
    eeprom_update_word(EEPROM_KDEL, settings.k_delitel);
    eeprom_update_byte(EEPROM_CAL_FLAG, 101); //делаем метку
#endif // !DISABLE_EEPROM
}
#pragma endregion