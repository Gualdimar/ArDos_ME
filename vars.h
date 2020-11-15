#define SETTINGS_STRUCT_VERSION 1
struct settings_struct
{
    boolean det_1_on : 1;
    boolean det_2_on : 1;
    boolean det_3_on : 1;
    boolean det_4_on : 1;
    boolean high_count_det_off : 1;
    boolean own_on : 1;
    boolean dead_time_on : 1;
    boolean measure_time : 1;
    boolean search_lr : 1;
    boolean debug_on : 1;
    boolean errors_on : 1;
    boolean button_sound : 1;
    boolean det_sound : 1;
    boolean det_sound_warning_only : 1;
    boolean measure_sound : 1;
    boolean errors_sound : 1;
    boolean nanosv : 1;
    boolean rad_alarm_on : 1;
    boolean dose_alarm_on : 1;
    boolean alarm_autooff : 1;
    boolean rfflash_on : 1;
    boolean rf_nosleep : 1;
    uint8_t sigma : 2;
    uint16_t rad_th2 : 12;
    uint16_t dose_th2 : 12;
    uint8_t puls : 5;
    uint16_t k_delitel : 10;
    uint8_t ADC_value;
    uint8_t opornoe;
    uint8_t lcd_contrast;
    uint8_t det_1_own;
    uint8_t det_2_own;
    uint8_t det_3_own;
    uint8_t det_4_own;
    uint8_t det_dead_time;
    uint8_t det_dead_time_th;
    uint8_t measure_th;
    uint8_t count_time;
    uint8_t dose_save_time;
    uint8_t auto_exit_time;
    uint8_t sleep_time;
    uint8_t bl_time;
    uint8_t rad_th1;
    uint8_t dose_th1;
} settings;

struct
{
    boolean default_settings : 1;
    boolean default_calibration : 1;
    boolean silent_mode : 1;
    boolean is_sleeping : 1;
    volatile boolean data : 1;
    volatile boolean screen : 1;
    boolean dose_alltime : 1;
    boolean statusbar_upd : 1;
    boolean btn_state : 1;
    boolean btn_check : 1;
    boolean btn_skip : 1;
    boolean selected : 1;
    boolean bl_off : 1;
    boolean confirmation : 1;
    boolean det_dbg : 1;
    boolean sound : 1;
    boolean rad_alarm1_off : 1;
    boolean rad_alarm2_off : 1;
    boolean dose_alarm1_off : 1;
    boolean dose_alarm2_off : 1;
    boolean measure : 1;
} flags;

volatile struct
{
    uint8_t dose_scr : 4;
    uint8_t bat_check : 4;
    uint16_t dose_save : 14;
    uint8_t auto_exit;
    uint8_t backlight;
    uint8_t error;
} timers;

#define DOSE_SCR_UPD_TIME 5 //secs
#define BAT_CHECK_TIME 10 //secs

#define LOGO_TIME 2 //secs
#define START_PUMP_TIME 3000 //ms
#define RF_TIME  17 //ms

#define CYCLE_OVERFLOW 10
#define IMP_BUFF_LENGTH 201

#define BTN_HOLD_MS 500 //ms
#define BTN_DEBOUNCE_MS 90 //ms

#define BUZZ_FREQ  500 //Hz
#define BUZZ_TIME  10 //ms

#define FREQ_BEEP  2000 //частота звука клавиш(10..10000)(Hz)
#define TIME_BEEP  30 //длительность звука клавиш(10..500)(ms)

#define DET_MAX_CNT 2600 //imp/s

#define BAT_MIN 90 //значение АЦП при разряженной батарее(0..255)
#define BAT_WARN 83 //значение АЦП для звука разряженной батарее(0..255)
#define BAT_MAX 70 //значение АЦП при заряженной батарее(0..255)

uint8_t hv_adc, det_count, avg_period, dose_mask, bat_level, bat_adc;
uint16_t det_off_th, speed_nak, dose_curr_offset, det_dbg_timeout;
uint32_t imp_dose, dose_time, dose, dose_time_old, dose_old, rad_back, rad_max, avg_buff, avg_avg, btn_tmr;
volatile uint8_t imp_buff_pos, det_1, det_2, det_3, det_4;
volatile uint16_t imp_buff[IMP_BUFF_LENGTH];
volatile uint32_t rf_timer;
float accur, det_own;


struct
{
    uint8_t cur_scr : 4;
    uint8_t prev_scr : 4;
    uint8_t prev2_scr : 4;
    uint8_t prev_pointer : 4;
    uint8_t pointer : 4;
    uint8_t mainscr_pointer : 2;
    uint8_t measure_pointer : 2;
    uint8_t settscr_pointer : 3;
    uint8_t sound : 5;
} pointers;

volatile struct
{
    uint16_t d1;
    uint16_t d2;
    uint16_t d3;
    uint16_t d4;
    uint16_t time;
} det_dbg;

struct 
{
    uint8_t accur;
    uint16_t imp;
    uint16_t imp1;
    uint32_t rad : 24;
    uint32_t rad1 : 24;
    uint16_t time : 14;
} measure;

//pointers
#define MONITORING_SCR 0
#define SEARCH_SCR 1
#define DOSE_SCR 2
#define CONFIRM_SETTINGS 0
#define CONFIRM_DOSE 1
#define CONFIRM_ALLTIME_DOSE 2
#define CONFIRM_MEASURE_RESET 3
#define CONFIRM_MEASURE_EXIT 4
#define CONFIRM_MEASURE_NEXT 5
#define ALARM_RAD1 1
#define ALARM_RAD2 2
#define ALARM_DOSE1 3
#define ALARM_DOSE2 4
#define MEASURE_1 0
#define MEASURE_2 1
#define MEASURE_RESULT 2
#define NO_SCR 14
//screens
#define MAIN_SCR 0
#define MEASURE_SCR 1
#define FAST_MENU_SCR 2
#define SETT_MAIN_SCR 3
#define SETT_GENERAL_SCR 4
#define SETT_ALARM_SCR 5
#define SETT_SOUND_SCR 6
#define SETT_DET_SCR 7
#define DEBUG_SCR 8
#define DET_DEBUG_SCR 9
#define CONFIRMATION_SCR 10
#define ERROR_SCR 11
#define ALARM_SCR 12

#define MAIN_ITEMS 3
#define SETT_ITEMS 7
#define GENERAL_ITEMS 14
#define ALARM_ITEMS 7
#define SOUND_ITEMS 5
#define DET_ITEMS 13
#define DEBUG_ITEMS 4

uint8_t error = 0;
#define ERR_NO 0
#define ERR_LOW_BAT 64
#define ERR_OVERLOAD 32
#define ERR_SHORT 16
#define ERR_NOIMP_D1 8
#define ERR_NOIMP_D2 4
#define ERR_NOIMP_D3 2
#define ERR_NOIMP_D4 1
#define ERROR_REPEAT_TIME 10 //sec
#define ERROR_DET_TIME 60 //sec


#define EEPROM_PULS 52
#define EEPROM_OPORNOE 53
#define EEPROM_ADC 102
#define EEPROM_KDEL 104
#define EEPROM_CAL_FLAG 101

#define EEPROM_SETTINGS_VERSION 130
#define EEPROM_SETTINGS_STRUCT 131

#define EEPROM_DOSE_DIRTY_BIT 511
#define EEPROM_DOSE 512

#define SOUND_BATTERY 1
#define SOUND_ERROR 2
#define SOUND_ALARM 3
#define SOUND_WARN 4
#define SOUND_MEASURE 5
