//#define DEBUG
//#define LIGHT_INV
//#define DISABLE_EEPROM_WRITES

void config() {

    //общие
    settings.sigma = 2;
    settings.measure_time = 0; //measur_time == 0 ? % : secs;
    settings.measure_th = 15; //measur_time == 0 ? % : secs;
    settings.bl_time = 0; // secs
    settings.lcd_contrast = 70; 
    settings.auto_exit_time = 0; // secs
    settings.dose_save_time = 5; //mins
    settings.sleep_time = 0; //secs
    settings.rf_nosleep = 1;
    settings.rfflash_on = 1;
    settings.search_lr = 0;
    settings.nanosv = 0;
    settings.errors_on = 1;
    settings.debug_on = 1;

    //тревога
    settings.alarm_autooff = 1;
    settings.rad_alarm_on = 0;
    settings.rad_th1 = 30; //mkR/h
    settings.rad_th2 = 300; //mkR/h
    settings.dose_alarm_on = 0;
    settings.dose_th1 = 30; //mkR
    settings.dose_th2 = 300; //mkR

    //звук
    settings.button_sound = 1;
    settings.det_sound = 1;
    settings.det_sound_warning_only = 0;
    settings.measure_sound = 1;
    settings.errors_sound = 1;

    //датчики
    settings.det_1_on = 0;
    settings.det_2_on = 1;
    settings.det_3_on = 1;
    settings.det_4_on = 1;
    settings.own_on = 1;
    settings.det_1_own = 0; //imp/s*1000
    settings.det_2_own = 140; //imp/s*1000
    settings.det_3_own = 150; //imp/s*1000
    settings.det_4_own = 200; //imp/s*1000
    settings.dead_time_on = 1;
    settings.det_dead_time = 200; // us
    settings.det_dead_time_th = 250; //imp/s
    settings.count_time = 55; // secs
    settings.high_count_det_off = 1;

    //отладка
    settings.opornoe = 108; // V*100
    settings.puls = 11;
    settings.k_delitel = 479;
    settings.ADC_value = 196;

}