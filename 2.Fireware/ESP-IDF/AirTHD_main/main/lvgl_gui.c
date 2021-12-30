#include "lvgl_gui.h"


/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

lv_obj_t* img1, *img2, *img3, *img4;
lv_obj_t *label_temp, *label_hum, *label_tvoc, *label_eco2;
lv_obj_t *wifi_icon_label,*img_ble,*img_battey;
lv_obj_t *Scroll_label;

extern float tempData, humData;
extern float tvocData, eco2Data;

extern int BLE_flag;
extern int WIFI_flag;
extern int BAT_flag;

extern int TypeWeather;
extern int outdoorTemp;
extern int bilibili_fans;

char weather_list[40][30] = {"Sunny","Clear","Fair","Fair","Cloudy","Partly Cloudy","Partly Cloudy","Mostly Cloudy","Mostly Cloudy","Overcast","Shower",
                            "Thundershower","Thundershower with Hail","Light Rain","Moderate Rain","Heavy Rain","Storm","Heavy Storm","Severe Storm","Ice Rain","Sleet",
                            "Snow Flurry","Light Snow","Moderate Snow","Heavy Snow","Snowstorm","Dust","Sand","Duststorm","Sandstorm","Foggy",
                            "Haze","Windy","Blustery","Hurricane","Tropical Storm","Tornado","Cold","Hot","unknow"};

void GUI_init(){
    xTaskCreatePinnedToCore(guiTask, "gui", 4096 * 2, NULL, 0, NULL, 1);
}

void guiTask(void *pvParameter)
{

    (void)pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    static lv_color_t buf1[DISP_BUF_SIZE];

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    static lv_color_t buf2[DISP_BUF_SIZE];
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820 || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
#endif

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
    create_demo_application();

    while (1)
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        //刷新数据
        if(BLE_flag == 1){
            lv_img_set_src(img_ble, &BLUETOOTH);
        }else{
            lv_img_set_src(img_ble, NULL);
        }

        if(WIFI_flag == 1){
            lv_label_set_text(wifi_icon_label, LV_SYMBOL_WIFI);
            lv_label_set_text_fmt(Scroll_label, "bilibili_Fans : %d   Weather : %s   OutdoorTemp : %d C   ", bilibili_fans, weather_list[TypeWeather], outdoorTemp);
        }else{
            lv_label_set_text(wifi_icon_label, NULL);
            lv_label_set_text(Scroll_label, "Please open wifi:XF-WIFI, password:363551935");
        }

        switch(BAT_flag){
            case 0:
                lv_img_set_src(img_battey, &BATTEY_0);BAT_flag++;
                break;
            case 1:
                lv_img_set_src(img_battey, &BATTEY_1);BAT_flag++;
                break;
            case 2:
                lv_img_set_src(img_battey, &BATTEY_2);BAT_flag++;
                break;
            case 3:
                lv_img_set_src(img_battey, &BATTEY_3);BAT_flag= 0;
                break;
            default:
                lv_img_set_src(img_battey, &BATTEY_0);
                break;
        }

        lv_label_set_text_fmt(label_temp,"%2.1f C",tempData);
        lv_label_set_text_fmt(label_hum,"%2.1f %%",humData);
        lv_label_set_text_fmt(label_tvoc,"%4.0f",tvocData);
        lv_label_set_text_fmt(label_eco2,"%4.0f",eco2Data);

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
        {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
        
    }

    /* A task should NEVER return */
    vTaskDelete(NULL);
}


void create_demo_application(void)
{


    /*Create a screen*/
    lv_obj_t *scr = lv_obj_create(NULL, NULL);
    lv_scr_load(scr); /*Load the screen*/

    static lv_style_t label_persian_style;
    lv_style_init(&label_persian_style);
    lv_style_set_text_font(&label_persian_style, LV_STATE_DEFAULT, &lv_font_dejavu_16_persian_hebrew); /*Set persian font*/

    static lv_style_t label_min_style;
    lv_style_init(&label_min_style);
    lv_style_set_text_font(&label_min_style, LV_STATE_DEFAULT, &lv_font_unscii_8); /*Set persian font*/

    static lv_style_t label_data_style;
    lv_style_init(&label_data_style);
    lv_style_set_text_font(&label_data_style, LV_STATE_DEFAULT, &lv_font_montserrat_12_subpx); /*Set persian font*/


    /*lv_obj_t *persian_label = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(persian_label, LV_LABEL_PART_MAIN, &label_persian_style);
    lv_label_set_text(persian_label, "سلام");//你好
    lv_obj_align(persian_label, NULL, LV_ALIGN_CENTER, 0, -25);*/

    static lv_style_t label_icon_style;
    lv_style_init(&label_icon_style);
    lv_style_set_text_font(&label_icon_style, LV_STATE_DEFAULT, &lv_font_montserrat_12_subpx); /*Set persian font*/

    //WIFI标志
    wifi_icon_label = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(wifi_icon_label, LV_LABEL_PART_MAIN, &label_icon_style);
    lv_label_set_text(wifi_icon_label, LV_SYMBOL_WIFI);
    lv_obj_align(wifi_icon_label, NULL, LV_ALIGN_CENTER, 55, -23);

    //蓝牙标志
    img_ble = lv_img_create(scr, NULL);
    lv_img_set_src(img_ble, &BLUETOOTH);
    lv_obj_align(img_ble, NULL, LV_ALIGN_CENTER, 55, -5);

    //电池标志
    img_battey = lv_img_create(scr, NULL);
    lv_img_set_src(img_battey, &BATTEY_3);
    lv_obj_align(img_battey, NULL, LV_ALIGN_CENTER, 55, 10);
    /*lv_obj_t *bettery_icon_label = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(bettery_icon_label, LV_LABEL_PART_MAIN, &label_icon_style);
    lv_label_set_text(bettery_icon_label, LV_SYMBOL_BATTERY_EMPTY);
    lv_obj_align(bettery_icon_label, NULL, LV_ALIGN_CENTER, 55, 10);*/

    //图片资源
    img1 = lv_img_create(scr, NULL);
    lv_img_set_src(img1, &TEMP);
    lv_obj_align(img1, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);

    img2 = lv_img_create(scr, NULL);
    lv_img_set_src(img2, &HUM);
    lv_obj_align(img2, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 25);

    img3 = lv_img_create(scr, NULL);
    lv_img_set_src(img3, &TVOC);
    lv_obj_align(img3, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);

    img4 = lv_img_create(scr, NULL);
    lv_img_set_src(img4, &ECO2);
    lv_obj_align(img4, NULL, LV_ALIGN_IN_TOP_MID, 0, 25);

    //数据展示
    label_temp = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(label_temp, LV_LABEL_PART_MAIN, &label_data_style);
    lv_label_set_text_fmt(label_temp,"--C");
    lv_obj_align(label_temp, NULL, LV_ALIGN_IN_TOP_LEFT, 16, 0);

    label_hum = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(label_hum, LV_LABEL_PART_MAIN, &label_data_style);
    lv_label_set_text_fmt(label_hum,"--%%");
    lv_obj_align(label_hum, NULL, LV_ALIGN_IN_TOP_LEFT, 16, 25);

    label_tvoc = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(label_tvoc, LV_LABEL_PART_MAIN, &label_data_style);
    lv_label_set_text_fmt(label_tvoc,"--");
    lv_obj_align(label_tvoc, NULL, LV_ALIGN_IN_TOP_MID, 20, 0);

    label_eco2 = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(label_eco2, LV_LABEL_PART_MAIN, &label_data_style);
    lv_label_set_text_fmt(label_eco2,"--");
    lv_obj_align(label_eco2, NULL, LV_ALIGN_IN_TOP_MID, 20, 25);

    //滚动字幕
    Scroll_label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_long_mode(Scroll_label, LV_LABEL_LONG_SROLL_CIRC); /*Circular scroll*/
    lv_obj_set_width(Scroll_label, 150);
    lv_label_set_text(Scroll_label, "It is a circularly scrolling text. ");
    lv_obj_align(Scroll_label, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
    
    //TVOC单位
    lv_obj_t *label5 = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(label5, LV_LABEL_PART_MAIN, &label_min_style);
    lv_label_set_text(label5,"ppb");
    lv_obj_align(label5, NULL, LV_ALIGN_CENTER, 35, -15);

    //eCO2单位
    lv_obj_t *label4 = lv_label_create(lv_scr_act(), NULL);
    lv_obj_add_style(label4, LV_LABEL_PART_MAIN, &label_min_style);
    lv_label_set_text(label4,"ppm");
    lv_obj_align(label4, NULL, LV_ALIGN_CENTER, 35, 10);

}
void lv_tick_task(void *arg){
    (void)arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}