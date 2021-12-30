/* LVGL Example project
 *
 * Basic project to test LVGL on ESP32 based projects.
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

/* Littlevgl specific */
#include "lvgl_gui.h"
#include "lvgl/lvgl.h"
#include "lvgl_helpers.h"

#include "esp_ibeacon_api.h"
#include "ws2812.h"
#include "sensor.h"
#include "wifi_station.h"

//标志位
int WIFI_flag = 0;  //0为未开启，1为开启
int BLE_flag = 0;   //0为未开启，1为开启
int BAT_flag = 0;   //0-3为电量
int WS2812_state = 0;   //0为正常，1为异常
//传感器数据
float tempData = 0, humData = 0;
float tvocData = 0, eco2Data = 0;
//滚动条数据
int TypeWeather = 0;    //天气类型
int outdoorTemp = 0;   //室外气温
int bilibili_fans = 0;  //B站粉丝数

static void ble_task(void *arg){
    int TH_major = 0,CT_minor=0;
    while(1){
        TH_major = (int)(tempData*10)*100+(int)humData;
        CT_minor = (int)(eco2Data/10)*100 + (int)tvocData/100;
        BLE_Update(TH_major, CT_minor);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

static void ws2812_task(void *arg) {
    int R=255,G=0,B=0;
    while(1){
        if(eco2Data>1000||tvocData>1000){
            WS2812_state = 1;
        }else{
            WS2812_state = 0;
        }

        if(WS2812_state == 0){
            //变色
            if(R==255 && B==0){G++;}
            if(G==255 && B==0){R--;}
            if(R==0 && G==255){B++;}
            if(R==0 && B==255){G--;}
            if(G==0 && B==255){R++;}
            if(R==255 && G==0){B--;}
            ws2812_setColor_grb(G,R,B);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }else{
            ws2812_setColor_grb(255,0,0);   //设为红色
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}



void app_main()
{
    //LVGL初始化
    GUI_init();
    //SHT30初始化
    Sensor_init();
    //wifi初始化
    wifi_init_sta();
    //蓝牙初始化
    BLE_init();


    //WS2812指示灯任务
    nvs_flash_init();
    ws2812_init(WS2812_PIN, WS2812_COUNTS);
    TaskHandle_t xTask_WS2812_Handle;
    xTaskCreate(ws2812_task, "ws2812_task", 4096, NULL, 10,  &xTask_WS2812_Handle);

    //蓝牙广播更新任务
    TaskHandle_t xTask_ble_Handle;
    xTaskCreate(ble_task, "ble_task", 1024, NULL, 10, &xTask_ble_Handle);

}
