#ifndef _BSP_WIFI_STATION_H_
#define _BSP_WIFI_STATION_H_

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_http_client.h"
#include "cJSON.h"
#include <stdlib.h> 


//定义联网相关的宏
#define EXAMPLE_ESP_WIFI_SSID      "XF-WIFI"            //账号
#define EXAMPLE_ESP_WIFI_PASS      "363551935"        //密码
#define EXAMPLE_ESP_MAXIMUM_RETRY  5					//wifi连接失败以后可以重新连接的次数
#define WIFI_CONNECTED_BIT BIT0                         //wifi连接成功标志位
#define WIFI_FAIL_BIT      BIT1							//wifi连接失败标志位

//定义联网所需要的变量
static EventGroupHandle_t s_wifi_event_group;  //事件组，用于对wifi响应结果进行标记
static const char* TAG = "wifi station";      //log标志位
static int s_retry_num = 0;                     //记录wifi重新连接尝试的次数

extern int WIFI_flag;
extern int TypeWeather;
extern int outdoorTemp;
extern int bilibili_fans;

void wifi_init_sta();  //用于连接wifi
static void http_weather_task(void *pvParameters);
static void http_bilibili_task(void *pvParameters);


#endif
