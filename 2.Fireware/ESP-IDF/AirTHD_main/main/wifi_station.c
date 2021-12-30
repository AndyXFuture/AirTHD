
#include "wifi_station.h"

/**
@brief 处理wifi连接和ip分配时候事件的回调函数
**/

static void event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {   //如果是wifi连接事件，就进行wifi连接
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {  //如果是wifi连接失败事件
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {   //如果没有达到最高尝试次数，继续尝试
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);   //如果达到了最高尝试次数，就停止尝试，并且标记连接失败
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {     //如果是ip获取事件，获取到了ip就打印出来
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);    //如果成功获取到了ip，就标记这次wifi连接成功
    }
}



/**
@brief 用于连接wifi的函数
@param[in] 无
@retval 无
@note 这里wifi连接选项设置了使用nvs，会把每次配置的参数存储在nvs中。因此请查看分区表中是否对nvs分区进行了设置

**/
void wifi_init_sta()
{

   //00 使能nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)  //如果nvs空间满了，就进行擦除
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

  //00 创建wifi事件的标志组
    s_wifi_event_group = xEventGroupCreate();

  //01 WIFI/LWIP初始化阶段

         //01-1 创建LWIP核心任务
        ESP_ERROR_CHECK(esp_netif_init());


         //01-2 创建系统事件任务
        ESP_ERROR_CHECK(esp_event_loop_create_default());

          //01-3 创建station实例
        esp_netif_create_default_wifi_sta();

          //01-4 创建wifi驱动程序任务，并初始化wifi驱动程序
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        //01-5 注册，用于处理wifi连接的过程中的事件
        esp_event_handler_instance_t instance_any_id;   //用于处理wifi连接时候的事件的句柄
        esp_event_handler_instance_t instance_got_ip;    //用于处理ip分配时候产生的事件的句柄
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,   //该句柄对wifi连接所有事件都产生响应，连接到event_handler回调函数
            ESP_EVENT_ANY_ID,
            &event_handler,
            NULL,
            &instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,   //该句柄仅仅处理IP_EVENT事件组中的获取ip地址事件，连接到event_handler回调函数
            IP_EVENT_STA_GOT_IP, 
            &event_handler,
            NULL,
            &instance_got_ip));
//02 WIFI配置阶段

        //02-1 定义wifi配置参数
        wifi_config_t wifi_config;                                           //定义wifi配置参数结构体
        memset(&wifi_config, 0, sizeof(wifi_config));                       //对结构体进行初始化，把参数全部定义为0
        sprintf((char*)wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID);        //配置wifi名称              
        sprintf((char*)wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS);    //配置wifi密码
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;            //配置加密协议
        wifi_config.sta.pmf_cfg.capable = true;                             
        wifi_config.sta.pmf_cfg.required = false;

        //02-2 配置wifi工作模式
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

        //02-3 写入配置
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

     
 //03 wifi启动阶段

        //03-1 启动wifi驱动程序
        ESP_ERROR_CHECK(esp_wifi_start());   //会触发回调函数
        ESP_LOGI(TAG, "wifi_init_sta finished.");

  //04 输出wifi连接结果      
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);


        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
            WIFI_flag = 1;
            xTaskCreate(&http_weather_task, "http_weather_task", 8192, NULL, 5, NULL);
            xTaskCreate(&http_bilibili_task, "http_bilibili_task", 8192, NULL, 5, NULL);
        }
        else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        }
        else {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
        }

  
//05  事件解绑定

        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
        vEventGroupDelete(s_wifi_event_group);

    

}



/**
 *  @brief 该代码要实现连接wifi，并且从心知天气获取天气和温度的代码
 * 
 **/

#define MAX_HTTP_OUTPUT_BUFFER 2048

static void http_weather_task(void *pvParameters)
{

//02-1 定义需要的变量
      char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};   //用于接收通过http协议返回的数据
    int content_length = 0;  //http协议头的长度
    

    //02-2 配置http结构体
   
   //定义http配置结构体，并且进行清零
    esp_http_client_config_t config ;
    memset(&config,0,sizeof(config));

    //向配置结构体内部写入url
    static const char *URL = "https://api.seniverse.com/v3/weather/now.json?key=SaBMO_sFlFq5XJy2w&location=guangzhou&language=zh-Hans&unit=c";
    config.url = URL;

    //初始化结构体
    esp_http_client_handle_t client = esp_http_client_init(&config);	//初始化http连接

    //设置发送请求 
    esp_http_client_set_method(client, HTTP_METHOD_GET);

    //02-3 循环通讯

    while(1)
    {


    // 与目标主机创建连接，并且声明写入内容长度为0
    esp_err_t err = esp_http_client_open(client, 0);

    //如果连接失败
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } 
    //如果连接成功
    else {

        //读取目标主机的返回内容的协议头
        content_length = esp_http_client_fetch_headers(client);

        //如果协议头长度小于0，说明没有成功读取到
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } 

        //如果成功读取到了协议头
        else {

            //读取目标主机通过http的响应内容
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {

                //打印响应内容，包括响应状态，响应体长度及其内容
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),				//获取响应状态信息
                esp_http_client_get_content_length(client));			//获取响应信息长度
                printf("data:%s\n", output_buffer);
				//对接收到的数据作相应的处理
                cJSON* root = NULL;
                root = cJSON_Parse(output_buffer);

                cJSON* cjson_item =cJSON_GetObjectItem(root,"results");
                cJSON* cjson_results =  cJSON_GetArrayItem(cjson_item,0);
                cJSON* cjson_now = cJSON_GetObjectItem(cjson_results,"now");
                cJSON* cjson_code = cJSON_GetObjectItem(cjson_now,"code");
                cJSON* cjson_temperature = cJSON_GetObjectItem(cjson_now,"temperature");
                
                TypeWeather = atoi(cjson_code->valuestring);
                outdoorTemp = atoi(cjson_temperature->valuestring);
                printf("type:%d, outdoorTemp:%d\n",TypeWeather, outdoorTemp);

            } 
            //如果不成功
            else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }

    //关闭连接
    esp_http_client_close(client);

    //延时，因为心知天气免费版本每分钟只能获取20次数据
    vTaskDelay(7000/portTICK_PERIOD_MS);

    }
	


}


static void http_bilibili_task(void *pvParameters)
{

//02-1 定义需要的变量
      char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};   //用于接收通过http协议返回的数据
    int content_length = 0;  //http协议头的长度
    

    //02-2 配置http结构体
   
   //定义http配置结构体，并且进行清零
    esp_http_client_config_t config ;
    memset(&config,0,sizeof(config));

    //向配置结构体内部写入url
    static const char *URL = "http://api.bilibili.com/x/relation/stat?vmid=32273370";
    config.url = URL;

    //初始化结构体
    esp_http_client_handle_t client = esp_http_client_init(&config);	//初始化http连接

    //设置发送请求 
    esp_http_client_set_method(client, HTTP_METHOD_GET);

    //02-3 循环通讯

    while(1)
    {


    // 与目标主机创建连接，并且声明写入内容长度为0
    esp_err_t err = esp_http_client_open(client, 0);

    //如果连接失败
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } 
    //如果连接成功
    else {

        //读取目标主机的返回内容的协议头
        content_length = esp_http_client_fetch_headers(client);

        //如果协议头长度小于0，说明没有成功读取到
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } 

        //如果成功读取到了协议头
        else {

            //读取目标主机通过http的响应内容
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {

                //打印响应内容，包括响应状态，响应体长度及其内容
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),				//获取响应状态信息
                esp_http_client_get_content_length(client));			//获取响应信息长度
                printf("data:%s\n", output_buffer);
				//对接收到的数据作相应的处理
                cJSON* root = NULL;
                root = cJSON_Parse(output_buffer);
                
                cJSON* cjson_data =cJSON_GetObjectItem(root,"data");
                cJSON* cjson_follower = cJSON_GetObjectItem(cjson_data,"follower");
                bilibili_fans = cjson_follower->valueint;
                printf("bilibili_Fans:%d\n",bilibili_fans);

            } 
            //如果不成功
            else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }

    //关闭连接
    esp_http_client_close(client);

    //延时
    vTaskDelay(5000/portTICK_PERIOD_MS);
    }
}