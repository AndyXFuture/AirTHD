/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/



/****************************************************************************
*
* This file is for iBeacon demo. It supports both iBeacon sender and receiver
* which is distinguished by macros IBEACON_SENDER and IBEACON_RECEIVER,
*
* iBeacon is a trademark of Apple Inc. Before building devices which use iBeacon technology,
* visit https://developer.apple.com/ibeacon/ to obtain a license.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_ibeacon_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <stdio.h>
#include "string.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <stdio.h>
#include "esp_system.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "oled.h"
#include "fonts.h"

#include "SGP30.h"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_MASTER_SCL_IO    33               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    32               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_1 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

i2c_port_t i2c_num = I2C_MASTER_NUM;

static const char *TAG = "sgp30-test";
int TVOC = 0, eCO2 = 0;

int TH_major = 0;
int CT_minor = 0;

/**
 * @brief generic function for reading I2C data
 * 
 * @param reg_addr register adress to read from 
 * @param reg_data pointer to save the data read 
 * @param len length of data to be read
 * @param intf_ptr 
 * 
 * >init: dev->intf_ptr = &dev_addr;
 * 
 * @return ESP_OK if reading was successful
 */
int8_t main_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { // *intf_ptr = dev->intf_ptr
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    if (len == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = *(uint8_t*)intf_ptr;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    
    if (reg_addr != 0xff) {
        i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_start(cmd);
    }
    
    i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    return ret;
}


/**
 * @brief generic function for writing data via I2C 
 *  
 * @param reg_addr register adress to write to 
 * @param reg_data register data to be written 
 * @param len length of data to be written
 * @param intf_ptr 
 * 
 * @return ESP_OK if writing was successful
 */
int8_t main_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t chip_addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    
    if (reg_addr != 0xFF) {
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }

    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    return ret;
}


static void main_task(void *arg) {
    ESP_LOGI(TAG, "SGP30 main task initializing...");

    sgp30_dev_t main_sgp30_sensor;

    //i2c_master_driver_initialize();

    sgp30_init(&main_sgp30_sensor, (sgp30_read_fptr_t)main_i2c_read, (sgp30_write_fptr_t)main_i2c_write);

    // SGP30 needs to be read every 1s and sends TVOC = 400 14 times when initializing
    for (int i = 0; i < 14; i++) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        sgp30_IAQ_measure(&main_sgp30_sensor);
        ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2);
    }

    // Read initial baselines 
    uint16_t eco2_baseline, tvoc_baseline;
    sgp30_get_IAQ_baseline(&main_sgp30_sensor, &eco2_baseline, &tvoc_baseline);
    ESP_LOGI(TAG, "BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);


    ESP_LOGI(TAG, "SGP30 main task is running...");
    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        sgp30_IAQ_measure(&main_sgp30_sensor);

        ESP_LOGI(TAG, "TVOC: %d,  eCO2: %d",  main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2);
        TVOC = main_sgp30_sensor.TVOC;
        eCO2 = main_sgp30_sensor.eCO2;
    }
}


/*
===========================
宏定义
=========================== 
*/
//I2C 
#define I2C_SCL_IO          33                  //SCL->IO33
#define I2C_SDA_IO          32                  //SDA->IO32
#define I2C_MASTER_NUM      I2C_NUM_1           //I2C_1
#define WRITE_BIT           I2C_MASTER_WRITE    //写:0
#define READ_BIT            I2C_MASTER_READ     //读:1
#define ACK_CHECK_EN        0x1                 //主机检查从机的ACK
#define ACK_CHECK_DIS       0x0                 //主机不检查从机的ACK
#define ACK_VAL             0x0                 //应答
#define NACK_VAL            0x1                 //不应答

//SHT30
#define SHT30_WRITE_ADDR    0x44                //地址 
#define CMD_FETCH_DATA_H    0x22                //循环采样，参考sht30 datasheet
#define CMD_FETCH_DATA_L    0x36

/*
===========================
全局变量定义
=========================== 
*/
unsigned char sht30_buf[6]={0}; 
float g_temp=0.0, g_rh=0.0;

/*
* IIC初始化
* @param[in]   void  		       :无
* @retval      void                :无
* @note        修改日志 
*               Ver0.0.1:
                    hx-zsj, 2018/08/06, 初始化版本\n 
*/
/*void i2c_init(void)
{
	//i2c配置结构体
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                    //I2C模式
    conf.sda_io_num = I2C_SDA_IO;                   //SDA IO映射
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;        //SDA IO模式
    conf.scl_io_num = I2C_SCL_IO;                   //SCL IO映射
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;        //SCL IO模式
    conf.master.clk_speed = 100000;                 //I2C CLK频率
    conf.clk_flags = 0;
    i2c_param_config(I2C_MASTER_NUM, &conf);        //设置I2C
    //注册I2C服务即使能
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,0,0,0);
}*/

/*
* sht30初始化
* @param[in]   void  		        :无
* @retval      int                  :0成功，其他失败
* @note        修改日志 
*               Ver0.0.1:
                    hx-zsj, 2018/08/06, 初始化版本\n 
*/
int sht30_init(void)
{
    int ret;
    //配置SHT30的寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                   //新建操作I2C句柄
    i2c_master_start(cmd);                                                          //启动I2C
    i2c_master_write_byte(cmd, SHT30_WRITE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);    //发地址+写+检查ack
    i2c_master_write_byte(cmd, CMD_FETCH_DATA_H, ACK_CHECK_EN);                     //发数据高8位+检查ack
    i2c_master_write_byte(cmd, CMD_FETCH_DATA_L, ACK_CHECK_EN);                     //发数据低8位+检查ack
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送
    i2c_cmd_link_delete(cmd);                                                       //删除I2C句柄
    return ret;
}

/*
* sht30校验算法
* @param[in]   pdata  		        :需要校验的数据
* @param[in]   nbrOfBytes  		    :需要校验的数据长度
* @retval      int                  :校验值
* @note        修改日志 
*               Ver0.0.1:
                    hx-zsj, 2018/08/06, 初始化版本\n 
*/
unsigned char SHT3X_CalcCrc(unsigned char *data, unsigned char nbrOfBytes)
{
	unsigned char bit;        // bit mask
    unsigned char crc = 0xFF; // calculated checksum
    unsigned char byteCtr;    // byte counter
    unsigned int POLYNOMIAL =  0x131;           // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            }  else {
                crc = (crc << 1);
            }
        }
    }
	return crc;
}
/*
* sht30数据校验
* @param[in]   pdata  		        :需要校验的数据
* @param[in]   nbrOfBytes  		    :需要校验的数据长度
* @param[in]   checksum  		    :校验的结果
* @retval      int                  :0成功，其他失败
* @note        修改日志 
*               Ver0.0.1:
                    hx-zsj, 2018/08/06, 初始化版本\n 
*/
unsigned char SHT3X_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum)
{
    unsigned char crc;
	crc = SHT3X_CalcCrc(pdata, nbrOfBytes);// calculates 8-Bit checksum
    if(crc != checksum) 
    {   
        return 1;           
    }
    return 0;              
}
/*
* 获取sht30温湿度
* @param[in]   void  		       :无
* @retval      void                :无
* @note        修改日志 
*               Ver0.0.1:
                    hx-zsj, 2018/08/06, 初始化版本\n 
*/
int sht30_get_value(void)
{
    int ret;
    //配置SHT30的寄存器
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();                                   //新建操作I2C句柄
    i2c_master_start(cmd);                                                          //启动I2C
     i2c_master_write_byte(cmd, SHT30_WRITE_ADDR << 1 | READ_BIT, ACK_CHECK_EN);    //发地址+读+检查ack
    i2c_master_read_byte(cmd, &sht30_buf[0], ACK_VAL);                               //读取数据+回复ack
    i2c_master_read_byte(cmd, &sht30_buf[1], ACK_VAL);                               //读取数据+回复ack
    i2c_master_read_byte(cmd, &sht30_buf[2], ACK_VAL);                               //读取数据+回复ack
    i2c_master_read_byte(cmd, &sht30_buf[3], ACK_VAL);                               //读取数据+回复ack
    i2c_master_read_byte(cmd, &sht30_buf[4], ACK_VAL);                               //读取数据+回复ack
    i2c_master_read_byte(cmd, &sht30_buf[5], NACK_VAL);                              //读取数据+不回复ack
    i2c_master_stop(cmd);                                                           //停止I2C
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);        //I2C发送
    if(ret!=ESP_OK)
    {
        return ret;
    }
    i2c_cmd_link_delete(cmd);                                                       //删除I2C句柄
    //校验读出来的数据，算法参考sht30 datasheet
    if( (!SHT3X_CheckCrc(sht30_buf,2,sht30_buf[2])) && (!SHT3X_CheckCrc(sht30_buf+3,2,sht30_buf[5])) )
    {
        ret = ESP_OK;//成功
    }
    else
    {
        ret = 1;
    }
    return ret;
}

static const char* DEMO_TAG = "IBEACON_DEMO";
extern esp_ble_ibeacon_vendor_t vendor_config;

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

#if (IBEACON_MODE == IBEACON_RECEIVER)
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

#elif (IBEACON_MODE == IBEACON_SENDER)
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
#endif


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
#if (IBEACON_MODE == IBEACON_SENDER)
        esp_ble_gap_start_advertising(&ble_adv_params);
#endif
        break;
    }
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
#if (IBEACON_MODE == IBEACON_RECEIVER)
        //the unit of the duration is second, 0 means scan permanently
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
#endif
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Scan start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //adv start complete event to indicate adv start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Adv start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            /* Search for BLE iBeacon Packet */
            if (esp_ble_is_ibeacon_packet(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len)){
                esp_ble_ibeacon_t *ibeacon_data = (esp_ble_ibeacon_t*)(scan_result->scan_rst.ble_adv);
                ESP_LOGI(DEMO_TAG, "----------iBeacon Found----------");
                esp_log_buffer_hex("IBEACON_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN );
                esp_log_buffer_hex("IBEACON_DEMO: Proximity UUID:", ibeacon_data->ibeacon_vendor.proximity_uuid, ESP_UUID_LEN_128);
                
                uint16_t major = ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.major);
                uint16_t minor = ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.minor);
                ESP_LOGI(DEMO_TAG, "Major: 0x%04x (%d)", major, major);
                ESP_LOGI(DEMO_TAG, "Minor: 0x%04x (%d)", minor, minor);
                ESP_LOGI(DEMO_TAG, "Measured power (RSSI at a 1m distance):%d dbm", ibeacon_data->ibeacon_vendor.measured_power);
                ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
            }
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Scan stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(DEMO_TAG, "Stop scan successfully");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(DEMO_TAG, "Stop adv successfully");
        }
        break;

    default:
        break;
    }
}


void ble_ibeacon_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG, "gap register error: %s", esp_err_to_name(status));
        return;
    }

}

void ble_ibeacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_ibeacon_appRegister();
}

int getLength(int num){
    int n = num, count = 0;
    while (n)
    {
        count++;
        n /= 10;
    }
    return count;
}

void app_main(void)
{
    
    // OLED ===========================================================================
    oled_init();
    oled_claer();
    oled_show_str(16,0,  "Temp", &Font_7x10, 1);
    oled_show_str(80,0,  "Hum", &Font_7x10, 1);
    oled_show_str(16,30, "TVOC", &Font_7x10, 1);
    oled_show_str(80,30, "eCO2", &Font_7x10, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);



    esp_log_level_set("SGP30-LIB", ESP_LOG_VERBOSE);
    xTaskCreate(main_task, "sgp30_main_test_task", 1024 * 2, (void *)0, 10, NULL);

            ESP_MAJOR=6535;
            ESP_MINOR=6535;

            vendor_config.major = ENDIAN_CHANGE_U16(ESP_MAJOR);
            vendor_config.minor = ENDIAN_CHANGE_U16(ESP_MINOR);

                ESP_ERROR_CHECK(nvs_flash_init());
                ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
                esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
                esp_bt_controller_init(&bt_cfg);
                esp_bt_controller_enable(ESP_BT_MODE_BLE);

                ble_ibeacon_init();
                esp_ble_gap_set_device_name("AirTHD_ESP32");

                /* set scan parameters */
            #if (IBEACON_MODE == IBEACON_RECEIVER)
                esp_ble_gap_set_scan_params(&ble_scan_params);

            #elif (IBEACON_MODE == IBEACON_SENDER)
                esp_ble_ibeacon_t ibeacon_adv_data;
                esp_err_t status = esp_ble_config_ibeacon_data (&vendor_config, &ibeacon_adv_data);
                if (status == ESP_OK){
                    esp_ble_gap_config_adv_data_raw((uint8_t*)&ibeacon_adv_data, sizeof(ibeacon_adv_data));
                }
                else {
                    ESP_LOGE(DEMO_TAG, "Config iBeacon data failed: %s\n", esp_err_to_name(status));
                }
            #endif


    // SHT30 ===========================================================================
    //i2c_init();                         //I2C初始化
    sht30_init();                       //sht30初始化
    vTaskDelay(100/portTICK_RATE_MS);   //延时100ms
    while(1)
    {
        if(sht30_get_value()==ESP_OK)   //获取温湿度
        {
            //算法参考sht30 datasheet
            g_temp    =( ( (  (sht30_buf[0]*256) +sht30_buf[1]) *175   )/65535.0  -45  );
            g_rh  =  ( ( (sht30_buf[3]*256) + (sht30_buf[4]) )*100/65535.0) ;
            ESP_LOGI("SHT30", "temp:%4.2f C   hum:%4.2f %%RH \r\n", g_temp, g_rh); //℃打印出来是乱码
            
            //温度
            char str_temp[20];
            sprintf(str_temp,"%f", g_temp);
            char str_temp_2[7];
            strncpy(str_temp_2, str_temp, 5);
            strncpy(str_temp_2+5, "C\0", 2);
            //湿度
            char str_hum[20];
            sprintf(str_hum,"%f", g_rh);
            char str_hum_2[8];
            strncpy(str_hum_2, str_hum, 5);
            strncpy(str_hum_2+5, "%\0", 2);
            //TVOC
            char str_TVOC[20];
            sprintf(str_TVOC,"%d", TVOC);
            int tvoc_x = 6;
            switch(getLength(TVOC)){
                case 1:tvoc_x = 6+10;  strncpy(str_TVOC+1, "ppm\0", 4);break;
                case 2:tvoc_x = 6+7;   strncpy(str_TVOC+2, "ppm\0", 4);break;
                case 3:tvoc_x = 6+5;   strncpy(str_TVOC+3, "ppm\0", 4);break;
                case 4:tvoc_x = 6+3;   strncpy(str_TVOC+4, "ppm\0", 4);break;
                default:tvoc_x = 6;    strncpy(str_TVOC+sizeof(eCO2), "ppm\0", 4);break;
            }
            //eCO2
            char str_eCO2[20];
            sprintf(str_eCO2,"%d", eCO2);
            int eco2_x = 6;
            switch(getLength(eCO2)){
                case 1:eco2_x = 6+10;  strncpy(str_eCO2+1, "ppb\0", 4);break;
                case 2:eco2_x = 6+7;   strncpy(str_eCO2+2, "ppb\0", 4);break;
                case 3:eco2_x = 6+5;   strncpy(str_eCO2+3, "ppb\0", 4);break;
                case 4:eco2_x = 6+3;   strncpy(str_eCO2+4, "ppb\0", 4);break;
                default:eco2_x = 6;    strncpy(str_eCO2+sizeof(eCO2), "ppb\0", 4);break;
            }

            oled_claer_2();
            oled_show_str(12,15, str_temp_2, &Font_7x10, 1);
            oled_show_str(76,15, str_hum_2 ,&Font_7x10,1);

            oled_claer_4();
            if(eCO2 == 0){
                oled_show_str(6,45, "Calibrating...", &Font_7x10, 1);
            }else{
                oled_show_str(tvoc_x,45, str_TVOC, &Font_7x10, 1);
                oled_show_str(eco2_x+64,45, str_eCO2 ,&Font_7x10,1);
            }

            //BLE========================================================================================
            int TH_major = (int)(g_temp*10)*100+(int)g_rh;
            int CT_minor = (int)(eCO2/10)*100 + (int)TVOC/10;
            ESP_LOGI("TH_major", "%d \r\n", TH_major);
            ESP_LOGI("CT_minor", "%d \r\n", CT_minor);
            
            ESP_MAJOR=TH_major;
            ESP_MINOR=CT_minor;

            vendor_config.major = ENDIAN_CHANGE_U16(ESP_MAJOR);
            vendor_config.minor = ENDIAN_CHANGE_U16(ESP_MINOR);


            #if (IBEACON_MODE == IBEACON_RECEIVER)
                esp_ble_gap_set_scan_params(&ble_scan_params);

            #elif (IBEACON_MODE == IBEACON_SENDER)
                esp_ble_ibeacon_t ibeacon_adv_data;
                esp_err_t status = esp_ble_config_ibeacon_data (&vendor_config, &ibeacon_adv_data);
                if (status == ESP_OK){
                    esp_ble_gap_config_adv_data_raw((uint8_t*)&ibeacon_adv_data, sizeof(ibeacon_adv_data));
                }
                else {
                    ESP_LOGE(DEMO_TAG, "Config iBeacon data failed: %s\n", esp_err_to_name(status));
                }
            #endif



        }            
        vTaskDelay(2000/portTICK_RATE_MS);
    }



}

