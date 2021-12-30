#ifndef __SENSOR_H
#define __SENSOR_H

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
extern float tempData, humData, tvocData, eco2Data;
//I2C 
#define I2C_MASTER_NUM      I2C_NUM_0           //I2C_1
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
static unsigned char sht30_buf[6]={0};       //用于接收SHT30返回数据
//SGP30
#define SGP30_ADDR  (0x58) 
#define SGP30_FEATURESET 0x0020    /**< The required set for this library */
#define SGP30_CRC8_POLYNOMIAL 0x31 /**< Seed for SGP30's CRC polynomial */
#define SGP30_CRC8_INIT 0xFF       /**< Init value for CRC */
#define SGP30_WORD_LEN 2           /**< 2 bytes per word */
#define NULL_REG 0xFF
typedef int8_t (*sgp30_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
typedef int8_t (*sgp30_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
typedef struct sgp30_dev {
    /**< The 48-bit serial number, this value is set when you call sgp30_init */
    uint16_t serial_number[3];
    /**< The last measurement of the IAQ-calculated Total Volatile Organic
            Compounds in ppb. This value is set when you call IAQmeasure() **/
    uint16_t TVOC;
    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This
            value is set when you call IAQmeasure() */
    uint16_t eCO2;
    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This
            value is set when you call IAQmeasureRaw() */
    uint16_t raw_H2;
    /**< The last measurement of the IAQ-calculated equivalent CO2 in ppm. This 
            value is set when you call IAQmeasureRaw */
    uint16_t raw_ethanol;
    /**< Interface pointer, used to store I2C address of the device */
    void *intf_ptr;
    /**< I2C read driver function pointer */
    sgp30_read_fptr_t i2c_read;
    /**< I2C write driver function pointer */
    sgp30_write_fptr_t i2c_write;
} sgp30_dev_t;



void Sensor_init();
void Sensor_task();
int sht30_init(void);
unsigned char SHT3X_CalcCrc(unsigned char *data, unsigned char nbrOfBytes);
unsigned char SHT3X_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum);
int sht30_get_value(void);


int8_t main_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t main_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
void sgp30_init(sgp30_dev_t *sensor, sgp30_read_fptr_t user_i2c_read, sgp30_write_fptr_t user_i2c_write);
void sgp30_softreset(sgp30_dev_t *sensor);
void sgp30_IAQ_init(sgp30_dev_t *sensor);
void sgp30_IAQ_measure(sgp30_dev_t *sensor);
void sgp30_IAQ_measure_raw(sgp30_dev_t *sensor);
void sgp30_get_IAQ_baseline(sgp30_dev_t *sensor, uint16_t *eco2_base, uint16_t *tvoc_base);
void sgp30_set_IAQ_baseline(sgp30_dev_t *sensor, uint16_t eco2_base, uint16_t tvoc_base);
void sgp30_set_humidity(sgp30_dev_t *sensor, uint32_t absolute_humidity);



#endif