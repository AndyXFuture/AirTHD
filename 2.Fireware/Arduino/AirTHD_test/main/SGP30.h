/**
 * @file SGP30.h
 *
 * @author Renato Freitas
 * @author Isabella Bologna
 * 
 * @brief SGP30 library, based on Adafruit's one. 
 * ----> https://www.adafruit.com/product/3709
 * ----> https://github.com/adafruit/Adafruit_SGP30
 *
 * @date 10/2020
 */

#ifndef __SGP30_H__
#define __SGP30_H__

#include <stdio.h>
#include <string.h>

#include <esp_types.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

/***********************************
    Default I2C address
************************************/
#define SGP30_ADDR  (0x58) 


/****** Commands and constants ******/
#define SGP30_FEATURESET 0x0020    /**< The required set for this library */
#define SGP30_CRC8_POLYNOMIAL 0x31 /**< Seed for SGP30's CRC polynomial */
#define SGP30_CRC8_INIT 0xFF       /**< Init value for CRC */
#define SGP30_WORD_LEN 2           /**< 2 bytes per word */

//!!!
#define NULL_REG 0xFF //????? 
//!!!


/*** I2C Driver Function Pointers ***/
typedef int8_t (*sgp30_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

typedef int8_t (*sgp30_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/** SGP30 Main Data Struct */
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


/**
 *  @brief  Setups the hardware and detects a valid SGP30. Initializes I2C
 *          then reads the serialnumber and checks that we are talking to an SGP30.
 */
void sgp30_init(sgp30_dev_t *sensor, sgp30_read_fptr_t user_i2c_read, sgp30_write_fptr_t user_i2c_write);

/**
 *   @brief  Commands the sensor to perform a soft reset using the "General
 *           Call" mode. Take note that this is not sensor specific and all devices that
 *           support the General Call mode on the on the same I2C bus will perform this.
 */
void sgp30_softreset(sgp30_dev_t *sensor);

/**
 *   @brief  Commands the sensor to begin the IAQ algorithm. Must be called
 *           after startup.
 */
void sgp30_IAQ_init(sgp30_dev_t *sensor);

/**
 *  @brief  Commands the sensor to take a single eCO2/VOC measurement. Places
 *          results in sensor.TVOC and sensor.eCO2
 */

void sgp30_IAQ_measure(sgp30_dev_t *sensor);

/**
 *  @brief  Commands the sensor to take a single H2/ethanol raw measurement.
 *          Places results in sensor.raw_H2 and sensor.raw_ethanol
 */
void sgp30_IAQ_measure_raw(sgp30_dev_t *sensor);

/*!
 *   @brief  Request baseline calibration values for both CO2 and TVOC IAQ
 *           calculations. Places results in parameter memory locaitons.
 *   @param  eco2_base
 *           A pointer to a uint16_t which we will save the calibration
 *           value to
 *   @param  tvoc_base
 *           A pointer to a uint16_t which we will save the calibration value to
 */
void sgp30_get_IAQ_baseline(sgp30_dev_t *sensor, uint16_t *eco2_base, uint16_t *tvoc_base);

/**
 *  @brief  Assign baseline calibration values for both CO2 and TVOC IAQ
 *          calculations.
 *  @param  eco2_base
 *          A uint16_t which we will save the calibration value from
 *  @param  tvoc_base
 *          A uint16_t which we will save the calibration value from
 */
void sgp30_set_IAQ_baseline(sgp30_dev_t *sensor, uint16_t eco2_base, uint16_t tvoc_base);

/**
 *  @brief  Set the absolute humidity value [mg/m^3] for compensation to
 *          increase precision of TVOC and eCO2.
 *  @param  absolute_humidity
 *          A uint32_t [mg/m^3] which we will be used for compensation.
 *          If the absolute humidity is set to zero, humidity compensation
 *          will be disabled.
 */
void sgp30_set_humidity(sgp30_dev_t *sensor, uint32_t absolute_humidity);


#endif  // __SGP30_H__