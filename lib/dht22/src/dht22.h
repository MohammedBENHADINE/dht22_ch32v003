/**
 * @file    dht22.h
 * @brief   DHT22 Temperature & Humidity Sensor Driver for CH32V003
 * @note    Uses TIM1 PWM Input mode for precise pulse width measurement
 */

#ifndef DHT22_H
#define DHT22_H

#include "debug.h"
#include <stdbool.h>
#include <string.h>

/* ========================================================================
 * DHT22 Return Codes
 * ======================================================================== */
#define DHT22_OK                    0   // Successful read with valid checksum
#define DHT22_ERROR_TIMEOUT_LOW     1   // Sensor didn't pull line LOW (no response)
#define DHT22_ERROR_TIMEOUT_HIGH    2   // Sensor didn't pull line HIGH (incomplete response)
#define DHT22_ERROR_TIMEOUT_DATA    3   // Incomplete data transmission (< 40 bits)
#define DHT22_ERROR_CHECKSUM        4   // Data corruption detected (checksum mismatch)
#define DHT22_ERROR_PARAM           5   // Invalid parameter (NULL pointer)

/* ========================================================================
 * DHT22 Data Structures
 * ======================================================================== */

/**
 * @brief   DHT22 sensor reading output
 * @note    Values are multiplied by 10 (e.g., 234 = 23.4°C, 999 = 99.9%RH)
 */
typedef struct {
    uint16_t temperature;   // Temperature in tenths of degree Celsius
    uint16_t humidity;      // Relative humidity in tenths of percent
} DHT22_Data;

/**
 * @brief   Internal state for DHT22 data capture via timer interrupt
 */
#define DHT22_MAX_BITS  40

typedef struct {
    volatile uint8_t bits[DHT22_MAX_BITS];  // Captured bit values (0 or 1)
    volatile uint8_t index;                  // Current bit position
    volatile uint8_t ready;                  // Flag: all 40 bits received
} DHT22_DataTypeDef;

extern volatile DHT22_DataTypeDef dht22;

/* ========================================================================
 * Public Function Prototypes
 * ======================================================================== */

/**
 * @brief   Initialize DHT22 timer peripheral (TIM1)
 * @return  DHT22_OK on success
 * @note    Call once during system initialization
 */
int DHT22_init(void);

/**
 * @brief   Read temperature and humidity from DHT22 sensor
 * @param   GPIOx: GPIO port (e.g., GPIOD)
 * @param   pin: GPIO pin mask (e.g., GPIO_Pin_2)
 * @param   out: Pointer to DHT22_Data structure to store results
 * @return  DHT22_OK on success, error code otherwise
 * @note    Minimum 2 seconds between successive reads required by sensor
 */
int DHT22_Read(GPIO_TypeDef *GPIOx, uint16_t pin, DHT22_Data *out);

#endif /* DHT22_H */