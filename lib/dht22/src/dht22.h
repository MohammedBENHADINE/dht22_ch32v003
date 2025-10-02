#ifndef DHT22_H
#define DHT22_H

#include "debug.h" // or your MCU header that defines GPIO_TypeDef, GPIO_Pin, etc.
#include <stdbool.h>
/* Return codes */
#define DHT22_OK            0
#define DHT22_ERROR_TIMEOUT_LOW 1
#define DHT22_ERROR_CHECKSUM 2
#define DHT22_ERROR_TIMEOUT_HIGH 3
#define DHT22_ERROR_TIMEOUT_DATA 4
#define DHT22_ERROR_PARAM 5
typedef struct {
    u16 temperature; /* Celsius */
    u16 humidity;    /* %RH */
} DHT22_Data;

#define DHT22_MAX_BITS      40

typedef struct {
   volatile  uint8_t bits[DHT22_MAX_BITS];
   volatile  uint8_t index;
   volatile  uint8_t ready;
} DHT22_DataTypeDef;

extern volatile DHT22_DataTypeDef dht22;

/* Read function:
 *  GPIOx : GPIO port (e.g. GPIOA)
 *  pin   : pin mask (e.g. GPIO_Pin_0)
 *  out   : pointer to DHT22_Data to fill
 * Returns DHT22_OK on success, otherwise error code.
 */
int DHT22_Read(GPIO_TypeDef *GPIOx, uint16_t pin, DHT22_Data *out);

int DHT22_init(void);

#endif /* DHT22_CH32_H */
