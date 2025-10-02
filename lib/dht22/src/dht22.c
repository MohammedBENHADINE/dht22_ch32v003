#include "dht22.h"

/* Helper macros */
#define DHT22_OUT_LOW(gpio, pin)    GPIO_ResetBits(gpio, pin)
#define DHT22_READ_DATA(gpio, pin)  GPIO_ReadInputDataBit(gpio, pin)

volatile DHT22_DataTypeDef dht22 = {0};

// Global debug array
volatile uint32_t debug_captures[45] = {0};
volatile uint8_t debug_count = 0;



int DHT22_init()
{
    Input_Capture_Init(0xFFFF, 23);
}


/**
 * @brief   Initializes input capture timer for DHT22
 */

void Input_Capture_Init(uint16_t arr, uint16_t psc)
{
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef        NVIC_InitStructure = {0};

    // Disable timer first
    TIM_Cmd(TIM1, DISABLE);
    
    // Clear all pending interrupts
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // **DON'T ENABLE INTERRUPTS YET**
    // TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE); // <-- REMOVE THIS

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    
    // Keep timer disabled
    TIM_Cmd(TIM1, DISABLE);
}

/**
 * @brief   Configures pin as open-drain output
 */
static void DHT22_PinOut(GPIO_TypeDef *GPIOx, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
 * @brief   Configures pin as floating input
 */
static void DHT22_PinIn(GPIO_TypeDef *GPIOx, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
 * @brief   Reads temperature and humidity from DHT22 sensor
 */
int DHT22_Read(GPIO_TypeDef *GPIOx, uint16_t pin, DHT22_Data *out) {
    if (!out) {
        return DHT22_ERROR_PARAM;
    }

    // Reset state
    dht22.index = 0;
    dht22.ready = 0;
    memset((void*)dht22.bits, 0, sizeof(dht22.bits));
    // Disable and clear everything first
    TIM_Cmd(TIM1, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update);



    // Send start signal (minimum 1ms low)
    DHT22_PinOut(GPIOx, pin);
    DHT22_OUT_LOW(GPIOx, pin);
    Delay_Ms(1);

    // Release line and wait for sensor response
    DHT22_PinIn(GPIOx, pin);

    // Wait for sensor to pull line low then high
    uint32_t timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 1) {
        if (++timeout > 1000000) return DHT22_ERROR_TIMEOUT_LOW;
    }
    timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 0) {
        if (++timeout > 1000000) return DHT22_ERROR_TIMEOUT_LOW;
    }
    // Wait for line to go LOW again (first data bit's LOW phase)
    timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 1) {
        if (++timeout > 1000000) return DHT22_ERROR_TIMEOUT_HIGH;
    }

        // Wait for line to go LOW again (first data bit's LOW phase)
    timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 0) {
        if (++timeout > 1000000) return DHT22_ERROR_TIMEOUT_HIGH;
    }
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update);

    // **NOW enable interrupts and start timer**
    TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);


    // **DON'T BLOCK HERE** - Use a delay instead
    // DHT22 takes ~5ms to send 40 bits at most
    Delay_Ms(5);
    
    // Check if we got all data
    if (!dht22.ready) {
        TIM_Cmd(TIM1, DISABLE);
        return DHT22_ERROR_TIMEOUT_DATA;
    }

        // After DHT22_Read returns
    printf("Total captures: %d\r\n", debug_count);
    printf("First 5 pulse widths: ");
    for (int i = 0; i < debug_count; i++) {
        printf("%lu ", debug_captures[i]);
    }
    printf("\r\n");
    debug_count = 0;  // Reset for next read

    // Disable timer
    TIM_Cmd(TIM1, DISABLE);

    // Parse bits into bytes
    uint8_t data[5] = {0};
    for (int byte = 0; byte < 5; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            data[byte] <<= 1;
            data[byte] |= dht22.bits[byte * 8 + bit];
        }
    }

    // Validate checksum
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (checksum != data[4]) {
        printf("DHT22 checksum error! Got 0x%02X expected 0x%02X\r\n", 
               data[4], checksum);
        return DHT22_ERROR_CHECKSUM;
    }

    // Extract humidity
    uint16_t raw_hum = (data[0] << 8) | data[1];
    out->humidity = raw_hum;

    // Extract temperature
    uint16_t raw_temp = (data[2] << 8) | data[3];
    if (raw_temp & 0x8000) {
        // Negative temperature
        raw_temp &= 0x7FFF;
        out->temperature = -raw_temp;
    } else {
        out->temperature = raw_temp;
    }

    return DHT22_OK;
}


/**
 * @brief   Timer 1 capture/compare interrupt handler
 * @note    CRITICAL FIX: Check for completion INSIDE the increment condition
 */

void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_CC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1)) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        uint32_t high_us = TIM_GetCapture2(TIM1);
        
        // Store for debugging
        if (debug_count < 40) {
            debug_captures[debug_count++] = high_us;
        }

        // Store the actual data bits (indices 3-42 map to bits 0-39)
        if (dht22.index < DHT22_MAX_BITS)  {
            dht22.bits[dht22.index++] = (high_us > 50) ? 1 : 0;
            
            // Check if we've received all 40 data bits
            if (dht22.index >= DHT22_MAX_BITS) {  // ✅ FIXED: 43 total captures
                TIM_Cmd(TIM1, DISABLE);
                TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);
                dht22.ready = 1;
            }
        }
    }
}