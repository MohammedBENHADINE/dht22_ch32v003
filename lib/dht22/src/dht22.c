/**
 * @file    dht22.c
 * @brief   DHT22 Temperature & Humidity Sensor Driver Implementation
 * @details Uses TIM1 in PWM Input mode to measure pulse widths
 *          - Channel 1: Captures on rising edges (measures period)
 *          - Channel 2: Captures on falling edges (measures HIGH pulse width)
 *          Timer is configured for 1µs resolution (24MHz / 24 = 1MHz)
 */

#include "dht22.h"

/* ========================================================================
 * Helper Macros
 * ======================================================================== */
#define DHT22_OUT_LOW(gpio, pin)    GPIO_ResetBits(gpio, pin)
#define DHT22_READ_DATA(gpio, pin)  GPIO_ReadInputDataBit(gpio, pin)

/* DHT22 Protocol Timing Constants */
#define DHT22_PULSE_THRESHOLD_US    50  // Threshold: <50µs = '0', >50µs = '1'

/* ========================================================================
 * Global Variables
 * ======================================================================== */
volatile DHT22_DataTypeDef dht22 = {0};

/* Debug capture array for pulse width analysis */
volatile uint32_t debug_captures[45] = {0};
volatile uint8_t debug_count = 0;

/* ========================================================================
 * Private Function Prototypes
 * ======================================================================== */
static void DHT22_PinOut(GPIO_TypeDef *GPIOx, uint16_t pin);
static void DHT22_PinIn(GPIO_TypeDef *GPIOx, uint16_t pin);
static void Input_Capture_Init(uint16_t arr, uint16_t psc);

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief   Initialize DHT22 timer (TIM1) for PWM Input capture
 * @return  DHT22_OK
 */
int DHT22_init(void) {
    // Configure TIM1: ARR=0xFFFF, PSC=23 → 24MHz/(23+1) = 1MHz = 1µs resolution
    Input_Capture_Init(0xFFFF, 23);
    return DHT22_OK;
}

/**
 * @brief   Read temperature and humidity from DHT22 sensor
 * @param   GPIOx: GPIO port
 * @param   pin: GPIO pin mask
 * @param   out: Pointer to store sensor data
 * @return  DHT22_OK on success, error code otherwise
 */
int DHT22_Read(GPIO_TypeDef *GPIOx, uint16_t pin, DHT22_Data *out) {
    if (!out) {
        return DHT22_ERROR_PARAM;
    }

    /* ====================================================================
     * Step 1: Reset state and disable timer
     * ==================================================================== */
    dht22.index = 0;
    dht22.ready = 0;
    memset((void*)dht22.bits, 0, sizeof(dht22.bits));
    
    TIM_Cmd(TIM1, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update);

    /* ====================================================================
     * Step 2: Send start signal to DHT22
     * Protocol: MCU pulls line LOW for ≥1ms, then releases
     * ==================================================================== */
    DHT22_PinOut(GPIOx, pin);           // Configure as output
    DHT22_OUT_LOW(GPIOx, pin);          // Pull LOW
    Delay_Ms(1);                        // Hold for 1ms

    /* ====================================================================
     * Step 3: Wait for DHT22 response sequence
     * Protocol timeline:
     *   1. MCU releases line → goes HIGH (pull-up)
     *   2. DHT22 pulls LOW for ~80µs (response starts)
     *   3. DHT22 pulls HIGH for ~80µs (response continues)
     *   4. DHT22 sends 40 data bits
     * ==================================================================== */
    DHT22_PinIn(GPIOx, pin);            // Configure as input (line goes HIGH)

    // Wait for DHT22 to pull line LOW (response acknowledgment)
    uint32_t timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 1) {
        if (++timeout > 1000000) {
            return DHT22_ERROR_TIMEOUT_LOW;  // Sensor not responding
        }
    }

    // Wait for DHT22 to pull line HIGH (response continues ~80µs)
    timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 0) {
        if (++timeout > 1000000) {
            return DHT22_ERROR_TIMEOUT_HIGH;  // Incomplete response
        }
    }

    // Wait for line to go LOW (first data bit's LOW phase starts)
    timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 1) {
        if (++timeout > 1000000) {
            return DHT22_ERROR_TIMEOUT_DATA;  // No data transmission
        }
    }

    // Wait for line to go HIGH (first data bit's HIGH pulse begins)
    timeout = 0;
    while (DHT22_READ_DATA(GPIOx, pin) == 0) {
        if (++timeout > 1000000) {
            return DHT22_ERROR_TIMEOUT_DATA;  // Incomplete bit transmission
        }
    }

    /* ====================================================================
     * Step 4: Enable timer to capture data bits
     * Clear any pending flags from GPIO transitions during setup
     * ==================================================================== */
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);

    /* ====================================================================
     * Step 5: Wait for data capture to complete
     * DHT22 sends 40 bits at ~100µs per bit = ~5ms total
     * ==================================================================== */
    Delay_Ms(5);

    // Check if all 40 bits were received
    if (!dht22.ready) {
        TIM_Cmd(TIM1, DISABLE);
        return DHT22_ERROR_TIMEOUT_DATA;  // Incomplete transmission
    }

    /* ====================================================================
     * Step 6: Debug output (optional - can be removed for production)
     * ==================================================================== */
    /*
    printf("DHT22 Debug: Captured %d pulses\r\n", debug_count);
    printf("Pulse widths (µs): ");
    for (int i = 0; i < debug_count && i < 40; i++) {
        printf("%lu ", debug_captures[i]);
    }
    printf("\r\n");
    debug_count = 0;
    */

    // Disable timer
    TIM_Cmd(TIM1, DISABLE);

    /* ====================================================================
     * Step 7: Parse bits into bytes
     * DHT22 sends 5 bytes: [HUM_H][HUM_L][TEMP_H][TEMP_L][CHECKSUM]
     * ==================================================================== */
    uint8_t data[5] = {0};
    for (int byte = 0; byte < 5; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            data[byte] <<= 1;
            data[byte] |= dht22.bits[byte * 8 + bit];
        }
    }

    /* ====================================================================
     * Step 8: Validate checksum
     * Checksum = (HUM_H + HUM_L + TEMP_H + TEMP_L) & 0xFF
     * ==================================================================== */
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (checksum != data[4]) {
        printf("DHT22 Error: Checksum mismatch (got 0x%02X, expected 0x%02X)\r\n", 
               data[4], checksum);
        printf("Raw data: HUM=%02X%02X TEMP=%02X%02X CHK=%02X\r\n",
               data[0], data[1], data[2], data[3], data[4]);
        return DHT22_ERROR_CHECKSUM;
    }

    /* ====================================================================
     * Step 9: Extract temperature and humidity
     * Both values are sent as 16-bit integers multiplied by 10
     * ==================================================================== */
    
    // Humidity: 0-100% (value × 10, e.g., 999 = 99.9%)
    uint16_t raw_hum = (data[0] << 8) | data[1];
    out->humidity = raw_hum;

    // Temperature: -40 to +80°C (value × 10, e.g., 234 = 23.4°C)
    // MSB of TEMP_H is sign bit
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

/* ========================================================================
 * Private Functions
 * ======================================================================== */

/**
 * @brief   Configure GPIO pin as open-drain output
 */
static void DHT22_PinOut(GPIO_TypeDef *GPIOx, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
 * @brief   Configure GPIO pin as floating input
 */
static void DHT22_PinIn(GPIO_TypeDef *GPIOx, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
 * @brief   Initialize TIM1 for PWM Input capture mode
 * @param   arr: Auto-reload value (counter period)
 * @param   psc: Prescaler value (clock divider - 1)
 * @note    PWM Input mode automatically configures:
 *          - CH1: Rising edge trigger (measures period)
 *          - CH2: Falling edge trigger (measures pulse width)
 */
static void Input_Capture_Init(uint16_t arr, uint16_t psc) {
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef        NVIC_InitStructure = {0};

    /* Disable timer and clear pending interrupts */
    TIM_Cmd(TIM1, DISABLE);
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update);

    /* Enable TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* Configure timer base: 24MHz / (23+1) = 1MHz → 1µs resolution */
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    /* Configure input capture for PWM Input mode */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    /* PWM Input mode auto-configures both CH1 and CH2 */
    TIM_PWMIConfig(TIM1, &TIM_ICInitStructure);

    /* Configure NVIC for timer interrupts */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure slave mode: timer resets on each rising edge */
    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

    /* Keep timer disabled until needed */
    TIM_Cmd(TIM1, DISABLE);
}

/* ========================================================================
 * Interrupt Service Routine
 * ======================================================================== */

/**
 * @brief   TIM1 Capture/Compare interrupt handler
 * @note    Captures HIGH pulse width on each rising edge
 *          - Pulse < 50µs → bit = '0'
 *          - Pulse > 50µs → bit = '1'
 */
void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_CC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1)) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

        /* Read pulse width from CH2 (measures HIGH duration) */
        uint32_t high_us = TIM_GetCapture2(TIM1);

        /* Optional : Store for debugging */
        /*
        if (debug_count < 40) {
            debug_captures[debug_count++] = high_us;
        }
        */
        /* Decode bit based on pulse width */
        if (dht22.index < DHT22_MAX_BITS) {
            dht22.bits[dht22.index++] = (high_us > DHT22_PULSE_THRESHOLD_US) ? 1 : 0;

            /* Check if all 40 bits received */
            if (dht22.index >= DHT22_MAX_BITS) {
                TIM_Cmd(TIM1, DISABLE);
                TIM_ITConfig(TIM1, TIM_IT_CC1 | TIM_IT_CC2, DISABLE);
                dht22.ready = 1;
            }
        }
    }
}