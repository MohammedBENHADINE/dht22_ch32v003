#include "debug.h"
#include "dht22.h"

/* ========================================================================
 * Configuration Constants
 * ======================================================================== */
#define UART_BAUD_RATE              115200
#define DHT22_GPIO_PORT             GPIOD
#define DHT22_GPIO_PIN              GPIO_Pin_2
#define SENSOR_READ_INTERVAL_MS     5000
#define TEMPERATURE_PRECISION       1
#define HUMIDITY_PRECISION          1

/* ========================================================================
 * Private Function Prototypes
 * ======================================================================== */
static void System_Init(void);
static void GPIO_Init_DHT22(void);
static void Display_SensorData(const DHT22_Data *sensor_data);
static void Display_ErrorMessage(int error_code);

/* ========================================================================
 * Main Application
 * ======================================================================== */

/**
 * @brief   Main application entry point
 * @return  Never returns
 */
int main(void) {
    DHT22_Data sensor_data = {0};
    int read_result;

    // Initialize system peripherals
    System_Init();

    printf("========================================\r\n");
    printf("  DHT22 Temperature & Humidity Sensor  \r\n");
    printf("========================================\r\n\r\n");

    // Main application loop
    while (1) {
        
        // Read sensor data
        read_result = DHT22_Read(DHT22_GPIO_PORT, DHT22_GPIO_PIN, &sensor_data);

        // Process result
        if (read_result == DHT22_OK) {
            Display_SensorData(&sensor_data);
        } else {
            Display_ErrorMessage(read_result);
        }
            
        //printf("Simulated Temperature: %d°C  |  Humidity: %d \r\n", (int)25.3f, (int)60.5f);
        printf("Next read in %d seconds...\r\n\r\n", SENSOR_READ_INTERVAL_MS / 1000);

        // Wait before next reading (DHT22 requires minimum 2s between reads)
        Delay_Ms(SENSOR_READ_INTERVAL_MS);
    }
}

/* ========================================================================
 * Private Functions
 * ======================================================================== */

/**
 * @brief   Initializes system peripherals and clocks
 */
static void System_Init(void) {
    // Configure interrupt priority grouping
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    // Initialize delay functions
    Delay_Init();

    // Initialize UART for debugging
    USART_Printf_Init(UART_BAUD_RATE);

    // Initialize GPIO for DHT22
    GPIO_Init_DHT22();
}

/**
 * @brief   Initializes GPIO peripheral for DHT22 sensor
 */
static void GPIO_Init_DHT22(void) {
    // Enable GPIO clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    DHT22_init();
}

/**
 * @brief   Displays sensor readings in formatted output
 * @param   sensor_data: Pointer to sensor data structure
 */
static void Display_SensorData(const DHT22_Data *sensor_data) {
    if (!sensor_data) {
        return;
    }

    // Display with decimal precision
    printf("Temperature: %d °C  |  Humidity: %d \r\n",
           sensor_data->temperature,
           sensor_data->humidity);
}

/**
 * @brief   Displays error message based on error code
 * @param   error_code: DHT22 error code
 */
static void Display_ErrorMessage(int error_code) {
    switch (error_code) {
        case DHT22_ERROR_CHECKSUM:
            printf("[ERROR] DHT22 checksum failed - data corruption detected\r\n");
            break;
        case DHT22_ERROR_TIMEOUT_LOW:
            printf("[ERROR] DHT22 timeout waiting for LOW signal - check sensor connection\r\n");
            break;
        case DHT22_ERROR_TIMEOUT_HIGH:
            printf("[ERROR] DHT22 timeout waiting for HIGH signal - check sensor connection\r\n");
            break;
        case DHT22_ERROR_TIMEOUT_DATA:
            printf("[ERROR] DHT22 timeout waiting for data - incomplete transmission\r\n");
            break;
        case DHT22_ERROR_PARAM:
            printf("[ERROR] DHT22 invalid parameter - null data pointer\r\n");
            break; 

        default:
            printf("[ERROR] DHT22 unknown error (code: %d)\r\n", error_code);
            break;
    }
}