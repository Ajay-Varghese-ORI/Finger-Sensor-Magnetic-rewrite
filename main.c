/**
 * @file    main.c
 * @brief   A program to read the magnetic sensor values of MLX90393SLW and store them to
 *          internal variable. This was used on  the finger sensor board for the ORI Hand 
 *          project.
 * @details The finger sensor board has 6 magnetic sensor chips, eeprom and LEDs and Lux 
 *          sensors. This was originally designed by Marco Pontin for use in a FDM hand.
 *          This code repurposes that PCB to get general readings from the magnetic sensors
 *          and store them in a buffer for later development of the PolyJet hand for the 
 *          Royal Society Summer Festival. The code is written in C and uses the MAX32660 
 *          microcontroller. You will need an MAX3262PICO board to upload and debug this code.
 * @author  Ajay Varghese
 * 
 * @date    2026-04-07
*/

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "gpio.h"
#include "i2c.h"
#include "mlx90393_driver.h"

/***** Definitions *****/
#define GPIO_PORT MXC_GPIO0     // The MAX32660 has only one GPIO port (GPIO0)
#define LED_PIN MXC_GPIO_PIN_11 // Pin 11 on GPIO0 is connected to the LEDs
#define I2C_MASTER MXC_I2C1     // We will use I2C1 to communicate with the sensors (P0.2 and P0.3)
#define I2C_MASTER_FREQ 100000  // I2C frequency (100 kHz)


/***** Globals *****/
mxc_gpio_cfg_t gpio_led;        // LED GPIO configuration structure
uint8_t i2c_addr_list[10];      // List to store detected I2C addresses
mlx90393_t magnetic_sensors[6]; // Array of driver handles for the 6 magnetic sensors


/***** Functions *****/
/**
 * @brief   Configures the GPIO for the LED.
 * @param   led_config Pointer to the GPIO configuration structure for the LED. (mxc_gpio_cfg_t *)
 * @return  None
*/
void setup_led(mxc_gpio_cfg_t *led_config);

/**
 * @brief   Sets up the I2C communications with the MLX90393SLW magnetic sensor
 *          It also performs a scan to see what addresses are on the I2C bus.
 * @param   i2c_addr_list Pointer to an array where the detected I2C addresses will be stored. (uint8_t *)
 * @details For the finger sensor the expected addresses are the following:
 *          - 0x0C (MLX90393SLW sensor 1) Distal Phalanx
 *          - 0x0D (MLX90393SLW sensor 2) Distal Phalanx
 *          - 0x0E (MLX90393SLW sensor 3) Distal Phalanx
 *          - 0x0F (MLX90393SLW sensor 4) Distal Phalanx
 *          - 0x10 (MLX90393SLW sensor 5) Proximal Phalanx
 *          - 0x11 (MLX90393SLW sensor 6) Middle Phalanx
 *          - 0x44 (Lux Sensor)
 *          - 0x45 (Lux Sensor)
 *          - 0x46 (Lux Sensor)
 *          - 0x50 (EEPROM)
 * @return  int 0 if successful, error code otherwise
*/
int setup_i2c(uint8_t *i2c_addr_list);

/**
 * @brief   Sets up the magnetic sensor drivers
 * @param   sensors Pointer to the array of magnetic sensor driver handles. (mlx90393_t *)
 * @param   i2c_addr_list Pointer to the array of I2C addresses. (uint8_t *)
 * @param   num_sensors Number of magnetic sensors to initialize. (int)
 * @return  int 0 if successful, error code otherwise
*/
int setup_magnetic_sensors(mlx90393_t *sensors, uint8_t *i2c_addr_list, int num_sensors);

/**
 * @brief Takes a single reading from one magnetic sensor and converts it to uT
 * @param   sensor Pointer to the magnetic sensor driver handle. (mlx90393_t *)
 * @param   x Pointer to store the x-axis reading in microteslas. (float *)
 * @param   y Pointer to store the y-axis reading in microteslas. (float *)
 * @param   z Pointer to store the z-axis reading in microteslas. (float *)
 * @return  int 0 if successful, error code otherwise
*/
int read_magnetic_sensor(mlx90393_t *sensor, float *x, float *y, float *z);


extern void initialise_monitor_handles(void);

// *****************************************************************************
int main(void)
{
    // Used for semihosting debug output
	initialise_monitor_handles();

    // Set up the LED GPIO
    setup_led(&gpio_led);

    // Set up I2C and scan for devices
    setup_i2c(i2c_addr_list);

    uint8_t i2c_addr_list[6] = {0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11};

    // Set up the magnetic sensor drivers
    setup_magnetic_sensors(magnetic_sensors, i2c_addr_list, 6);

    while (1)
    {
        // Set the LED pin high
        MXC_GPIO_OutSet(GPIO_PORT, LED_PIN);

        // take a reading from 0x10
        float x, y, z;
        read_magnetic_sensor(&magnetic_sensors[4], &x, &y, &z);
        printf("Magnetic Sensor 5 (0x10) Readings: X = %.2f uT, Y = %.2f uT, Z = %.2f uT\r\n", \
            (double)x, (double)y, (double)z);
        
        // Small delay to make the LED blink visible and to avoid overwhelming the I2C bus with readings
        MXC_Delay(MXC_DELAY_MSEC(200));

        // Set the LED pin low
        MXC_GPIO_OutClr(GPIO_PORT, LED_PIN);
        MXC_Delay(MXC_DELAY_MSEC(200));
    }
}

// *****************************************************************************

void setup_led(mxc_gpio_cfg_t *led_config)
{
    led_config->port = GPIO_PORT;
    led_config->mask = LED_PIN;
    led_config->pad = MXC_GPIO_PAD_NONE;
    led_config->func = MXC_GPIO_FUNC_OUT;
    led_config->vssel = MXC_GPIO_VSSEL_VDDIO;
    led_config->drvstr = MXC_GPIO_DRVSTR_0;

    // Configure the GPIO for the LED
    if (MXC_GPIO_Config(led_config) != E_NO_ERROR) // E_NO_ERROR is 0 
    {
        printf("LED GPIO configuration failed\r\n");
        while (1) {} // Loop indefinitely if configuration fails
    }
}

int setup_i2c(uint8_t *i2c_addr_list)
{
    // Error variable to track if any I2C operations fail
    int error = 0;

    // Initialize I2C master
    error = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (error != E_NO_ERROR) {
        printf("I2C master initialization failed\r\n");
        return error;
    }

    // Set I2C frequency
    MXC_I2C_SetFrequency(I2C_MASTER, I2C_MASTER_FREQ);

    // Scan for I2C devices and store their addresses in i2c_addr_list
    int addr_index = 0;

    // Try to read from the address to see if a device responds
    for (uint8_t addr = 8; addr < 120; addr++)
    {
        // Create an I2C request structure for the transaction
        mxc_i2c_req_t req;
        req.i2c = I2C_MASTER;
        req.addr = addr;
        req.tx_buf = NULL;
        req.tx_len = 0;
        req.rx_buf = NULL;
        req.rx_len = 0;
        req.restart = 0;
        req.callback = NULL;

        // Send the I2C transaction
        error = MXC_I2C_MasterTransaction(&req);
        
        // Check for errors. If there is no error, it means a device responded at this address
        if (error == E_NO_ERROR) // E_NO_ERROR is 0, so this means the transaction was successful
        {
            // Device responded at this address, store it in the list
            i2c_addr_list[addr_index++] = addr;
            printf("Device found at I2C address: 0x%02X\r\n", addr);
        }

        // delay between address checks to avoid overwhelming the bus (200 ms)
        MXC_Delay(MXC_DELAY_MSEC(20));

    }

    if (addr_index == 0)
    {
        printf("No I2C devices found\r\n");
    }

    return 0; // Return 0 to indicate success
}

int setup_magnetic_sensors(mlx90393_t *sensors, uint8_t *i2c_addr_list, int num_sensors)
{
    for (int i = 0; i < num_sensors; i++)
    {
        // Using the driver to initialize the sensor
        int error = mlx90393_init(&sensors[i], I2C_MASTER, i2c_addr_list[i]);
        if (error != E_NO_ERROR)
        {
            printf("Failed to initialize magnetic sensor at I2C address: 0x%02X\r\n", i2c_addr_list[i]);
            return error; // Return the error code if initialization fails
        }

        printf("Initialized magnetic sensor %d with I2C address: 0x%02X\r\n", i+1, sensors[i].i2c_addr);

        // Small delay after initialization
        MXC_Delay(MXC_DELAY_MSEC(100));

    }

    return 0; // Return 0 to indicate success
}

int read_magnetic_sensor(mlx90393_t *sensor, float *x, float *y, float *z)
{
    int error = 0;

    // Get the current configuration of the sensor to use for conversion
    mlx90393_config_t config;
    error = mlx90393_get_config(sensor, &config);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Take a single measurement
    mlx90393_measurement_t measurement;
    error = mlx90393_start_single_measurement(sensor, 0x0F, &measurement.status);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Wait for the measurement to be ready (this is a simple delay, in a real application you might want to check the status byte instead)
    MXC_Delay(MXC_DELAY_MSEC(100));

    // Read the raw measurement data
    error = mlx90393_read_measurement_raw(sensor, 0x0F, &measurement);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Convert the raw measurement data to microteslas
    error = mlx90393_convert_raw_to_uT(&config, &measurement, x, y, z);
    if (error != E_NO_ERROR) {
        return error;
    }

    return 0; // Return 0 to indicate success
}
