/**
 * @file    mlx90393_driver.h
 * @brief   A blocking I2C driver for the MLX90393 magnetic sensor on the MAX32660.
 * @details This driver is written in C and is intended for use with the Analog Devices /
 *          Maxim PeriphDrivers I2C API. The file includes the public type definitions,
 *          macros and function prototypes at the top because this driver was intentionally
 *          written as a single .c file with no separate header.
 *
 *          The driver supports:
 *          - Selecting the I2C peripheral instance and 7-bit slave address
 *          - Reading and writing the MLX90393 volatile registers
 *          - Setting and reading back the user configuration fields
 *          - Starting single, burst and wake-on-change measurements
 *          - Reading raw TXYZ measurement data
 *          - Reading back a full configuration snapshot
 *
 *          The MLX90393 always ACKs I2C commands, even when a command is not accepted.
 *          Because of that the returned status byte must always be checked.
 *
 * @author  Ajay Varghese
 *
 * @date    2026-04-07
*/

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "i2c.h"

/* -----------------------------------------------------------------------------
 * Public result codes
 * -------------------------------------------------------------------------- */
#ifndef E_NO_ERROR
#define E_NO_ERROR (0)
#endif

#define MLX90393_OK                         E_NO_ERROR
#define MLX90393_ERROR_NULL_PTR            (-2000)
#define MLX90393_ERROR_BAD_PARAM           (-2001)
#define MLX90393_ERROR_STATUS              (-2002)
#define MLX90393_ERROR_RESPONSE_LENGTH     (-2003)
#define MLX90393_ERROR_UNSUPPORTED_CONFIG  (-2004)

/* -----------------------------------------------------------------------------
 * MLX90393 command bytes
 * -------------------------------------------------------------------------- */
#define MLX90393_CMD_NOP   (0x00) // No operation (used for reading status byte)
#define MLX90393_CMD_SB    (0x10) // Start burst mode measurement with selected channels
#define MLX90393_CMD_SW    (0x20) // Start wake-on-change measurement with selected channels
#define MLX90393_CMD_SM    (0x30) // Start polled single measurement with selected channels
#define MLX90393_CMD_RM    (0x40) // Read measurement result of previous
#define MLX90393_CMD_WR    (0x60) // Write register
#define MLX90393_CMD_HS    (0xE0) // Memory store
#define MLX90393_CMD_EX    (0x80) // Exit measurement mode and return to idle mode
#define MLX90393_CMD_HR    (0xD0) // Memory recall
#define MLX90393_CMD_RT    (0xF0) // Reset (soft reset, returns to idle mode)
#define MLX90393_CMD_RR    (0x50) // Read register

/* -----------------------------------------------------------------------------
 * Register addresses in the customer area
 * -------------------------------------------------------------------------- */
#define MLX90393_REG_CONF1         (0x00) // Contains the Z-axis series, gain selection, hall configuration and some reserved bits
#define MLX90393_REG_CONF2         (0x01) // Contains the interrupt, communication, burst and wake-on-change configuration fields
#define MLX90393_REG_CONF3         (0x02) // Contains the resolution, digital filter and oversampling configuration fields
#define MLX90393_REG_CONF4         (0x03) // Contains the temperature compensation thresholds
#define MLX90393_REG_OFFSET_X      (0x04) // Contains the X-axis offset compensation value
#define MLX90393_REG_OFFSET_Y      (0x05) // Contains the Y-axis offset compensation value
#define MLX90393_REG_OFFSET_Z      (0x06) // Contains the Z-axis offset compensation value
#define MLX90393_REG_WOXY_THRESH   (0x07) // Contains the XY wake-on-change threshold 
#define MLX90393_REG_WOZ_THRESH    (0x08) // Contains the Z wake-on-change threshold
#define MLX90393_REG_WOT_THRESH    (0x09) // Contains the temperature wake-on-change threshold

/* -----------------------------------------------------------------------------
 * Status byte masks
 * -------------------------------------------------------------------------- */
#define MLX90393_STATUS_BURST_MODE   (0x80) // Set to 1 if the device is currently in burst mode, cleared if in single measurement or idle mode
#define MLX90393_STATUS_WOC_MODE     (0x40) // Set to 1 if the device is currently in wake-on-change mode, cleared if in single measurement or idle mode
#define MLX90393_STATUS_SM_MODE      (0x20) // Set to 1 if the device is currently in single measurement mode, cleared if in burst or wake-on-change mode
#define MLX90393_STATUS_ERROR        (0x10) // Set to 1 if an error has occurred, cleared if no error
#define MLX90393_STATUS_SED          (0x08) // Set to 1 if a signal edge detection has occurred, cleared if no edge detected
#define MLX90393_STATUS_RS           (0x04) // Set to 1 if the device is in reset state, cleared if not in reset state
#define MLX90393_STATUS_D1           (0x02)
#define MLX90393_STATUS_D0           (0x01)
#define MLX90393_STATUS_D_MASK       (0x03) // D1 and D0 together indicate the number of data bytes to be read after the status byte in a measurement read command response (00 = 3 bytes, 01 = 5 bytes, 10 = 7 bytes, 11 = reserved)

/* -----------------------------------------------------------------------------
 * CONF1 bit masks
 * -------------------------------------------------------------------------- */
#define MLX90393_CONF1_Z_SERIES_MASK          (0x0080)
#define MLX90393_CONF1_GAIN_SEL_MASK          (0x0070)
#define MLX90393_CONF1_GAIN_SEL_SHIFT         (4)
#define MLX90393_CONF1_HALLCONF_MASK          (0x000F)
#define MLX90393_CONF1_HALLCONF_SHIFT         (0)
#define MLX90393_CONF1_ANA_RESERVED_LOW_MASK  (0xFE00)
#define MLX90393_CONF1_ANA_RESERVED_LOW_SHIFT (9)
#define MLX90393_CONF1_BIST_MASK              (0x0100)
#define MLX90393_CONF1_BIST_SHIFT             (8)

/* -----------------------------------------------------------------------------
 * CONF2 bit masks
 * -------------------------------------------------------------------------- */
#define MLX90393_CONF2_TRIG_INT_MASK            (0x8000)
#define MLX90393_CONF2_TRIG_INT_SHIFT           (15)
#define MLX90393_CONF2_COMM_MODE_MASK           (0x6000)
#define MLX90393_CONF2_COMM_MODE_SHIFT          (13)
#define MLX90393_CONF2_WOC_DIFF_MASK            (0x1000)
#define MLX90393_CONF2_WOC_DIFF_SHIFT           (12)
#define MLX90393_CONF2_EXT_TRIG_MASK            (0x0800)
#define MLX90393_CONF2_EXT_TRIG_SHIFT           (11)
#define MLX90393_CONF2_TCMP_EN_MASK             (0x0400)
#define MLX90393_CONF2_TCMP_EN_SHIFT            (10)
#define MLX90393_CONF2_BURST_SEL_MASK           (0x03C0)
#define MLX90393_CONF2_BURST_SEL_SHIFT          (6)
#define MLX90393_CONF2_BURST_DATA_RATE_MASK     (0x003F)
#define MLX90393_CONF2_BURST_DATA_RATE_SHIFT    (0)

/* -----------------------------------------------------------------------------
 * CONF3 bit masks
 * -------------------------------------------------------------------------- */
#define MLX90393_CONF3_OSR2_MASK             (0x1800)
#define MLX90393_CONF3_OSR2_SHIFT            (11)
#define MLX90393_CONF3_RES_Z_MASK            (0x0600)
#define MLX90393_CONF3_RES_Z_SHIFT           (9)
#define MLX90393_CONF3_RES_Y_MASK            (0x0180)
#define MLX90393_CONF3_RES_Y_SHIFT           (7)
#define MLX90393_CONF3_RES_X_MASK            (0x0060)
#define MLX90393_CONF3_RES_X_SHIFT           (5)
#define MLX90393_CONF3_DIG_FILT_MASK         (0x001C)
#define MLX90393_CONF3_DIG_FILT_SHIFT        (2)
#define MLX90393_CONF3_OSR_MASK              (0x0003)
#define MLX90393_CONF3_OSR_SHIFT             (0)

/* -----------------------------------------------------------------------------
 * CONF4 bit masks
 * -------------------------------------------------------------------------- */
#define MLX90393_CONF4_SENS_TC_HT_MASK       (0xFF00)
#define MLX90393_CONF4_SENS_TC_HT_SHIFT      (8)
#define MLX90393_CONF4_SENS_TC_LT_MASK       (0x00FF)
#define MLX90393_CONF4_SENS_TC_LT_SHIFT      (0)

/* -----------------------------------------------------------------------------
 * Command argument masks
 * The bit order for measurement selection is ZYXT.
 * -------------------------------------------------------------------------- */
#define MLX90393_CHANNEL_T    (0x01)
#define MLX90393_CHANNEL_X    (0x02)
#define MLX90393_CHANNEL_Y    (0x04)
#define MLX90393_CHANNEL_Z    (0x08)
#define MLX90393_CHANNEL_ALL  (0x0F)

/* -----------------------------------------------------------------------------
 * Recommended default values
 * -------------------------------------------------------------------------- */
#define MLX90393_DEFAULT_HALLCONF   (0x0C)


/***** Type Definitions *****/

/**
 * @brief   MLX90393 analog gain selections.
*/
typedef enum
{
    MLX90393_GAIN_5X = 0,
    MLX90393_GAIN_4X = 1,
    MLX90393_GAIN_3X = 2,
    MLX90393_GAIN_2P5X = 3,
    MLX90393_GAIN_2X = 4,
    MLX90393_GAIN_1P67X = 5,
    MLX90393_GAIN_1P33X = 6,
    MLX90393_GAIN_1X = 7
} mlx90393_gain_t;

/**
 * @brief   MLX90393 axis resolution selections.
*/
typedef enum
{
    MLX90393_RES_16 = 0,
    MLX90393_RES_17 = 1,
    MLX90393_RES_18 = 2,
    MLX90393_RES_19 = 3
} mlx90393_resolution_t;

/**
 * @brief   MLX90393 digital filter selections.
*/
typedef enum
{
    MLX90393_FILTER_0 = 0,
    MLX90393_FILTER_1 = 1,
    MLX90393_FILTER_2 = 2,
    MLX90393_FILTER_3 = 3,
    MLX90393_FILTER_4 = 4,
    MLX90393_FILTER_5 = 5,
    MLX90393_FILTER_6 = 6,
    MLX90393_FILTER_7 = 7
} mlx90393_filter_t;

/**
 * @brief   MLX90393 magnetic oversampling selections.
*/
typedef enum
{
    MLX90393_OSR_0 = 0,
    MLX90393_OSR_1 = 1,
    MLX90393_OSR_2 = 2,
    MLX90393_OSR_3 = 3
} mlx90393_osr_t;

/**
 * @brief   MLX90393 temperature oversampling selections.
*/
typedef enum
{
    MLX90393_OSR2_0 = 0,
    MLX90393_OSR2_1 = 1,
    MLX90393_OSR2_2 = 2,
    MLX90393_OSR2_3 = 3
} mlx90393_osr2_t;

/**
 * @brief   MLX90393 communication mode selections.
*/
typedef enum
{
    MLX90393_COMM_MODE_BOTH_0 = 0,
    MLX90393_COMM_MODE_BOTH_1 = 1,
    MLX90393_COMM_MODE_SPI_ONLY = 2,
    MLX90393_COMM_MODE_I2C_ONLY = 3
} mlx90393_comm_mode_t;

/**
 * @brief   Axis identifiers for per-axis configuration fields.
*/
typedef enum
{
    MLX90393_AXIS_X = 0,
    MLX90393_AXIS_Y = 1,
    MLX90393_AXIS_Z = 2
} mlx90393_axis_t;

/**
 * @brief   Wake-on-change threshold selectors.
*/
typedef enum
{
    MLX90393_WOC_THRESHOLD_XY = 0,
    MLX90393_WOC_THRESHOLD_Z = 1,
    MLX90393_WOC_THRESHOLD_T = 2
} mlx90393_woc_threshold_t;

/**
 * @brief   Decoded status byte fields.
*/
typedef struct
{
    uint8_t raw_status;
    uint8_t burst_mode;
    uint8_t woc_mode;
    uint8_t sm_mode;
    uint8_t error;
    uint8_t sed;
    uint8_t rs;
    uint8_t data_byte_count;
} mlx90393_status_t;

/**
 * @brief   MLX90393 measurement container.
*/
typedef struct
{
    int16_t t;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t valid_channels;
    uint8_t status;
} mlx90393_measurement_t;

/**
 * @brief   Full configuration snapshot of the user-accessible fields.
*/
typedef struct
{
    uint8_t z_series;
    uint8_t gain;
    uint8_t hallconf;
    uint8_t ana_reserved_low;
    uint8_t bist;

    uint8_t trig_int;
    uint8_t comm_mode;
    uint8_t woc_diff;
    uint8_t ext_trig;
    uint8_t tcmp_en;
    uint8_t burst_sel;
    uint8_t burst_data_rate;

    uint8_t osr2;
    uint8_t res_x;
    uint8_t res_y;
    uint8_t res_z;
    uint8_t dig_filt;
    uint8_t osr;

    uint8_t sens_tc_ht;
    uint8_t sens_tc_lt;

    uint16_t offset_x;
    uint16_t offset_y;
    uint16_t offset_z;

    uint16_t woxy_threshold;
    uint16_t woz_threshold;
    uint16_t wot_threshold;
} mlx90393_config_t;

/**
 * @brief   Driver handle containing the selected I2C port and slave address.
*/
typedef struct
{
    mxc_i2c_regs_t *i2c;
    uint8_t i2c_addr;
} mlx90393_t;


/***** Public Function Prototypes *****/
/**
 * @brief   Initialises the MLX90393 driver handle and performs a soft reset sequence.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   i2c_port Pointer to the MAX32660 I2C peripheral to be used. (mxc_i2c_regs_t *)
 * @param   i2c_addr 7-bit I2C address of the target sensor. (uint8_t)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_init(mlx90393_t *dev, mxc_i2c_regs_t *i2c_port, uint8_t i2c_addr);

/**
 * @brief   Changes the I2C peripheral used by an already created driver handle.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   i2c_port Pointer to the MAX32660 I2C peripheral to be used. (mxc_i2c_regs_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_set_i2c_port(mlx90393_t *dev, mxc_i2c_regs_t *i2c_port);

/**
 * @brief   Changes the 7-bit I2C address used by an already created driver handle.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   i2c_addr 7-bit I2C address of the target sensor. (uint8_t)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_set_i2c_address(mlx90393_t *dev, uint8_t i2c_addr);

/**
 * @brief   Sends the EX command to force the device back to idle mode.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_exit_mode(mlx90393_t *dev, uint8_t *status);

/**
 * @brief   Sends the RT command to perform a warm reset.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_reset(mlx90393_t *dev, uint8_t *status);

/**
 * @brief   Copies the non-volatile memory into volatile memory.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_memory_recall(mlx90393_t *dev, uint8_t *status);

/**
 * @brief   Stores the current volatile memory content into non-volatile memory.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @details VDD must be at least 3.3 V for a reliable HS store operation.
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_memory_store(mlx90393_t *dev, uint8_t *status);

/**
 * @brief   Reads a 16-bit register from the MLX90393 volatile memory.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   reg Register address in the customer memory map. (uint8_t)
 * @param   value Pointer to the returned register value. (uint16_t *)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_read_register(mlx90393_t *dev, uint8_t reg, uint16_t *value, uint8_t *status);

/**
 * @brief   Writes a 16-bit register in the MLX90393 volatile memory.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   reg Register address in the customer memory map. (uint8_t)
 * @param   value Value to be written to the register. (uint16_t)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_write_register(mlx90393_t *dev, uint8_t reg, uint16_t value, uint8_t *status);

/**
 * @brief   Decodes the returned status byte into separate fields.
 * @param   status_byte Raw status byte from the sensor. (uint8_t)
 * @param   decoded_status Pointer to the decoded status structure. (mlx90393_status_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_decode_status(uint8_t status_byte, mlx90393_status_t *decoded_status);

/**
 * @brief   Starts a single measurement using the provided ZYXT channel mask.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   zyxt_mask Channel mask in ZYXT order. (uint8_t)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_start_single_measurement(mlx90393_t *dev, uint8_t zyxt_mask, uint8_t *status);

/**
 * @brief   Starts burst mode using the provided ZYXT channel mask.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   zyxt_mask Channel mask in ZYXT order. (uint8_t)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_start_burst_mode(mlx90393_t *dev, uint8_t zyxt_mask, uint8_t *status);

/**
 * @brief   Starts wake-on-change mode using the provided ZYXT channel mask.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   zyxt_mask Channel mask in ZYXT order. (uint8_t)
 * @param   status Pointer to the returned status byte. (uint8_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_start_woc_mode(mlx90393_t *dev, uint8_t zyxt_mask, uint8_t *status);

/**
 * @brief   Reads raw measurement values for the selected channels.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   zyxt_mask Channel mask in ZYXT order. (uint8_t)
 * @param   measurement Pointer to the returned measurement structure. (mlx90393_measurement_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_read_measurement_raw(mlx90393_t *dev, uint8_t zyxt_mask, mlx90393_measurement_t *measurement);

/**
 * @brief   Converts raw magnetic readings to microtesla using the current configuration.
 * @param   config Pointer to a configuration snapshot. (mlx90393_config_t *)
 * @param   measurement Pointer to the raw measurement values. (mlx90393_measurement_t *)
 * @param   x_uT Pointer to the returned X axis value in microtesla. (float *)
 * @param   y_uT Pointer to the returned Y axis value in microtesla. (float *)
 * @param   z_uT Pointer to the returned Z axis value in microtesla. (float *)
 * @details This helper is intended for TCMP_EN = 0 and HALLCONF = 0x0 or 0xC.
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_convert_raw_to_uT(const mlx90393_config_t *config,
                               const mlx90393_measurement_t *measurement,
                               float *x_uT, float *y_uT, float *z_uT);

/**
 * @brief   Reads back every user-accessible configuration field into a snapshot structure.
 * @param   dev Pointer to the driver handle. (mlx90393_t *)
 * @param   config Pointer to the returned configuration structure. (mlx90393_config_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_get_config(mlx90393_t *dev, mlx90393_config_t *config);

/**
 * @brief   Prints the current configuration snapshot using printf.
 * @param   config Pointer to the configuration structure to be printed. (mlx90393_config_t *)
 * @return  int 0 if successful, error code otherwise
*/
int mlx90393_print_config(const mlx90393_config_t *config);

/* -----------------------------------------------------------------------------
 * Configuration setter and getter prototypes
 * -------------------------------------------------------------------------- */
int mlx90393_set_z_series(mlx90393_t *dev, uint8_t enable);
int mlx90393_get_z_series(mlx90393_t *dev, uint8_t *enable);

int mlx90393_set_bist(mlx90393_t *dev, uint8_t enable);
int mlx90393_get_bist(mlx90393_t *dev, uint8_t *enable);

int mlx90393_set_gain(mlx90393_t *dev, mlx90393_gain_t gain);
int mlx90393_get_gain(mlx90393_t *dev, mlx90393_gain_t *gain);

int mlx90393_set_hallconf(mlx90393_t *dev, uint8_t hallconf);
int mlx90393_get_hallconf(mlx90393_t *dev, uint8_t *hallconf);

int mlx90393_set_trig_int_sel(mlx90393_t *dev, uint8_t enable_interrupt_output);
int mlx90393_get_trig_int_sel(mlx90393_t *dev, uint8_t *enable_interrupt_output);

int mlx90393_set_comm_mode(mlx90393_t *dev, mlx90393_comm_mode_t comm_mode);
int mlx90393_get_comm_mode(mlx90393_t *dev, mlx90393_comm_mode_t *comm_mode);

int mlx90393_set_woc_diff(mlx90393_t *dev, uint8_t use_previous_sample);
int mlx90393_get_woc_diff(mlx90393_t *dev, uint8_t *use_previous_sample);

int mlx90393_set_ext_trig(mlx90393_t *dev, uint8_t enable);
int mlx90393_get_ext_trig(mlx90393_t *dev, uint8_t *enable);

int mlx90393_set_tcmp_en(mlx90393_t *dev, uint8_t enable);
int mlx90393_get_tcmp_en(mlx90393_t *dev, uint8_t *enable);

int mlx90393_set_burst_sel(mlx90393_t *dev, uint8_t zyxt_mask);
int mlx90393_get_burst_sel(mlx90393_t *dev, uint8_t *zyxt_mask);

int mlx90393_set_burst_data_rate(mlx90393_t *dev, uint8_t burst_data_rate);
int mlx90393_get_burst_data_rate(mlx90393_t *dev, uint8_t *burst_data_rate);

int mlx90393_set_osr2(mlx90393_t *dev, mlx90393_osr2_t osr2);
int mlx90393_get_osr2(mlx90393_t *dev, mlx90393_osr2_t *osr2);

int mlx90393_set_resolution(mlx90393_t *dev, mlx90393_axis_t axis, mlx90393_resolution_t resolution);
int mlx90393_get_resolution(mlx90393_t *dev, mlx90393_axis_t axis, mlx90393_resolution_t *resolution);

int mlx90393_set_dig_filt(mlx90393_t *dev, mlx90393_filter_t dig_filt);
int mlx90393_get_dig_filt(mlx90393_t *dev, mlx90393_filter_t *dig_filt);

int mlx90393_set_osr(mlx90393_t *dev, mlx90393_osr_t osr);
int mlx90393_get_osr(mlx90393_t *dev, mlx90393_osr_t *osr);

int mlx90393_set_sens_tc_ht(mlx90393_t *dev, uint8_t sens_tc_ht);
int mlx90393_get_sens_tc_ht(mlx90393_t *dev, uint8_t *sens_tc_ht);

int mlx90393_set_sens_tc_lt(mlx90393_t *dev, uint8_t sens_tc_lt);
int mlx90393_get_sens_tc_lt(mlx90393_t *dev, uint8_t *sens_tc_lt);

int mlx90393_set_offset(mlx90393_t *dev, mlx90393_axis_t axis, int16_t offset);
int mlx90393_get_offset(mlx90393_t *dev, mlx90393_axis_t axis, int16_t *offset);

int mlx90393_set_woc_threshold(mlx90393_t *dev, mlx90393_woc_threshold_t threshold_id, uint16_t threshold);
int mlx90393_get_woc_threshold(mlx90393_t *dev, mlx90393_woc_threshold_t threshold_id, uint16_t *threshold);


/***** Private Function Prototypes *****/
int mlx90393_i2c_transceive(mlx90393_t *dev,
                                   uint8_t *tx_buf,
                                   uint32_t tx_len,
                                   uint8_t *rx_buf,
                                   uint32_t rx_len);

int mlx90393_send_simple_command(mlx90393_t *dev, uint8_t command, uint8_t *status);

int mlx90393_read_modify_write(mlx90393_t *dev,
                                      uint8_t reg,
                                      uint16_t mask,
                                      uint8_t shift,
                                      uint16_t value);

int mlx90393_get_field(mlx90393_t *dev,
                              uint8_t reg,
                              uint16_t mask,
                              uint8_t shift,
                              uint16_t *value);

uint8_t mlx90393_channel_count(uint8_t zyxt_mask);

int16_t mlx90393_adjust_signed_output(int16_t raw_value, mlx90393_resolution_t resolution);
