
/**
 * @file    mlx90393_driver.c
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
#include "mlx90393_driver.h"

/***** Globals *****/
/* Lookup tables
 * Index order:
 * [gain][resolution][axis type]
 * axis type index 0 = XY
 * axis type index 1 = Z
*/
const float mlx90393_lsb_lookup[8][4][2] =
{
    //   RES 16,            RES 17,            RES 18,             RES 19
    {{0.805f, 1.468f}, {1.610f, 2.936f}, {3.220f, 5.872f}, {6.440f, 11.744f}}, // Gain 0
    {{0.644f, 1.174f}, {1.288f, 2.349f}, {2.576f, 4.698f}, {5.152f,  9.395f}}, // Gain 1
    {{0.483f, 0.881f}, {0.966f, 1.762f}, {1.932f, 3.523f}, {3.864f,  7.046f}}, // Gain 2
    {{0.403f, 0.734f}, {0.805f, 1.468f}, {1.610f, 2.936f}, {3.220f,  5.872f}}, // Gain 3
    {{0.322f, 0.587f}, {0.644f, 1.174f}, {1.288f, 2.349f}, {2.576f,  4.698f}}, // Gain 4
    {{0.268f, 0.489f}, {0.537f, 0.979f}, {1.073f, 1.957f}, {2.147f,  3.915f}}, // Gain 5
    {{0.215f, 0.391f}, {0.429f, 0.783f}, {0.859f, 1.566f}, {1.717f,  3.132f}}, // Gain 6
    {{0.161f, 0.294f}, {0.322f, 0.587f}, {0.644f, 1.174f}, {1.288f,  2.349f}}  // Gain 7
    //  XY      Z         XY      Z         XY      Z          XY       Z
};


/***** Public Functions *****/

// *****************************************************************************
int mlx90393_init(mlx90393_t *dev, mxc_i2c_regs_t *i2c_port, uint8_t i2c_addr)
{
    int error = 0;
    uint8_t status = 0;

    if ((dev == NULL) || (i2c_port == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    dev->i2c = i2c_port;
    dev->i2c_addr = (i2c_addr & 0x7F);

    // Force the part back to idle mode in case it is already in burst or WOC mode
    error = mlx90393_exit_mode(dev, &status);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Melexis recommends a short wait before the reset command
    MXC_Delay(MXC_DELAY_MSEC(1));

    // Perform a warm reset
    error = mlx90393_reset(dev, &status);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Wait for the start-up sequence to complete
    MXC_Delay(MXC_DELAY_MSEC(2));

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_i2c_port(mlx90393_t *dev, mxc_i2c_regs_t *i2c_port)
{
    if ((dev == NULL) || (i2c_port == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    dev->i2c = i2c_port;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_i2c_address(mlx90393_t *dev, uint8_t i2c_addr)
{
    if (dev == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    dev->i2c_addr = (i2c_addr & 0x7F);

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_exit_mode(mlx90393_t *dev, uint8_t *status)
{
    return mlx90393_send_simple_command(dev, MLX90393_CMD_EX, status);
}

// *****************************************************************************
int mlx90393_reset(mlx90393_t *dev, uint8_t *status)
{
    int error = 0;
    uint8_t command = MLX90393_CMD_RT;
    uint8_t rx_status = 0;

    if (dev == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_i2c_transceive(dev, &command, 1, &rx_status, 1);
    if (error != E_NO_ERROR) {
        return error;
    }

    if (status != NULL) {
        *status = rx_status;
    }

    // The RS bit is expected when the reset is accepted
    if ((rx_status & MLX90393_STATUS_RS) == 0) {
        return MLX90393_ERROR_STATUS;
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_memory_recall(mlx90393_t *dev, uint8_t *status)
{
    int error = 0;

    error = mlx90393_send_simple_command(dev, MLX90393_CMD_HR, status);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Give the part a moment to complete the recall operation
    MXC_Delay(MXC_DELAY_MSEC(2));

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_memory_store(mlx90393_t *dev, uint8_t *status)
{
    int error = 0;

    error = mlx90393_send_simple_command(dev, MLX90393_CMD_HS, status);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Datasheet recommendation is at least 15 ms after HS
    MXC_Delay(MXC_DELAY_MSEC(16));

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_read_register(mlx90393_t *dev, uint8_t reg, uint16_t *value, uint8_t *status)
{
    int error = 0;
    uint8_t tx_buf[2];
    uint8_t rx_buf[3];

    if ((dev == NULL) || (value == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    tx_buf[0] = MLX90393_CMD_RR;
    tx_buf[1] = (uint8_t)(reg << 2);

    error = mlx90393_i2c_transceive(dev, tx_buf, 2, rx_buf, 3);
    if (error != E_NO_ERROR) {
        return error;
    }

    if (status != NULL) {
        *status = rx_buf[0];
    }

    if ((rx_buf[0] & MLX90393_STATUS_ERROR) != 0) {
        return MLX90393_ERROR_STATUS;
    }

    *value = (uint16_t)(((uint16_t)rx_buf[1] << 8) | rx_buf[2]);

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_write_register(mlx90393_t *dev, uint8_t reg, uint16_t value, uint8_t *status)
{
    int error = 0;
    uint8_t tx_buf[4];
    uint8_t rx_status = 0;

    if (dev == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    tx_buf[0] = MLX90393_CMD_WR;
    tx_buf[1] = (uint8_t)((value >> 8) & 0xFF);
    tx_buf[2] = (uint8_t)(value & 0xFF);
    tx_buf[3] = (uint8_t)(reg << 2);

    error = mlx90393_i2c_transceive(dev, tx_buf, 4, &rx_status, 1);
    if (error != E_NO_ERROR) {
        return error;
    }

    if (status != NULL) {
        *status = rx_status;
    }

    if ((rx_status & MLX90393_STATUS_ERROR) != 0) {
        return MLX90393_ERROR_STATUS;
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_decode_status(uint8_t status_byte, mlx90393_status_t *decoded_status)
{
    if (decoded_status == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    decoded_status->raw_status = status_byte;
    decoded_status->burst_mode = ((status_byte & MLX90393_STATUS_BURST_MODE) != 0);
    decoded_status->woc_mode = ((status_byte & MLX90393_STATUS_WOC_MODE) != 0);
    decoded_status->sm_mode = ((status_byte & MLX90393_STATUS_SM_MODE) != 0);
    decoded_status->error = ((status_byte & MLX90393_STATUS_ERROR) != 0);
    decoded_status->sed = ((status_byte & MLX90393_STATUS_SED) != 0);
    decoded_status->rs = ((status_byte & MLX90393_STATUS_RS) != 0);
    decoded_status->data_byte_count = (uint8_t)(2 * (status_byte & MLX90393_STATUS_D_MASK) + 2);

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_start_single_measurement(mlx90393_t *dev, uint8_t zyxt_mask, uint8_t *status)
{
    return mlx90393_send_simple_command(dev, (uint8_t)(MLX90393_CMD_SM | (zyxt_mask & 0x0F)), status);
}

// *****************************************************************************
int mlx90393_start_burst_mode(mlx90393_t *dev, uint8_t zyxt_mask, uint8_t *status)
{
    return mlx90393_send_simple_command(dev, (uint8_t)(MLX90393_CMD_SB | (zyxt_mask & 0x0F)), status);
}

// *****************************************************************************
int mlx90393_start_woc_mode(mlx90393_t *dev, uint8_t zyxt_mask, uint8_t *status)
{
    return mlx90393_send_simple_command(dev, (uint8_t)(MLX90393_CMD_SW | (zyxt_mask & 0x0F)), status);
}

// *****************************************************************************
int mlx90393_read_measurement_raw(mlx90393_t *dev, uint8_t zyxt_mask, mlx90393_measurement_t *measurement)
{
    int error = 0;
    uint8_t tx_buf[1];
    uint8_t rx_buf[9];
    uint8_t expected_bytes = 0;
    uint8_t channel_count = 0;
    uint8_t index = 1;

    if ((dev == NULL) || (measurement == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    channel_count = mlx90393_channel_count(zyxt_mask & 0x0F);
    if (channel_count == 0) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    tx_buf[0] = (uint8_t)(MLX90393_CMD_RM | (zyxt_mask & 0x0F));

    // One status byte plus up to 8 bytes of measurement data
    error = mlx90393_i2c_transceive(dev, tx_buf, 1, rx_buf, (uint32_t)(1 + (2 * channel_count)));
    if (error != E_NO_ERROR) {
        return error;
    }

    if ((rx_buf[0] & MLX90393_STATUS_ERROR) != 0) {
        return MLX90393_ERROR_STATUS;
    }

    expected_bytes = (uint8_t)(2 * (rx_buf[0] & MLX90393_STATUS_D_MASK) + 2);
    if (expected_bytes != (uint8_t)(2 * channel_count)) {
        return MLX90393_ERROR_RESPONSE_LENGTH;
    }

    measurement->t = 0;
    measurement->x = 0;
    measurement->y = 0;
    measurement->z = 0;
    measurement->valid_channels = (zyxt_mask & 0x0F);
    measurement->status = rx_buf[0];

    // The sensor returns the selected data in T, X, Y, Z order
    if ((zyxt_mask & MLX90393_CHANNEL_T) != 0) {
        measurement->t = (int16_t)(((uint16_t)rx_buf[index] << 8) | rx_buf[index + 1]);
        index += 2;
    }

    if ((zyxt_mask & MLX90393_CHANNEL_X) != 0) {
        measurement->x = (int16_t)(((uint16_t)rx_buf[index] << 8) | rx_buf[index + 1]);
        index += 2;
    }

    if ((zyxt_mask & MLX90393_CHANNEL_Y) != 0) {
        measurement->y = (int16_t)(((uint16_t)rx_buf[index] << 8) | rx_buf[index + 1]);
        index += 2;
    }

    if ((zyxt_mask & MLX90393_CHANNEL_Z) != 0) {
        measurement->z = (int16_t)(((uint16_t)rx_buf[index] << 8) | rx_buf[index + 1]);
        index += 2;
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_convert_raw_to_uT(const mlx90393_config_t *config,
                               const mlx90393_measurement_t *measurement,
                               float *x_uT, float *y_uT, float *z_uT)
{
    int16_t x_adjusted = 0;
    int16_t y_adjusted = 0;
    int16_t z_adjusted = 0;

    if ((config == NULL) || (measurement == NULL) || (x_uT == NULL) || (y_uT == NULL) || (z_uT == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    if (config->tcmp_en != 0) {
        return MLX90393_ERROR_UNSUPPORTED_CONFIG;
    }

    // uint8_t hallconf_index = 0;
    // if (config->hallconf == 0x0C) {
    //     hallconf_index = 0;
    // } else if (config->hallconf == 0x00) {
    //     hallconf_index = 1;
    // } else {
    //     return MLX90393_ERROR_UNSUPPORTED_CONFIG;
    // }

    x_adjusted = mlx90393_adjust_signed_output(measurement->x, (mlx90393_resolution_t)config->res_x);
    y_adjusted = mlx90393_adjust_signed_output(measurement->y, (mlx90393_resolution_t)config->res_y);
    z_adjusted = mlx90393_adjust_signed_output(measurement->z, (mlx90393_resolution_t)config->res_z);

    *x_uT = ((float)x_adjusted) * mlx90393_lsb_lookup[config->gain][config->res_x][0];
    *y_uT = ((float)y_adjusted) * mlx90393_lsb_lookup[config->gain][config->res_y][0];
    *z_uT = ((float)z_adjusted) * mlx90393_lsb_lookup[config->gain][config->res_z][1];

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_get_config(mlx90393_t *dev, mlx90393_config_t *config)
{
    int error = 0;
    uint16_t conf1 = 0;
    uint16_t conf2 = 0;
    uint16_t conf3 = 0;
    uint16_t conf4 = 0;

    if ((dev == NULL) || (config == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_CONF1, &conf1, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_CONF2, &conf2, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_CONF3, &conf3, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_CONF4, &conf4, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    config->z_series = (uint8_t)((conf1 & MLX90393_CONF1_Z_SERIES_MASK) >> 7);
    config->gain = (uint8_t)((conf1 & MLX90393_CONF1_GAIN_SEL_MASK) >> MLX90393_CONF1_GAIN_SEL_SHIFT);
    config->hallconf = (uint8_t)((conf1 & MLX90393_CONF1_HALLCONF_MASK) >> MLX90393_CONF1_HALLCONF_SHIFT);
    config->ana_reserved_low = (uint8_t)((conf1 & MLX90393_CONF1_ANA_RESERVED_LOW_MASK) >> MLX90393_CONF1_ANA_RESERVED_LOW_SHIFT);
    config->bist = (uint8_t)((conf1 & MLX90393_CONF1_BIST_MASK) >> MLX90393_CONF1_BIST_SHIFT);

    config->trig_int = (uint8_t)((conf2 & MLX90393_CONF2_TRIG_INT_MASK) >> MLX90393_CONF2_TRIG_INT_SHIFT);
    config->comm_mode = (uint8_t)((conf2 & MLX90393_CONF2_COMM_MODE_MASK) >> MLX90393_CONF2_COMM_MODE_SHIFT);
    config->woc_diff = (uint8_t)((conf2 & MLX90393_CONF2_WOC_DIFF_MASK) >> MLX90393_CONF2_WOC_DIFF_SHIFT);
    config->ext_trig = (uint8_t)((conf2 & MLX90393_CONF2_EXT_TRIG_MASK) >> MLX90393_CONF2_EXT_TRIG_SHIFT);
    config->tcmp_en = (uint8_t)((conf2 & MLX90393_CONF2_TCMP_EN_MASK) >> MLX90393_CONF2_TCMP_EN_SHIFT);
    config->burst_sel = (uint8_t)((conf2 & MLX90393_CONF2_BURST_SEL_MASK) >> MLX90393_CONF2_BURST_SEL_SHIFT);
    config->burst_data_rate = (uint8_t)((conf2 & MLX90393_CONF2_BURST_DATA_RATE_MASK) >> MLX90393_CONF2_BURST_DATA_RATE_SHIFT);

    config->osr2 = (uint8_t)((conf3 & MLX90393_CONF3_OSR2_MASK) >> MLX90393_CONF3_OSR2_SHIFT);
    config->res_x = (uint8_t)((conf3 & MLX90393_CONF3_RES_X_MASK) >> MLX90393_CONF3_RES_X_SHIFT);
    config->res_y = (uint8_t)((conf3 & MLX90393_CONF3_RES_Y_MASK) >> MLX90393_CONF3_RES_Y_SHIFT);
    config->res_z = (uint8_t)((conf3 & MLX90393_CONF3_RES_Z_MASK) >> MLX90393_CONF3_RES_Z_SHIFT);
    config->dig_filt = (uint8_t)((conf3 & MLX90393_CONF3_DIG_FILT_MASK) >> MLX90393_CONF3_DIG_FILT_SHIFT);
    config->osr = (uint8_t)((conf3 & MLX90393_CONF3_OSR_MASK) >> MLX90393_CONF3_OSR_SHIFT);

    config->sens_tc_ht = (uint8_t)((conf4 & MLX90393_CONF4_SENS_TC_HT_MASK) >> MLX90393_CONF4_SENS_TC_HT_SHIFT);
    config->sens_tc_lt = (uint8_t)((conf4 & MLX90393_CONF4_SENS_TC_LT_MASK) >> MLX90393_CONF4_SENS_TC_LT_SHIFT);

    error = mlx90393_read_register(dev, MLX90393_REG_OFFSET_X, &config->offset_x, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_OFFSET_Y, &config->offset_y, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_OFFSET_Z, &config->offset_z, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_WOXY_THRESH, &config->woxy_threshold, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_WOZ_THRESH, &config->woz_threshold, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = mlx90393_read_register(dev, MLX90393_REG_WOT_THRESH, &config->wot_threshold, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_print_config(const mlx90393_config_t *config)
{
    if (config == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    printf("MLX90393 configuration\n");
    printf("  z_series        = %u\n", config->z_series);
    printf("  gain            = %u\n", config->gain);
    printf("  hallconf        = 0x%02X\n", config->hallconf);
    printf("  ana_reserved    = 0x%02X\n", config->ana_reserved_low);
    printf("  bist            = %u\n", config->bist);

    printf("  trig_int_sel    = %u\n", config->trig_int);
    printf("  comm_mode       = %u\n", config->comm_mode);
    printf("  woc_diff        = %u\n", config->woc_diff);
    printf("  ext_trig        = %u\n", config->ext_trig);
    printf("  tcmp_en         = %u\n", config->tcmp_en);
    printf("  burst_sel       = 0x%X\n", config->burst_sel);
    printf("  burst_data_rate = %u\n", config->burst_data_rate);

    printf("  osr2            = %u\n", config->osr2);
    printf("  res_x           = %u\n", config->res_x);
    printf("  res_y           = %u\n", config->res_y);
    printf("  res_z           = %u\n", config->res_z);
    printf("  dig_filt        = %u\n", config->dig_filt);
    printf("  osr             = %u\n", config->osr);

    printf("  sens_tc_ht      = 0x%02X\n", config->sens_tc_ht);
    printf("  sens_tc_lt      = 0x%02X\n", config->sens_tc_lt);

    printf("  offset_x        = 0x%04X\n", config->offset_x);
    printf("  offset_y        = 0x%04X\n", config->offset_y);
    printf("  offset_z        = 0x%04X\n", config->offset_z);

    printf("  woxy_threshold  = 0x%04X\n", config->woxy_threshold);
    printf("  woz_threshold   = 0x%04X\n", config->woz_threshold);
    printf("  wot_threshold   = 0x%04X\n", config->wot_threshold);

    return E_NO_ERROR;
}


/***** Configuration Setters and Getters *****/

// *****************************************************************************
int mlx90393_set_z_series(mlx90393_t *dev, uint8_t enable)
{
    return mlx90393_read_modify_write(dev, MLX90393_REG_CONF1, MLX90393_CONF1_Z_SERIES_MASK, 7, (enable ? 1U : 0U));
}

// *****************************************************************************
int mlx90393_get_z_series(mlx90393_t *dev, uint8_t *enable)
{
    uint16_t value = 0;
    int error = 0;

    if (enable == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev, MLX90393_REG_CONF1, MLX90393_CONF1_Z_SERIES_MASK, 7, &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *enable = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_bist(mlx90393_t *dev, uint8_t enable)
{
    return mlx90393_read_modify_write(dev, MLX90393_REG_CONF1, MLX90393_CONF1_BIST_MASK, MLX90393_CONF1_BIST_SHIFT, (enable ? 1U : 0U));
}

// *****************************************************************************
int mlx90393_get_bist(mlx90393_t *dev, uint8_t *enable)
{
    uint16_t value = 0;
    int error = 0;

    if (enable == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev, MLX90393_REG_CONF1, MLX90393_CONF1_BIST_MASK, MLX90393_CONF1_BIST_SHIFT, &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *enable = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_gain(mlx90393_t *dev, mlx90393_gain_t gain)
{
    if ((uint8_t)gain > 7U) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF1,
                                      MLX90393_CONF1_GAIN_SEL_MASK,
                                      MLX90393_CONF1_GAIN_SEL_SHIFT,
                                      (uint16_t)gain);
}

// *****************************************************************************
int mlx90393_get_gain(mlx90393_t *dev, mlx90393_gain_t *gain)
{
    uint16_t value = 0;
    int error = 0;

    if (gain == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF1,
                               MLX90393_CONF1_GAIN_SEL_MASK,
                               MLX90393_CONF1_GAIN_SEL_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *gain = (mlx90393_gain_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_hallconf(mlx90393_t *dev, uint8_t hallconf)
{
    if (hallconf > 0x0F) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF1,
                                      MLX90393_CONF1_HALLCONF_MASK,
                                      MLX90393_CONF1_HALLCONF_SHIFT,
                                      hallconf);
}

// *****************************************************************************
int mlx90393_get_hallconf(mlx90393_t *dev, uint8_t *hallconf)
{
    uint16_t value = 0;
    int error = 0;

    if (hallconf == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF1,
                               MLX90393_CONF1_HALLCONF_MASK,
                               MLX90393_CONF1_HALLCONF_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *hallconf = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_trig_int_sel(mlx90393_t *dev, uint8_t enable_interrupt_output)
{
    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF2,
                                      MLX90393_CONF2_TRIG_INT_MASK,
                                      MLX90393_CONF2_TRIG_INT_SHIFT,
                                      (enable_interrupt_output ? 1U : 0U));
}

// *****************************************************************************
int mlx90393_get_trig_int_sel(mlx90393_t *dev, uint8_t *enable_interrupt_output)
{
    uint16_t value = 0;
    int error = 0;

    if (enable_interrupt_output == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF2,
                               MLX90393_CONF2_TRIG_INT_MASK,
                               MLX90393_CONF2_TRIG_INT_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *enable_interrupt_output = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_comm_mode(mlx90393_t *dev, mlx90393_comm_mode_t comm_mode)
{
    if ((uint8_t)comm_mode > 3U) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF2,
                                      MLX90393_CONF2_COMM_MODE_MASK,
                                      MLX90393_CONF2_COMM_MODE_SHIFT,
                                      (uint16_t)comm_mode);
}

// *****************************************************************************
int mlx90393_get_comm_mode(mlx90393_t *dev, mlx90393_comm_mode_t *comm_mode)
{
    uint16_t value = 0;
    int error = 0;

    if (comm_mode == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF2,
                               MLX90393_CONF2_COMM_MODE_MASK,
                               MLX90393_CONF2_COMM_MODE_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *comm_mode = (mlx90393_comm_mode_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_woc_diff(mlx90393_t *dev, uint8_t use_previous_sample)
{
    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF2,
                                      MLX90393_CONF2_WOC_DIFF_MASK,
                                      MLX90393_CONF2_WOC_DIFF_SHIFT,
                                      (use_previous_sample ? 1U : 0U));
}

// *****************************************************************************
int mlx90393_get_woc_diff(mlx90393_t *dev, uint8_t *use_previous_sample)
{
    uint16_t value = 0;
    int error = 0;

    if (use_previous_sample == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF2,
                               MLX90393_CONF2_WOC_DIFF_MASK,
                               MLX90393_CONF2_WOC_DIFF_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *use_previous_sample = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_ext_trig(mlx90393_t *dev, uint8_t enable)
{
    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF2,
                                      MLX90393_CONF2_EXT_TRIG_MASK,
                                      MLX90393_CONF2_EXT_TRIG_SHIFT,
                                      (enable ? 1U : 0U));
}

// *****************************************************************************
int mlx90393_get_ext_trig(mlx90393_t *dev, uint8_t *enable)
{
    uint16_t value = 0;
    int error = 0;

    if (enable == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF2,
                               MLX90393_CONF2_EXT_TRIG_MASK,
                               MLX90393_CONF2_EXT_TRIG_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *enable = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_tcmp_en(mlx90393_t *dev, uint8_t enable)
{
    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF2,
                                      MLX90393_CONF2_TCMP_EN_MASK,
                                      MLX90393_CONF2_TCMP_EN_SHIFT,
                                      (enable ? 1U : 0U));
}

// *****************************************************************************
int mlx90393_get_tcmp_en(mlx90393_t *dev, uint8_t *enable)
{
    uint16_t value = 0;
    int error = 0;

    if (enable == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF2,
                               MLX90393_CONF2_TCMP_EN_MASK,
                               MLX90393_CONF2_TCMP_EN_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *enable = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_burst_sel(mlx90393_t *dev, uint8_t zyxt_mask)
{
    if ((zyxt_mask & 0xF0U) != 0U) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF2,
                                      MLX90393_CONF2_BURST_SEL_MASK,
                                      MLX90393_CONF2_BURST_SEL_SHIFT,
                                      (uint16_t)(zyxt_mask & 0x0F));
}

// *****************************************************************************
int mlx90393_get_burst_sel(mlx90393_t *dev, uint8_t *zyxt_mask)
{
    uint16_t value = 0;
    int error = 0;

    if (zyxt_mask == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF2,
                               MLX90393_CONF2_BURST_SEL_MASK,
                               MLX90393_CONF2_BURST_SEL_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *zyxt_mask = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_burst_data_rate(mlx90393_t *dev, uint8_t burst_data_rate)
{
    if (burst_data_rate > 0x3F) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF2,
                                      MLX90393_CONF2_BURST_DATA_RATE_MASK,
                                      MLX90393_CONF2_BURST_DATA_RATE_SHIFT,
                                      (uint16_t)burst_data_rate);
}

// *****************************************************************************
int mlx90393_get_burst_data_rate(mlx90393_t *dev, uint8_t *burst_data_rate)
{
    uint16_t value = 0;
    int error = 0;

    if (burst_data_rate == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF2,
                               MLX90393_CONF2_BURST_DATA_RATE_MASK,
                               MLX90393_CONF2_BURST_DATA_RATE_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *burst_data_rate = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_osr2(mlx90393_t *dev, mlx90393_osr2_t osr2)
{
    if ((uint8_t)osr2 > 3U) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF3,
                                      MLX90393_CONF3_OSR2_MASK,
                                      MLX90393_CONF3_OSR2_SHIFT,
                                      (uint16_t)osr2);
}

// *****************************************************************************
int mlx90393_get_osr2(mlx90393_t *dev, mlx90393_osr2_t *osr2)
{
    uint16_t value = 0;
    int error = 0;

    if (osr2 == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF3,
                               MLX90393_CONF3_OSR2_MASK,
                               MLX90393_CONF3_OSR2_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *osr2 = (mlx90393_osr2_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_resolution(mlx90393_t *dev, mlx90393_axis_t axis, mlx90393_resolution_t resolution)
{
    uint16_t mask = 0;
    uint8_t shift = 0;

    if ((axis > MLX90393_AXIS_Z) || ((uint8_t)resolution > 3U)) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    switch (axis)
    {
        case MLX90393_AXIS_X:
            mask = MLX90393_CONF3_RES_X_MASK;
            shift = MLX90393_CONF3_RES_X_SHIFT;
            break;

        case MLX90393_AXIS_Y:
            mask = MLX90393_CONF3_RES_Y_MASK;
            shift = MLX90393_CONF3_RES_Y_SHIFT;
            break;

        case MLX90393_AXIS_Z:
            mask = MLX90393_CONF3_RES_Z_MASK;
            shift = MLX90393_CONF3_RES_Z_SHIFT;
            break;

        default:
            return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev, MLX90393_REG_CONF3, mask, shift, (uint16_t)resolution);
}

// *****************************************************************************
int mlx90393_get_resolution(mlx90393_t *dev, mlx90393_axis_t axis, mlx90393_resolution_t *resolution)
{
    uint16_t mask = 0;
    uint8_t shift = 0;
    uint16_t value = 0;
    int error = 0;

    if (resolution == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    switch (axis)
    {
        case MLX90393_AXIS_X:
            mask = MLX90393_CONF3_RES_X_MASK;
            shift = MLX90393_CONF3_RES_X_SHIFT;
            break;

        case MLX90393_AXIS_Y:
            mask = MLX90393_CONF3_RES_Y_MASK;
            shift = MLX90393_CONF3_RES_Y_SHIFT;
            break;

        case MLX90393_AXIS_Z:
            mask = MLX90393_CONF3_RES_Z_MASK;
            shift = MLX90393_CONF3_RES_Z_SHIFT;
            break;

        default:
            return MLX90393_ERROR_BAD_PARAM;
    }

    error = mlx90393_get_field(dev, MLX90393_REG_CONF3, mask, shift, &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *resolution = (mlx90393_resolution_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_dig_filt(mlx90393_t *dev, mlx90393_filter_t dig_filt)
{
    if ((uint8_t)dig_filt > 7U) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF3,
                                      MLX90393_CONF3_DIG_FILT_MASK,
                                      MLX90393_CONF3_DIG_FILT_SHIFT,
                                      (uint16_t)dig_filt);
}

// *****************************************************************************
int mlx90393_get_dig_filt(mlx90393_t *dev, mlx90393_filter_t *dig_filt)
{
    uint16_t value = 0;
    int error = 0;

    if (dig_filt == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF3,
                               MLX90393_CONF3_DIG_FILT_MASK,
                               MLX90393_CONF3_DIG_FILT_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *dig_filt = (mlx90393_filter_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_osr(mlx90393_t *dev, mlx90393_osr_t osr)
{
    if ((uint8_t)osr > 3U) {
        return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF3,
                                      MLX90393_CONF3_OSR_MASK,
                                      MLX90393_CONF3_OSR_SHIFT,
                                      (uint16_t)osr);
}

// *****************************************************************************
int mlx90393_get_osr(mlx90393_t *dev, mlx90393_osr_t *osr)
{
    uint16_t value = 0;
    int error = 0;

    if (osr == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF3,
                               MLX90393_CONF3_OSR_MASK,
                               MLX90393_CONF3_OSR_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *osr = (mlx90393_osr_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_sens_tc_ht(mlx90393_t *dev, uint8_t sens_tc_ht)
{
    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF4,
                                      MLX90393_CONF4_SENS_TC_HT_MASK,
                                      MLX90393_CONF4_SENS_TC_HT_SHIFT,
                                      sens_tc_ht);
}

// *****************************************************************************
int mlx90393_get_sens_tc_ht(mlx90393_t *dev, uint8_t *sens_tc_ht)
{
    uint16_t value = 0;
    int error = 0;

    if (sens_tc_ht == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF4,
                               MLX90393_CONF4_SENS_TC_HT_MASK,
                               MLX90393_CONF4_SENS_TC_HT_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *sens_tc_ht = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_sens_tc_lt(mlx90393_t *dev, uint8_t sens_tc_lt)
{
    return mlx90393_read_modify_write(dev,
                                      MLX90393_REG_CONF4,
                                      MLX90393_CONF4_SENS_TC_LT_MASK,
                                      MLX90393_CONF4_SENS_TC_LT_SHIFT,
                                      sens_tc_lt);
}

// *****************************************************************************
int mlx90393_get_sens_tc_lt(mlx90393_t *dev, uint8_t *sens_tc_lt)
{
    uint16_t value = 0;
    int error = 0;

    if (sens_tc_lt == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_get_field(dev,
                               MLX90393_REG_CONF4,
                               MLX90393_CONF4_SENS_TC_LT_MASK,
                               MLX90393_CONF4_SENS_TC_LT_SHIFT,
                               &value);
    if (error != E_NO_ERROR) {
        return error;
    }

    *sens_tc_lt = (uint8_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_offset(mlx90393_t *dev, mlx90393_axis_t axis, int16_t offset)
{
    uint8_t reg = 0;

    switch (axis)
    {
        case MLX90393_AXIS_X:
            reg = MLX90393_REG_OFFSET_X;
            break;

        case MLX90393_AXIS_Y:
            reg = MLX90393_REG_OFFSET_Y;
            break;

        case MLX90393_AXIS_Z:
            reg = MLX90393_REG_OFFSET_Z;
            break;

        default:
            return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_write_register(dev, reg, (uint16_t)offset, NULL);
}

// *****************************************************************************
int mlx90393_get_offset(mlx90393_t *dev, mlx90393_axis_t axis, int16_t *offset)
{
    uint16_t value = 0;
    uint8_t reg = 0;
    int error = 0;

    if (offset == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    switch (axis)
    {
        case MLX90393_AXIS_X:
            reg = MLX90393_REG_OFFSET_X;
            break;

        case MLX90393_AXIS_Y:
            reg = MLX90393_REG_OFFSET_Y;
            break;

        case MLX90393_AXIS_Z:
            reg = MLX90393_REG_OFFSET_Z;
            break;

        default:
            return MLX90393_ERROR_BAD_PARAM;
    }

    error = mlx90393_read_register(dev, reg, &value, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    *offset = (int16_t)value;

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_set_woc_threshold(mlx90393_t *dev, mlx90393_woc_threshold_t threshold_id, uint16_t threshold)
{
    uint8_t reg = 0;

    switch (threshold_id)
    {
        case MLX90393_WOC_THRESHOLD_XY:
            reg = MLX90393_REG_WOXY_THRESH;
            break;

        case MLX90393_WOC_THRESHOLD_Z:
            reg = MLX90393_REG_WOZ_THRESH;
            break;

        case MLX90393_WOC_THRESHOLD_T:
            reg = MLX90393_REG_WOT_THRESH;
            break;

        default:
            return MLX90393_ERROR_BAD_PARAM;
    }

    return mlx90393_write_register(dev, reg, threshold, NULL);
}

// *****************************************************************************
int mlx90393_get_woc_threshold(mlx90393_t *dev, mlx90393_woc_threshold_t threshold_id, uint16_t *threshold)
{
    uint8_t reg = 0;
    int error = 0;

    if (threshold == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    switch (threshold_id)
    {
        case MLX90393_WOC_THRESHOLD_XY:
            reg = MLX90393_REG_WOXY_THRESH;
            break;

        case MLX90393_WOC_THRESHOLD_Z:
            reg = MLX90393_REG_WOZ_THRESH;
            break;

        case MLX90393_WOC_THRESHOLD_T:
            reg = MLX90393_REG_WOT_THRESH;
            break;

        default:
            return MLX90393_ERROR_BAD_PARAM;
    }

    error = mlx90393_read_register(dev, reg, threshold, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}




int mlx90393_i2c_transceive(mlx90393_t *dev,
                                   uint8_t *tx_buf,
                                   uint32_t tx_len,
                                   uint8_t *rx_buf,
                                   uint32_t rx_len)
{
    mxc_i2c_req_t req;

    if ((dev == NULL) || (dev->i2c == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    req.i2c = dev->i2c;
    req.addr = dev->i2c_addr;
    req.tx_buf = tx_buf;
    req.tx_len = tx_len;
    req.rx_buf = rx_buf;
    req.rx_len = rx_len;
    req.restart = 0;
    req.callback = NULL;

    return MXC_I2C_MasterTransaction(&req);
}


int mlx90393_send_simple_command(mlx90393_t *dev, uint8_t command, uint8_t *status)
{
    int error = 0;
    uint8_t rx_status = 0;

    if (dev == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_i2c_transceive(dev, &command, 1, &rx_status, 1);
    if (error != E_NO_ERROR) {
        return error;
    }

    if (status != NULL) {
        *status = rx_status;
    }

    if ((rx_status & MLX90393_STATUS_ERROR) != 0) {
        return MLX90393_ERROR_STATUS;
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_read_modify_write(mlx90393_t *dev,
                                      uint8_t reg,
                                      uint16_t mask,
                                      uint8_t shift,
                                      uint16_t value)
{
    int error = 0;
    uint16_t current_value = 0;
    uint16_t new_value = 0;

    if (dev == NULL) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_read_register(dev, reg, &current_value, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    new_value = (uint16_t)((current_value & (~mask)) | ((uint16_t)(value << shift) & mask));

    error = mlx90393_write_register(dev, reg, new_value, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}

// *****************************************************************************
int mlx90393_get_field(mlx90393_t *dev,
                              uint8_t reg,
                              uint16_t mask,
                              uint8_t shift,
                              uint16_t *value)
{
    int error = 0;
    uint16_t register_value = 0;

    if ((dev == NULL) || (value == NULL)) {
        return MLX90393_ERROR_NULL_PTR;
    }

    error = mlx90393_read_register(dev, reg, &register_value, NULL);
    if (error != E_NO_ERROR) {
        return error;
    }

    *value = (uint16_t)((register_value & mask) >> shift);

    return E_NO_ERROR;
}

// *****************************************************************************
uint8_t mlx90393_channel_count(uint8_t zyxt_mask)
{
    uint8_t count = 0;

    if ((zyxt_mask & MLX90393_CHANNEL_T) != 0) {
        count++;
    }

    if ((zyxt_mask & MLX90393_CHANNEL_X) != 0) {
        count++;
    }

    if ((zyxt_mask & MLX90393_CHANNEL_Y) != 0) {
        count++;
    }

    if ((zyxt_mask & MLX90393_CHANNEL_Z) != 0) {
        count++;
    }

    return count;
}

// *****************************************************************************
int16_t mlx90393_adjust_signed_output(int16_t raw_value, mlx90393_resolution_t resolution)
{
    if (resolution == MLX90393_RES_18) {
        return (int16_t)(raw_value - 0x8000);
    }

    if (resolution == MLX90393_RES_19) {
        return (int16_t)(raw_value - 0x4000);
    }

    return raw_value;
}
