/* The bridge between ST's portable sensor driver and NXP's I2C driver.
 * Only one main loop uses this bus. If tasks share it later, protect whole
 * sensor operations with a mutex; individual register calls are insufficient.
 */
#include "vl53l0x_platform.h"
#include "app_config.h"

static VL53L0X_Error Transfer(VL53L0X_DEV dev, uint8_t index,
                             uint8_t *data, uint32_t count,
                             lpi2c_direction_t direction)
{
    /* static keeps this helper private to this source file.
     * A transfer descriptor describes an operation; it is not a data packet.
     */
    lpi2c_master_transfer_t transfer = {0};
    transfer.slaveAddress   = dev->I2cDevAddr;
    transfer.direction      = direction;
    transfer.subaddress     = index; /* Sensor register number. */
    transfer.subaddressSize = 1U;    /* Register numbers occupy one byte. */
    transfer.data           = data;  /* Pointer to bytes, not a copy of them. */
    transfer.dataSize       = count;
    transfer.flags          = kLPI2C_TransferDefaultFlag;

    /* Blocking means this call returns after completion or an error.
     * A read writes the register index, issues a repeated START, then reads:
     * START -> address/write -> index -> RESTART -> address/read -> bytes -> STOP.
     * The SDK generates address bits and ACK/NACK handling for us.
     * An ACK is the receiver pulling SDA low to acknowledge a byte.
     */
    dev->last_i2c_status = LPI2C_MasterTransferBlocking(dev->i2c, &transfer);
    if (dev->last_i2c_status != kStatus_Success)
    {
        /* ST and NXP use different error-number systems. Translate them here. */
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count)
{
    return Transfer(dev, index, data, count, kLPI2C_Write);
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count)
{
    return Transfer(dev, index, data, count, kLPI2C_Read);
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data)
{
    /* &data points to the one-byte local value for this blocking operation. */
    return VL53L0X_WriteMulti(dev, index, &data, 1U);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data)
{
    /* Sensor multi-byte values use most-significant byte first (big endian).
     * >> shifts bits right; converting to uint8_t keeps the bottom eight bits.
     * 0x1234 becomes bytes 0x12, 0x34 regardless of the MCU's own byte order.
     */
    uint8_t bytes[2] = {(uint8_t)(data >> 8), (uint8_t)data};
    return VL53L0X_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data)
{
    uint8_t bytes[4] = {(uint8_t)(data >> 24), (uint8_t)(data >> 16),
                        (uint8_t)(data >> 8), (uint8_t)data};
    return VL53L0X_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(dev, index, data, 1U);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data)
{
    uint8_t bytes[2];
    VL53L0X_Error error = VL53L0X_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (error == VL53L0X_ERROR_NONE)
    {
        /* Cast BEFORE shifting; | combines non-overlapping bit fields.
         * *data writes the assembled result into the caller's variable.
         * Never assemble bytes after a failed read: they might be uninitialized.
         */
        *data = ((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1];
    }
    return error;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data)
{
    uint8_t bytes[4];
    VL53L0X_Error error = VL53L0X_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (error == VL53L0X_ERROR_NONE)
    {
        *data = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
                ((uint32_t)bytes[2] << 8) | (uint32_t)bytes[3];
    }
    return error;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index, uint8_t and_mask, uint8_t or_mask)
{
    uint8_t value;
    VL53L0X_Error error = VL53L0X_RdByte(dev, index, &value);
    if (error != VL53L0X_ERROR_NONE)
    {
        return error; /* Stop rather than overwrite a register using bad data. */
    }
    /* & clears bits where and_mask has zero; | sets bits where or_mask has one.
     * This read/modify/write preserves unrelated settings in the same register.
     */
    value = (value & and_mask) | or_mask;
    return VL53L0X_WrByte(dev, index, value);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev)
{
    (void)dev; /* This delay applies to the MCU, so the sensor pointer is unused. */
    APP_DelayMs(1U);
    return VL53L0X_ERROR_NONE;
}
