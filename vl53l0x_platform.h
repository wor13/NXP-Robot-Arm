#ifndef APP_VL53L0X_PLATFORM_H
#define APP_VL53L0X_PLATFORM_H
#include "vl53l0x_def.h"
#include "vl53l0x_platform_log.h"
#include "fsl_lpi2c.h"

/* One instance represents one physical sensor. Data is ST's private state;
 * the remaining fields tell our adapter how to contact that sensor.
 * I2cDevAddr ALWAYS stores a 7-bit address here, not ST examples' 0x52 form.
 */
typedef struct
{
    VL53L0X_DevData_t Data;
    uint8_t I2cDevAddr;
    LPI2C_Type *i2c;
    status_t last_i2c_status; /* Preserve NXP's detailed error for diagnostics. */
} VL53L0X_Dev_t;
typedef VL53L0X_Dev_t *VL53L0X_DEV;

/* -> accesses a field through a pointer. These names are required by ST. */
#define PALDevDataGet(Dev, field) ((Dev)->Data.field)
#define PALDevDataSet(Dev, field, value) ((Dev)->Data.field = (value))

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count);
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data);
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data);
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data);
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data);
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data);
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data);
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index, uint8_t and_mask, uint8_t or_mask);
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev);
#endif
