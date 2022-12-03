/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_ACCGYRO_BMI088

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_spi_bmi088.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

// 10 MHz max SPI frequency
#define BMI088_MAX_SPI_CLK_HZ 10000000
#define BMI088_ACCEL_CHIP_ID 0x1E

// BMI088 ACCEL registers (not the complete list)
typedef enum {
    BMI088_ACC_REG_CHIP_ID = 0x00,
    BMI088_ACC_REG_RATE_X_LSB = 0x12,
    BMI088_ACC_REG_SENSORTIME_0 = 0x18,
    BMI088_ACC_REG_SENSORTIME_1 = 0x19,
    BMI088_ACC_REG_SENSORTIME_2 = 0x1A,
    BMI088_ACC_REG_TEMP_MSB = 0x22,
    BMI088_ACC_REG_TEMP_LSB = 0x23,
    BMI088_ACC_REG_ACC_CONF = 0x40,
    BMI088_ACC_REG_ACC_RANGE = 0x41,
    BMI088_ACC_REG_INT1_IO_CONF = 0x53,
    BMI088_ACC_REG_INT_MAP_DATA = 0x58,
    BMI088_ACC_REG_ACC_PWR_CONF = 0x7C,
    BMI088_ACC_REG_ACC_PWR_CTRL = 0x7D,
    BMI088_ACC_REG_SOFTRESET = 0x7E,
} bmi088AccelRegister_e;

// BMI088 ACCEL register configuration values
typedef enum {
    BMI088_ACC_VAL_ACC_CONF = 0x8B,                 // OSR4 mode, 800Hz ODR
    BMI088_ACC_VAL_ACC_RANGE_24G = 0x03,            // set accel to 24g full scale
    BMI088_ACC_VAL_INT1_IO_CONF_PINMODE = 0x12,     // int1, push-pull, active high
    BMI088_ACC_VAL_INT_MAP_DATA_INT1 = 0x04,        // map data ready to int1
    BMI088_ACC_VAL_SELF_TEST_OFF = 0x00,            // self-test off
    BMI088_ACC_VAL_SELF_TEST_POSITIVE = 0x0D,       // positive self-test
    BMI088_ACC_VAL_SELF_TEST_NEGATIVE = 0x09,       // negative self-test
    BMI088_ACC_VAL_ACC_PWR_CONF_ACTIVE = 0x00,      // set accel to active mode
    BMI088_ACC_VAL_ACC_PWR_CTRL_ENABLE = 0x04,      // enable accel
    BMI088_ACC_VAL_SOFTRESET = 0xB6,
} bmi088AccelConfigValue_e;

// BMI088 Acc register reads are 16bits with the first byte a "dummy" value 0
// that must be ignored. The result is in the second byte.
static uint8_t bmi088AccRegisterRead(const extDevice_t *dev, bmi088AccelRegister_e registerId)
{
    uint8_t data[2] = { 0, 0 };

    if (spiReadRegMskBufRB(dev, registerId, data, 2)) {
        return data[1];
    } else {
        return 0;
    }
}

static void bmi088AccRegisterWrite(const extDevice_t *dev, bmi088AccelRegister_e registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

uint8_t bmi088AccDetect(const extDevice_t *dev)
{
    bmi088EnableSPI(dev);

    if (bmi088AccRegisterRead(dev, BMI088_ACC_REG_CHIP_ID) == BMI088_ACC_CHIP_ID) {
        return BMI_088_ACC_SPI;
    }

    return MPU_NONE;
}

static void bmi088AccEnableSPI(const extDevice_t *dev)
{
    IOLo(dev->busType_u.spi.csnPin);
    delay(1);
    IOHi(dev->busType_u.spi.csnPin);
    delay(10);
}


static void bmi088AccInit(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi088AccRegisterWrite(dev, BMI088_ACC_REG_SOFTRESET, BMI088_ACC_VAL_SOFTRESET, 100);

    // Toggle the chip into SPI mode
    bmi088AccEnableSPI(dev);

    // enable accelerometer
    bmi088AccRegisterWrite(dev, BMI088_ACC_REG_ACC_PWR_CTRL, BMI088_ACC_VAL_ACC_PWR_CTRL_ENABLE, 450);

    // Configure the accelerometer
    bmi088AccRegisterWrite(dev, BMI088_ACC_REG_ACC_CONF, BMI088_ACC_VAL_ACC_CONF, 1);

    // Configure the accelerometer range
    bmi088AccRegisterWrite(dev, BMI088_ACC_REG_ACC_RANGE, BMI088_ACC_VAL_ACC_RANGE_24G, 1);
}


// bool bmi088AccRead(gyroDev_t *gyro)
bool bmi088AccRead(accDev_t *acc)
{
    uint16_t *gyroData = (uint16_t *)acc->gyro->dev.rxBuf;
    // acc data reading
    acc->gyro->dev.txBuf[0] = BMI088_ACC_REG_RATE_X_LSB | 0x80;
    busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 8, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
    };
    segments[0].u.buffers.txData = acc->gyro->dev.txBuf;
    segments[0].u.buffers.rxData = acc->gyro->dev.rxBuf;

    spiSequence(&acc->gyro->dev, &segments[0]);

    // Wait for completion
    spiWait(&acc->gyro->dev);

    acc->ADCRaw[X] = gyroData[1];
    acc->ADCRaw[Y] = gyroData[2];
    acc->ADCRaw[Z] = gyroData[3];

    return true;
}

bool bmi088SpiAccInit(accDev_t *acc)
{
    acc->acc_1G = 1365;   // 24g sensor scale
}

bool bmi088SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != BMI_088_ACC_SPI) {
        return false;
    }

    acc->initFn = bmi088SpiAccInit;
    acc->readFn = bmi088AccRead;

    return true;
}

#endif
