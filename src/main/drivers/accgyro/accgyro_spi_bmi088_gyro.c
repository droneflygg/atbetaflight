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

#define BMI088_GYRO_CHIP_ID 0x0F

// BMI088 GYRO registers (not the complete list)
typedef enum {
    BMI088_GYRO_REG_CHIP_ID = 0x00,
    BMI088_GYRO_RET_RATE_X_LSB = 0x02,
    BMI088_GYRO_REG_RANGE = 0x0F,
    BMI088_GYRO_REG_BANDWIDTH = 0x10,
    BMI088_GYRO_REG_SOFTRESET = 0x14,
    BMI088_GYRO_REG_INT_CTRL = 0x15,
    BMI088_GYRO_REG_INT3_INT4_IO_CONF = 0x16,
    BMI088_GYRO_REG_INT3_INT4_IO_MAP = 0x18,
    BMI088_GYRO_REG_SELF_TEST = 0x3C,
} bmi088GyroRegister_e;

// BMI088 GYRO register configuration values
typedef enum {
    BMI088_GYRO_VAL_RANGE_2000DPS = 0x00,           // set gyro to 2000 dps full scale
    BMI088_GYRO_VAL_BANDWIDTH_532Hz = 0x00,         // 2000Hz ODR, 532Hz filter
    BMI088_GYRO_VAL_BANDWIDTH_230Hz = 0x01,         // 2000Hz ODR, 230Hz filter
    BMI088_GYRO_VAL_SOFTRESET = 0xB6,
    BMI088_GYRO_VAL_INT_CTRL_NEW_DATA = 0x80,       // enable new data interrupt
    BMI088_GYRO_VAL_INT3_IO_CONF_PINMODE = 0x01,    // int3, push-pull, active high
    BMI088_GYRO_VAL_INT3_INT4_IO_MAP_INT3 = 0x01,   // map data ready to int3
    BMI088_GYRO_VAL_SELF_TEST_START = 0x00,         // start the built-in self-test
} bmi088ConfigValue_e;

// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// BMI088 gyro register reads are 8bits result.
static uint8_t bmi088GyroRegisterRead(const extDevice_t *dev, bmi088GyroRegister_e registerId)
{
    uint8_t data[1] = {0};

    if (spiReadRegMskBufRB(dev, registerId, data, 1)) {
        return data[0];
    } else {
        return 0;
    }
}

static void bmi088GyroRegisterWrite(const extDevice_t *dev, bmi088GyroRegister_e registerId, uint8_t value, unsigned delayMs)
{
    spiWriteReg(dev, registerId, value);
    if (delayMs) {
        delay(delayMs);
    }
}

uint8_t bmi088GyroDetect(const extDevice_t *dev)
{
    bmi088EnableSPI(dev);

    if (bmi088GyroRegisterRead(dev, BMI088_GYRO_REG_CHIP_ID) == BMI088_GYRO_CHIP_ID) {
        return BMI_088_GYRO_SPI;
    }

    return MPU_NONE;
}

static uint8_t getBmiGyroBandwidth()
{
    switch (gyroConfig()->gyro_hardware_lpf) {
    case GYRO_HARDWARE_LPF_NORMAL:
        return BMI088_GYRO_VAL_BANDWIDTH_230Hz;
    case GYRO_HARDWARE_LPF_OPTION_1:
        return BMI088_GYRO_VAL_BANDWIDTH_532Hz;
    case GYRO_HARDWARE_LPF_OPTION_2:
        return BMI088_GYRO_VAL_BANDWIDTH_532Hz;
    case GYRO_HARDWARE_LPF_EXPERIMENTAL:
        return BMI088_GYRO_VAL_BANDWIDTH_532Hz;
    }
    return 0;
}

static void bmi088GyroConfig(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    // Perform a soft reset to set all configuration to default
    // Delay 100ms before continuing configuration
    bmi088GyroRegisterWrite(dev, BMI088_GYRO_REG_SOFTRESET, BMI088_GYRO_VAL_SOFTRESET, 100);

    // Configure the gyro range
    bmi088GyroRegisterWrite(dev, BMI088_GYRO_REG_RANGE, BMI088_GYRO_VAL_RANGE_2000DPS, 1);

    // Configure the gyro bandwidth
    bmi088GyroRegisterWrite(dev, BMI088_GYRO_REG_BANDWIDTH, getBmiGyroBandwidth(), 1);

    // Configure the INT CTRL mode
    bmi088GyroRegisterWrite(dev, BMI088_GYRO_REG_INT_CTRL, BMI088_GYRO_VAL_INT_CTRL_NEW_DATA, 1);

    // Configure the INT3 pinmode
    bmi088GyroRegisterWrite(dev, BMI088_GYRO_REG_INT3_INT4_IO_CONF, BMI088_GYRO_VAL_INT3_IO_CONF_PINMODE, 1);

    // Configure the INT3 interrupt map
    bmi088GyroRegisterWrite(dev, BMI088_GYRO_REG_INT3_INT4_IO_MAP, BMI088_GYRO_VAL_INT3_INT4_IO_MAP_INT3, 1);
}


/*
 * Gyro interrupt service routine
 */
extiCallbackRec_t bmi088GyroIntCallbackRec;
#ifdef USE_GYRO_EXTI
// Called in ISR context
// Gyro read has just completed
busStatus_e bmi088GyroIntcallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);

    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }

    gyro->dataReady = true;

    return BUS_READY;
}

void bmi088GyroExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    // Ideally we'd use a timer to capture such information, but unfortunately the port used for EXTI interrupt does
    // not have an associated timer
    uint32_t nowCycles = getCycleCounter();
    gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    gyro->gyroLastEXTI = nowCycles;

    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(&gyro->dev, gyro->segments);
    }

    gyro->detectedEXTI++;

}

static void bmi088GyroIntExtiInit(gyroDev_t *gyro)
{
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }

    IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);

    IOInit(mpuIntIO, OWNER_GYRO_EXTI, 0);
    EXTIHandlerInit(&gyro->exti, bmi088GyroExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO);
}
#else
void bmi088GyroExtiHandler(extiCallbackRec_t *cb)
{
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    gyro->dataReady = true;
}
#endif

bool bmi088GyroRead(gyroDev_t *gyro)
{
    uint16_t *gyroData = (uint16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        // Initialise the tx buffer to all 0x00
        memset(gyro->dev.txBuf, 0x00, 8);
#ifdef USE_GYRO_EXTI
        // Check that minimum number of interrupts have been detected

        // We need some offset from the gyro interrupts to ensure sampling after the interrupt
        gyro->gyroDmaMaxDuration = 5;
        // Using DMA for gyro access upsets the scheduler on the F4
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[1] = BMI088_GYRO_RET_RATE_X_LSB | 0x80;
                gyro->segments[0].len = 8;
                gyro->segments[0].callback = bmi088GyroIntcallback;
                gyro->segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                // Interrupts are present, but no DMA
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else
#endif
        {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        // gyro data reading
        gyro->dev.txBuf[1] = BMI088_GYRO_RET_RATE_X_LSB | 0x80;
        busSegment_t segments[] = {
                {.u.buffers = {NULL, NULL}, 7, true, NULL},
                {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = &gyro->dev.txBuf[1];
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);

        // Wait for completion
        spiWait(&gyro->dev);

        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];

        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // If read was triggered in interrupt don't bother waiting. The worst that could happen is that we pick
        // up an old value.
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;
    }

    default:
        break;
    }

    return true;
}

static void bmi088SpiGyroInit(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;

    bmi088GyroConfig(gyro);

#if defined(USE_GYRO_EXTI)
    bmi088GyroIntExtiInit(gyro);
#endif

    spiSetClkDivisor(dev, spiCalculateDivider(BMI088_MAX_SPI_CLK_HZ));
}

bool bmi088SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != BMI_088_GYRO_SPI) {
        return false;
    }

    gyro->initFn = bmi088SpiGyroInit;
    gyro->readFn = bmi088GyroRead;
    gyro->scale = GYRO_SCALE_2000DPS;

    return true;
}



#endif