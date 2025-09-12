#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <lux-BNO055.h>




uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);

void bno055_setPage(uint8_t page) {bno055_writeData(BNO055_PAGE_ID, page); }


bno055_opmode_t bno055_getOpeationMode() {
    bno055_opmode_t mode;
    bno055_readData(BNO055_OPR_MODE, &mode, 1);
    return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode) {
    bno055_writeData(BNO055_OPR_MODE, mode);
    if (mode == BNO055_OPERATION_MODE_CONFIG) {
        bno055_delay(19);
    } else {
        bno055_delay(7);
    }
}

void bno055_setOperationModeConfig() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void bno055_setExternalCrystalUse(bool state) {
    bno055_setPage(0);
    uint8_t tmp = 0;
    bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
    tmp |= (state == true) ? 0x80 : 0x0;
    bno055_writeData(BNO055_SYS_TRIGGER, tmp);
    bno055_delay(700);
}


void bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
void bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }


void bno055_reset() {
    bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
    bno055_delay(700);
}

int8_t bno055_getTemp() {
    bno055_setPage(0);
    uint8_t t;
    bno055_readData(BNO055_TEMP, &t, 1);
    return t;
}