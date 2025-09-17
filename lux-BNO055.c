#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <lux-BNO055.h>
#include <errno.h>
#include <string.h>   
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>



uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1<<14);











void bno055_setPage(uint8_t page) {bno055_writeData(BNO055_PAGE_ID, page); }


bno055_opmode_t bno055_getOpeationMode() {
    uint8_t mode;
    bno055_readData(BNO055_OPR_MODE, &mode, 1);
    return (bno055_opmode_t)mode;
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


void bno055_setup() {
    bno055_reset();

    uint8_t id = 0;
    bno055_readData(BNO055_CHIP_ID, &id, 1);
    if (id != BNO055_ID) {

    }
    bno055_setPage(0);
    bno055_writeData(BNO055_SYS_TRIGGER, 0x0);
    
    bno055_setOperationModeConfig();
    bno055_delay(10);
}


bno055_self_test_result_t bno055_getSelfTestResult() {
    bno055_setPage(0);
    uint8_t tmp;
    bno055_self_test_result_t res = {
        .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
    bno055_readData(BNO055_ST_RESULT, &tmp, 1);
    res.mcuState = (tmp >> 3) & 0x01;
    res.gyrState = (tmp >> 2) & 0x01;
    res.magState = (tmp >> 1) & 0x01;
    res.accState = (tmp >> 0) & 0x01;
    return res;
}

bno055_calibration_state_t bno055_getCalibrationState() {
    bno055_setPage(0);
    bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
    uint8_t calState = 0;
    bno055_readData(BNO055_CALIB_STAT, &calState, 1);
    cal.sys = (calState >> 6) & 0x03;
    cal.gyro = (calState >> 4) & 0x03;
    cal.accel = (calState >> 2) & 0x03;
    cal.mag = calState & 0x03;
    return cal;
}




bno055_calibration_data_t bno055_getCalibrationData() {
    bno055_calibration_data_t calData;
    uint8_t buffer[22];
    bno055_opmode_t operationMode = bno055_getOpeationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(0);

    bno055_readData(BNO055_ACC_OFFSET_X_LSB,  buffer, 22);
    memcpy(&calData.offset.accel, buffer, 6);
    memcpy(&calData.offset.mag, buffer + 6, 6);
    memcpy(&calData.offset.gyro, buffer + 12, 6);
    memcpy(&calData.radius.accel, buffer + 18, 2);
    memcpy(&calData.radius.mag, buffer + 20, 2);

    bno055_setOperationMode(operationMode);

    return calData;
}


void bno055_setCalibrationData(bno055_calibration_data_t calData) {
    uint8_t buffer[22];
    bno055_opmode_t operationMode = bno055_getOpeationMode();
    bno055_setOperationModeConfig();
    bno055_setPage(0);

    memcpy(buffer, &calData.offset.accel, 6);
    memcpy(buffer + 6, &calData.offset.mag, 6);
    memcpy(buffer + 12, &calData.offset.gyro, 6);
    memcpy(buffer + 18, &calData.radius.accel, 2);
    memcpy(buffer + 20, &calData.radius.mag, 2);


    for (uint8_t i=0; i < 22; i++) {
        bno055_writeData(BNO055_ACC_DATA_X_LSB+i, buffer[i]);
    }
  
    bno055_setOperationMode(operationMode);
}



bno055_vector_t bno055_getVector(uint8_t vec) {
    bno055_setPage(0);
    uint8_t buffer[8];
    
    if (vec == BNO055_VECTOR_QUATERNION)
        bno055_readData(vec, buffer, 8);
    else
        bno055_readData(vec, buffer, 6);


    double scale = 1;

    if (vec == BNO055_VECTOR_MAGNETOMETER) {
        scale = magScale;
    } else if (vec == BNO055_VECTOR_ACCELEROMETER || vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GYROSCOPE) {
        scale = accelScale;
    } else if (vec == BNO055_VECTOR_GYROSCOPE) {
        scale = angularRateScale;
    } else if (vec == BNO055_VECTOR_EULER) {
        scale = eulerScale;
    } else if (vec == BNO055_VECTOR_QUATERNION) {
        scale = quaScale;
    }


    bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z =0};
    if (vec == BNO055_VECTOR_QUATERNION) {
        xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
        xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
        xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
        xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
    } else {
        xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
        xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
        xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    }


    return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer() {
    return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagetometer() {
    return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope() {
    return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler() {
    return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel() {
    return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity() {
    return bno055_getVector(BNO055_vector_GRAVITY);
}
bno055_vector_t bno055_getVectorQuaternion() {
    return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

double bno055_yawFromQuaternion(bno055_vector_t quat) {
    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);

    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);

    double yaw = atan2(siny_cosp, cosy_cosp);

    double yaw_degrees = yaw * (180.0 / pi);
    if  (yaw_degrees < 0) {
        yaw_degrees += 360.0;
    }
    return yaw_degrees;
}




void bno055_setAxisMap(bno055_axis_map_t  axis) {
    uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
    uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
    bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
    bno055_writeData(BNO055_AXIS_MAP_SING, axisMapSign);
}

static int i2c_fd = -1;
static uint8_t i2c_addr = 0;

int bno055_linux_init(int bus, uint8_t addr) {
  char filename[32];
  snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);

  i2c_fd = open(filename, O_RDWR);
  if (i2c_fd < 0) {
    fprintf(stderr, "bno055: open %s failed: %s\n", filename, strerror(errno));
    return -1;
  }

i2c_addr = addr;

if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
    fprintf(stderr, "bno055: ioctl I2C_SLAVE 0x%02x failed: %s\n", addr, strerror(errno));
    close(i2c_fd);
    i2c_fd = -1;
    return -1;
  }
  return 0;
}


void bno055_delay(uint32_t ms) {
  usleep(ms * 1000u);
}

void bno055_close(void) {
  if (i2c_fd >= 0) {
    close(i2c_fd);
    i2c_fd = -1;
  }
}

void bno055_writeData(uint8_t reg, uint8_t data)
{
    if (i2c_fd < 0) {
        fprintf(stderr, "bno055_writeData: i2c not initialized\n");
        return;
    }

    uint8_t out[2] = { reg, data };
    ssize_t w = write(i2c_fd, out, sizeof(out));
    if (w != (ssize_t)sizeof(out)) {
        fprintf(stderr, "bno055_writeData: write failed (w=%zd): %s\n", w, strerror(errno));
    }
}


void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (i2c_fd < 0) {
        fprintf(stderr, "bno055_readData: i2c not initialized\n");
        if (data && len) memset(data, 0, len);
        return;
    }
    if (len == 0) return;

    ssize_t w = write(i2c_fd, &reg, 1);
    if (w != 1) {
        fprintf(stderr, "bno055_readData: fallback write(reg) failed (w=%zd): %s\n", w, strerror(errno));
        memset(data, 0, len);
        return;
    }

    ssize_t r = read(i2c_fd, data, len);
    if (r != (ssize_t)len) {
        fprintf(stderr, "bno055_readData: read failed (r=%zd, want=%u): %s\n", r, len, strerror(errno));
        memset(data, 0, len);
        return;
    }
}




int bno055_saveCalibrationData(const char *path) {
    


    bno055_calibration_state_t cal = bno055_getCalibrationState();
    if (cal.sys < 3 || cal.gyro < 3) {
        fprintf(stderr, "beno055_saveCalibrationData: not fully calibrated (sys=%u, mag=%u)\n", cal.sys, cal.gyro);
        return -1;
    }

    bno055_calibration_data_t calData = bno055_getCalibrationData();

    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "this shit ant work - hope the error helps %s\n", path, strerror(errno));
        return -2;
    }

    size_t written = fwrite(&calData, 1, sizeof(calData), f);
    fclose(f);

    if (written != sizeof(calData)) {
        fprintf(stderr , "did not write all of the data\n");
        return -3;

    } 
    return 0;
}


int bno055_loadCalibrationData(const char *path) {
    
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "this shit ant work - hope the error helps  %s:  %s\n", path, strerror(errno));
        return -1;
    }

    bno055_calibration_data_t data;

    size_t read = fread(&data, 1, sizeof(data), f);
    fclose(f);

    if (read != sizeof(data)) {
        fprintf(stderr , "this is prob not the right file, or it has been coruped try again\n");
        return -2;

    } 

    bno055_setCalibrationData(data);
    
    bno055_delay(50);

    return 0;
}






