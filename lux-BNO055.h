#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>



#define START_BYTE 0xAA
#define RESPONSE_BYTE 0xBB
#define ERROR_BYTE 0xEE
#define BNO055_I2C_ADDR_HI 0x28
#define BNO055_I2C_ADDR_LO 0x29
#define BNO055_I2C_ADDR BNO055_I2C_ADDR_LO

#define BNO055_READ_TIMEOUT 100
#define BNO055_WRITE_TIMEOUT 10 

#define ERROR_WRITE_SUCCESS 0x01
#define ERROR_WRITE_FAIL 0x03



#define BNO055_ID (0x0A)
#define BNO055_CHIP_ID 0x00
#define BNO055_ACC_ID 0x01
#define BNO055_MAG_ID 0x02
#define BNO055_GYRO_ID 0x03
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B
#define BNO055_EUL_ROLL_LSB 0x1C
#define BNO055_EUL_ROLL_MSB 0x1D
#define BNO055_EUL_PITCH_LSB 0x1E
#define BNO055_EUL_PITCH_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D

#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33
#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SING 0x42

#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB 0x67
#define BNO055_ACC_RADIUS_MSB 0x68
#define BNO055_MAG_RADIUS_LSB 0x69
#define BNO055_MAG_RADIUS_MSB 0x6A


#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRP_CONFIG_1 0x0B
#define BNO055_INT_MSK 0x0





enum bno055_system_status_t {
  BNO055_SYSTEM_STATUS_IDLE = 0x00,
  BNO055_SYSTEM_STATUS_ERROR = 0x01,
  BNO055_SYSTEM_STATUS_INITILAIZING_PERIPHERALS = 0x02,
  BNO055_SYSTEM_STATUS_SYSTEM_INITALIZING = 0x03,
  BNO055_SYSTEM_STATYS_EXECUTING_SELF_TEST = 0x04,
  BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING = 0x05,
  BNO055_SYSTEM_STATUS_FUSION_ALGO_NOT_RUNNING = 0x06
};


typedef enum {
  BNO055_OPERATION_MODE_CONFIG = 0x00,

  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYROONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_AMG,

  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF
} bno055_opmode_t;


typedef struct {
  uint8_t mcuState;
  uint8_t gyrState;
  uint8_t magState;
  uint8_t accState;
} bno055_self_test_result_t;

typedef struct {
  uint8_t sys;
  uint8_t gyro;
  uint8_t mag;
  uint8_t accel;
} bno055_calibration_state_t;


typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} bno055_vector_xyz_int16_t;

typedef struct {
  bno055_vector_xyz_int16_t gyro;
  bno055_vector_xyz_int16_t mag;
  bno055_vector_xyz_int16_t accel;
} bno055_calibration_offset_t;

typedef struct {
  uint16_t mag;
  uint16_t accel;  
} bno055_calibration_raduis_t;


typedef struct {
  bno055_calibration_offset_t offset;
  bno055_calibration_raduis_t radius;
} bno055_calibration_data_t;

typedef struct {
  double w;
  double x;
  double y;
  double z;
} bno055_vector_t;

typedef struct {
  uint8_t x;
  uint8_t x_sign;
  uint8_t y;
  uint8_t y_sign;
  uint8_t z;
  uint8_t z_sign;
} bno055_axis_map_t;



typedef enum {
  BNO055_VECTOR_ACCELEROMETER = 0x08,
  BNO055_VECTOR_MAGNETOMETER = 0x0E,
  BNO055_VECTOR_GYROSCOPE = 0x14,
  BNO055_VECTOR_EULER = 0x1A,
  BNO055_VECTOR_QUATERNION = 0x20,
  BNO055_VECTOR_LINEARACCEL = 0x28,
  BNO055_vector_GRAVITY = 0x2E
} bno055_vector_type_t;


enum bno055_axis_map_represetation_t {
  BNO055_AXIS_X = 0x00,
  BNO055_AXIS_y = 0x01,
  BNO055_AXIS_Z = 0x02
};

enum bno055_axis_map_sign_t {
  BNO055_AXIS_SIGN_POSITIVE = 0x00,
  BNO055_AXIS_SIGN_NEGATIVE = 0x01
};


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





void bno055_writeData(uint8_t reg, uint8_t data);
void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len);
//void bno055_delay(int time);

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




void bno055_reset();
bno055_opmode_t bno055_getOpeationMode();
void bno055_setOperationMode(bno055_opmode_t mode);
void bno055_setOperationModeConfig();
void bno055_setOperationModeNDOF();
void bno055_enableExternalCrystal();
void bno055_disableExternalCrystal();
void bno055_setuo();

int8_t bno055_getTemp();

uint8_t bno055_getSystemStatus();
uint8_t bno055_getSystemError();

bno055_self_test_result_t bno055_getSelfTestResult();
bno055_calibration_state_t bno055_getCalibrationState();
bno055_calibration_data_t bno055_getCalibrationData();
void bno055_setCalibrationData(bno055_calibration_data_t calData);

bno055_vector_t bno055_getVectorAccelerometer();
bno055_vector_t bno055_getVectorMagetometer();
bno055_vector_t bno055_getVectorGyroscope();
bno055_vector_t bno055_getVectorEuler();
bno055_vector_t bno055_getVectorGravity();
bno055_vector_t bno055_getVectorQuaternion();
void bno055_setAxisMap(bno055_axis_map_t axis);








