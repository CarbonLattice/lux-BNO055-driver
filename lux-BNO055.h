#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>



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
#define BNO055_ACC_DATA_X_LSB
#define BNO055_ACC_DATA_X_MSB
#define BNO055_ACC_DATA_Y_LSB
#define BNO055_ACC_DATA_Y_MSB
#define BNO055_ACC_DATA_Z_LSB
#define BNO055_ACC_DATA_Z_MSB
#define BNO055_MAG_DATA_X_LSB
#define BNO055_MAG_DATA_X_MSB
#define BNO055_MAG_DATA_Y_LSB
#define BNO055_MAG_DATA_Y_MSB
#define BNO055_MAG_DATA_Z_LSB
#define BNO055_MAG_DATA_Z_MSB
#define BNO055_GYR_DATA_X_LSB
#define BNO055_GYR_DATA_X_MSB
#define BNO055_GYR_DATA_Y_LSB
#define BNO055_GYR_DATA_Y_MSB
#define BNO055_GYR_DATA_Z_LSB
#define BNO055_GYR_DATA_Z_MSB
#define BNO055_EUL_HEADING_LSB
#define BNO055_EUL_HEADING_MSB
#define BNO055_EUL_ROLL_LSB
#define BNO055_EUL_ROLL_MSB
#define BNO055_EUL_PITCH_LSB
#define BNO055_EUL_PITCH_MSB
#define BNO055_QUA_DATA_W_LSB
#define BNO055_QUA_DATA_W_MSB
#define BNO055_QUA_DATA_X_LSB
#define BNO055_QUA_DATA_X_MSB
#define BNO055_QUA_DATA_Y_LSB
#define BNO055_QUA_DATA_Y_MSB
#define BNO055_QUA_DATA_Z_LSB
#define BNO055_QUA_DATA_Z_MSB
#define BNO055_LIA_DATA_X_LSB
#define BNO055_LIA_DATA_X_MSB
#define BNO055_LIA_DATA_Y_LSB
#define BNO055_LIA_DATA_Y_MSB
#define BNO055_LIA_DATA_Z_LSB
#define BNO055_LIA_DATA_Z_MSB

#define BNO055_GRV_DATA_X_LSB
#define BNO055_GRV_DATA_X_MSB
#define BNO055_GRV_DATA_Y_LSB
#define BNO055_GRV_DATA_Y_MSB
#define BNO055_GRV_DATA_Z_LSB
#define BNO055_GRV_DATA_Z_MSB
#define BNO055_TEMP
#define BNO055_CALIB_STAT
#define BNO055_ST_RESULT
#define BNO055_INT_STATUS
#define BNO055_SYS_CLK_STATUS
#define BNO055_SYS_STATUS
#define BNO055_SYS_ERR
#define BNO055_UNIT_SEL
#define BNO055_OPR_MODE
#define BNO055_PWR_MODE
#define BNO055_SYS_TRIGGER
#define BNO055_TEMP_SOURCE
#define BNO055_AXIS_MAP_CONFIG
#define BNO055_AXIS_MAP_SING

#define BNO055_ACC_OFFSET_X_LSB
#define BNO055_ACC_OFFSET_X_MSB
#define BNO055_ACC_OFFSET_Y_LSB
#define BNO055_ACC_OFFSET_Y_MSB
#define BNO055_ACC_OFFSET_Z_LSB
#define BNO055_ACC_OFFSET_Z_MSB
#define BNO055_MAG_OFFSET_X_LSB
#define BNO055_MAG_OFFSET_X_MSB
#define BNO055_MAG_OFFSET_Y_LSB
#define BNO055_MAG_OFFSET_Y_MSB
#define BNO055_MAG_OFFSET_Z_LSB
#define BNO055_MAG_OFFSET_Z_MSB
#define BNO055_GYR_OFFSET_X_LSB
#define BNO055_GYR_OFFSET_X_MSB
#define BNO055_GYR_OFFSET_Y_LSB
#define BNO055_GYR_OFFSET_Y_MSB
#define BNO055_GYR_OFFSET_Z_LSB
#define BNO055_GYR_OFFSET_Z_MSB
#define BNO055_ACC_RADIUS_LSB
#define BNO055_ACC_RADIUS_MSB
#define BNO055_MAG_RADIUS_LSB
#define BNO055_MAG_RADIUS_MSB


#define BNO055_PAGE_ID
#define BNO055_ACC_CONFIG
#define BNO055_MAG_CONFIG
#define BNO055_GYRO_CONFIG_0
#define BNO055_GYRP_CONFIG_1
#define BNO055_INT_MSK
#define BNO055_INT_EN
#define BNO055_ACC_AM_THRES
#define BNO055_ACC_INT_SETTINGS
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_
#define BNO055_






enum bno055_system_status_t {
  BNO055_SYSTEM_STATUS_IDLE = 0x00,
  BNO055_SYSTEM_STATUS_ERROR = 0x01,
  BNO055_SYSTEM_STATUS_INITILAIZING_PERIPHERALS = 0x02,
  BNO055_SYSTEM_STATUS_SYSTEM_INITALIZING = 0x03,
  BNO055_SYSTEM_STATYS_EXECUTING_SELF_TEST = 0x04,
  BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING = 0x05,
  BNO055_SYSTEM_STATUS_FUSION_ALGO_NOT_RUNNING 0x06
};


typedef enum {
  BNO055_OPERATION_MODE_CONFIG = 0x00,

  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYROONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_ACCMAG,

  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF
} bno055_opmode_t


typedef struct {
  uint8_t mcuState;
  uint8_t gyrState;
  uint8_t magState;
  uint8_t accState;
} bno_self_test_result_t;

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
  BNO055_VECTOR_MAGNETOMETER - 0x0E,
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


void bno055_writeData(uint8_t reg, uint8_t data);
void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len);
void bno055_delay(int time);

void bno055_reset();
bno055_opmode_t bno055_getOpeationMode();
void bno055_setOperationMode(bno055_opmode_t mode);
void bno055_setOperationModeConfig();
void bno055_setOperationModeNDOF();
void bno055_enableExternalCrystal();
void bno055_disableExternalCrystal();
void bno055_setuo();

int8_t bno055_getTemp():

uint8_t bno055_getSystemStatus();
uint8_t bno055_getSystemError();

bno055_self_test_result_t bno055_getSelfTestResult();
bno055_calibration_state_t bno055_getCalibrationState();








