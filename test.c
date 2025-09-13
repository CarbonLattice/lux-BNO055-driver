// test.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>    
#include "lux-BNO055.h"

int main(void)
{

    const int i2c_bus = 7;                    
    const uint8_t addr = BNO055_I2C_ADDR;     

    if (bno055_linux_init(i2c_bus, addr) < 0) {
        fprintf(stderr, "Failed to initialize I2C bus %d for BNO055 at 0x%02X\n", i2c_bus, addr);
        return 1;
    }

    printf("BNO055: initialized on /dev/i2c-%d addr 0x%02X\n", i2c_bus, addr);


    bno055_setup();
	bno055_setOperationModeNDOF();

    bno055_delay(100);

    for (int i = 0; i < 100; i++) {
 
    int8_t temp = bno055_getTemp();
    printf("TEMP = %d °C\n", temp);

   
    bno055_self_test_result_t st = bno055_getSelfTestResult();
    printf("SELF TEST -> MCU:%u  GYRO:%u  MAG:%u  ACC:%u\n",
           st.mcuState, st.gyrState, st.magState, st.accState);


    bno055_calibration_state_t cal = bno055_getCalibrationState();
    printf("CALIB STAT -> SYS:%u  GYRO:%u  ACCEL:%u  MAG:%u\n",
           cal.sys, cal.gyro, cal.accel, cal.mag);


    bno055_vector_t accel = bno055_getVectorAccelerometer();
    printf("ACCEL (units) -> X: %.3f  Y: %.3f  Z: %.3f\n", accel.x, accel.y, accel.z);

 
    bno055_vector_t quat = bno055_getVectorQuaternion();
    printf("QUAT (w x y z) -> %.6f  %.6f  %.6f  %.6f\n", quat.w, quat.x, quat.y, quat.z);

    sleep(1);
}
    // done
    bno055_close();
    return 0;
}