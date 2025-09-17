// test.c 
#include "lux-BNO055.h"

void set_nonblocking(int enable) {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (enable)
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    else
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}



int main(void)
{
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    unsigned long counter = 0;

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

    if (bno055_loadCalibrationData("bno055.cal") == 0) {
        printf("Loaded calibration data from bno055.cal\n");
    } else {
        printf("No calibration data loaded\n");
    }

    set_nonblocking(1);

    char ch = 0;

    while (ch != 'q' && ch != 'Q') {
        //printf("\033[H\033[J");
        //int8_t temp = bno055_getTemp();
        //bno055_self_test_result_t st = bno055_getSelfTestResult();    
        //bno055_calibration_state_t cal = bno055_getCalibrationState();
        //bno055_vector_t accel = bno055_getVectorAccelerometer();    
        bno055_vector_t quat = bno055_getVectorQuaternion();
        //bno055_vector_t euler = bno055_getVectorEuler();
        double yaw = bno055_yawFromQuaternion(quat);



        //printf("TEMP = %d °C\n", temp);
        counter++;
   
        //printf("SELF TEST -> MCU:%u  GYRO:%u  MAG:%u  ACC:%u\n",
          //st.mcuState, st.gyrState, st.magState, st.accState);

        //printf("CALIB STAT -> SYS:%u  GYRO:%u  ACCEL:%u  MAG:%u\n",
           //cal.sys, cal.gyro, cal.accel, cal.mag);

        //printf("ACCEL (units) -> X: %.3f  Y: %.3f  Z: %.3f\n", accel.x, accel.y, accel.z);

        //printf("QUAT (w x y z) -> %.6f  %.6f  %.6f  %.6f\n", quat.w, quat.x, quat.y, quat.z);

        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) +
                     (now.tv_nsec - start.tv_nsec) / 1e9;

        if (elapsed >= 1.0) {
            printf("Rate: %lu Hz\n", counter);
            counter = 0;
            start = now;
        }


        printf("Quat (dps) -> W: %.3f  X: %.3f  Y: %.3f  Z: %.3f               ", quat.w, quat.x, quat.y, quat.z);
        printf(" Yaw: %.5f deg               ", yaw);


        fflush(stdout);
        if (read(STDIN_FILENO, &ch, 1) < 0) {
            // no input
        }
    }   
    // done
    set_nonblocking(0);
    bno055_close();
    return 0;
}