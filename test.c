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
    const double target_period = 0.01; // 100hz
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

        clock_gettime(CLOCK_MONOTONIC, &now);
        double loop_start = now.tv_sec + now.tv_nsec / 1e9;

        bno055_vector_t quat = bno055_getVectorQuaternion();
        double yaw = bno055_yawFromQuaternion(quat);




        counter++;
   

        clock_gettime(CLOCK_MONOTONIC, &now);
        double loop_end = now.tv_sec + now.tv_nsec / 1e9;
        double loop_time = loop_end - loop_start;


        printf("Quat (dps) -> W: %.3f  X: %.3f  Y: %.3f  Z: %.3f               \n", quat.w, quat.x, quat.y, quat.z);
        printf(" Yaw: %.5f deg               \n", yaw);


        double elapsed = target_period - loop_time;
        if (elapsed > 0) {
            usleep((useconds_t)(elapsed * 1e6));
        }

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