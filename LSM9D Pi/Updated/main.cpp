#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "LSM9D_Library.h"

int main(int argc, char *argv[])
{
    long int start_time;
    long int current_time;
    long int elapsed_time;
    struct timespec gettime_now;

    LSM9DS1 imu(0x6b, 0x1e);
    imu.begin();
    if (!imu.begin())
	{
        fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        exit(EXIT_FAILURE);
    }
    imu.calibrate();

    while(true)
    {
        clock_gettime(CLOCK_REALTIME, &gettime_now);
	    start_time = gettime_now.tv_nsec;
        while (!imu.gyroAvailable()) ;
        imu.readGyro();
        while(!imu.accelAvailable()) ;
        imu.readAccel();
        while(!imu.magAvailable()) ;
        imu.readMag();

        clock_gettime(CLOCK_REALTIME, &gettime_now);
	    current_time = gettime_now.tv_nsec;
		elapsed_time = current_time - start_time;
		if (elapsed_time < 0)
			elapsed_time += 1000000000;				//(Rolls over every 1 second)

        printf("Time:  %8.3f [s] ", (float)elapsed_time/1000000000.0);
        printf("Gyro:  %8.3f %8.3f %8.3f [deg/s] ", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
        printf("Accel: %8.3f %8.3f %8.3f [Gs] ", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
        printf("Mag:   %8.3f %8.3f %8.3f [gauss]\n", imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        //sleep(0.05);
    }

    exit(EXIT_SUCCESS);
}
