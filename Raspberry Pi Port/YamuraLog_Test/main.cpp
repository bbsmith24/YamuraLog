#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include "I2C_Library/I2C_Library.h"
#include "LSM9D/LSM9D_Library.h"
#include "BNO080/BNO080_Library.h"
#include "SX1509/SX1509_Library.h"
#include "ADS1015/ADS1015_Library.h"

int main(int argc, char *argv[])
{
    long int start_nsec;
    long int current_nsec;
    long int elapsed_nsec;
    time_t start_sec;
    time_t current_sec;
    time_t elapsed_sec;
    struct timespec gettime_now;
    I2C_Library::I2CWord pins;

    LSM9DS1 imu(0x6b, 0x1e);
    BNO080  vrIMU;
    SX1509  ioExp;
    ADS1015 a2d;

    //vrIMU.begin();

    imu.begin();
    if (!imu.begin())
	{
        fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        exit(EXIT_FAILURE);
    }
    imu.calibrate();
    // digital i/o expander
    if(!ioExp.SX1509_Begin())
    {
        fprintf(stderr, "Failed to communicate with SX1509.\n");
        //exit(EXIT_FAILURE);
    }
    ioExp.pinMode(0, INPUT);
    ioExp.pinMode(1, INPUT);
    // A2D expander
    a2d.begin();

    // start!
    clock_gettime(CLOCK_REALTIME, &gettime_now);
    start_nsec = gettime_now.tv_nsec;
    start_sec = gettime_now.tv_sec;
    while(true)
    {
        clock_gettime(CLOCK_REALTIME, &gettime_now);
	    current_nsec = gettime_now.tv_nsec;
	    current_sec = gettime_now.tv_sec;
		elapsed_nsec = current_nsec - start_nsec;
		elapsed_sec = current_sec - start_sec;
		if (elapsed_sec < 0)
		{
			elapsed_nsec += 1000000000;				//(Rolls over every 1 second)
		}
        start_nsec = current_nsec;
        start_sec = current_sec;

        printf("Time:  %8.3f [s] ",  (float)elapsed_sec + (float)elapsed_nsec/1000000000.0);
        //printf("Gyro:  %8.3f %8.3f %8.3f [deg/s] ", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
        //printf("Accel: %8.3f %8.3f %8.3f [Gs] ", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
        //printf("Mag:   %8.3f %8.3f %8.3f [gauss]\n", imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        if (imu.gyroAvailable())
        {
            imu.readGyro();
            printf("Gyro:  %8.3f %8.3f %8.3f [deg/s] ", imu.calcGyro(imu.gyroVals.intVals[0]),
                                                        imu.calcGyro(imu.gyroVals.intVals[1]),
                                                        imu.calcGyro(imu.gyroVals.intVals[2]));
        }
        else
        {
            printf("Gyro:  -------- -------- -------- [deg/s] ");
        }
        if (imu.accelAvailable())
        {
            imu.readAccel();
            printf("Accel: %8.3f %8.3f %8.3f [Gs] ", imu.calcAccel(imu.accelVals.intVals[0]),
                                                     imu.calcAccel(imu.accelVals.intVals[1]),
                                                     imu.calcAccel(imu.accelVals.intVals[2]));
        }
        else
        {
            printf("Accel: -------- -------- -------- [Gs] ");
        }
        if (imu.magAvailable())
        {
            imu.readMag();
            printf("Mag:   %8.3f %8.3f %8.3f [gauss] ", imu.calcMag(imu.magVals.intVals[0]),
                                                         imu.calcMag(imu.magVals.intVals[1]),
                                                         imu.calcMag(imu.magVals.intVals[0]));
        }
        else
        {
            printf("Mag:   -------- -------- -------- [gauss] ");
        }
        ioExp.readPins(&pins);
        printf("DIGITAL: 0x%02X 0x%02X 0x%04X PIN_00 %d PIN_01 %d PIN_02 %d ", pins.byteVals[0],
                                                                  pins.byteVals[1],
                                                                  pins.intVal,
                                                                  ((pins.intVal & SX1509_PIN_00) > 0 ? 1 : 0),
                                                                  ((pins.intVal & SX1509_PIN_01) > 0 ? 1 : 0),
                                                                  ((pins.intVal & SX1509_PIN_02) > 0 ? 1 : 0));

        //uint16_t a2d_0_Val = a2d.getSingleEnded(0);
        //uint16_t a2d_1_Val = a2d.getSingleEnded(1);
        //uint16_t a2d_2_Val = a2d.getSingleEnded(2);
        uint16_t a2d_3_Val = a2d.readADC_SingleEnded(3);
        printf("A2D: 0x%04X %d  ", a2d_3_Val, a2d_3_Val);

        printf("\n");
        sleep(1);
    }

    exit(EXIT_SUCCESS);
}
