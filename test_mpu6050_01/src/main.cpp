#include "mbed.h"
#include <MPU6050.h>
#include <IMUfilter.h>


MPU6050 mpu(PTE25,PTE24);
Serial pc(USBTX,USBRX);


float gx,gy,gz,ax,ay,az;
float x, y, z, heading;
int16_t raw[3];
float acc[3];
float gry[3];


float accOffset[3]; //offset values
float gyroOffset[3];
float angle[3];

Ticker t;

double toDegrees(double ang){

	return (M_PI*ang)/180.0;
}

void imp(){
	pc.printf("%.3f || %.3f || %.3f\n", angle[0], angle[1], angle[2]);
}

int main()
{
	pc.baud(115200);

	//t.attach(&imp,0.6f);
	mpu.setAlpha(0);
	while(1){

	wait(0.1);

	    mpu.getOffset(accOffset, gyroOffset,1.0);

    	mpu.getAccelero(acc);
      mpu.getGyro(gry);
      mpu.getAcceleroAngle(angle);

			pc.printf("%.3f || %.3f || %.3f\n", angle[0], angle[1], angle[2]);

			//Aplicar el Filtro Complementario
		 angle[0] = 0.98 *(angle[0]+gry[0]*0.010) + 0.02*acc[0];
		 angle[1] = 0.98 *(angle[1]+gry[1]*0.010) + 0.02*acc[1];

     //mpu.computeAngle(angle, accOffset, gyroOffset,100);
		// wait(0.6);
    }
}

/*0.000 -3.491
0.000 -3.521
0.000 -3.550*/
