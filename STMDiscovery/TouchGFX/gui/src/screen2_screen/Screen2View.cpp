#include <gui/screen2_screen/Screen2View.hpp>
#include <cstring>
#include "mpu6050.h"

extern MPU6050_t MPU6050;
extern float TempDH, WilgDH;
extern float TempBMP;
extern int32_t CisnBMP;


Screen2View::Screen2View()
{

}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();
}

void Screen2View::handleTickEvent()
{

	Unicode::snprintfFloat(tA2Buffer,TA2_SIZE,"%03.6f",TempDH);
	Unicode::snprintfFloat(tA3Buffer,TA3_SIZE,"%03.6f",WilgDH);
	Unicode::snprintfFloat(tA4Buffer,TA4_SIZE,"%03.6f",TempBMP);
	Unicode::snprintf(tA5Buffer,TA5_SIZE,"%d",CisnBMP);
	Unicode::snprintf(tA6Buffer,TA6_SIZE,"%i",MPU6050.Mag_X_RAW);
	Unicode::snprintf(tA7Buffer,TA7_SIZE,"%i",MPU6050.Mag_Y_RAW);
	Unicode::snprintf(tA8Buffer,TA8_SIZE,"%i",MPU6050.Mag_Z_RAW);
	Unicode::snprintfFloat(tA9Buffer,TA9_SIZE,"%03.6f",MPU6050.Gx);
	Unicode::snprintfFloat(tA10Buffer,TA10_SIZE,"%03.6f",MPU6050.Gy);
	Unicode::snprintfFloat(tA11Buffer,TA11_SIZE,"%03.6f",MPU6050.Gz);
	Unicode::snprintfFloat(tA12Buffer,TA12_SIZE,"%03.6f",MPU6050.Ax);
	Unicode::snprintfFloat(tA13Buffer,TA13_SIZE,"%03.6f",MPU6050.Ay);
	Unicode::snprintfFloat(tA14Buffer,TA14_SIZE,"%03.6f",MPU6050.Az);

	tA2.invalidate();
	tA3.invalidate();
	tA4.invalidate();
	tA5.invalidate();
	tA6.invalidate();
	tA7.invalidate();
	tA8.invalidate();
	tA9.invalidate();
	tA10.invalidate();
	tA11.invalidate();
	tA12.invalidate();
	tA13.invalidate();
	tA14.invalidate();

}
