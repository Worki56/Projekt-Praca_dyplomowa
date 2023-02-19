#include <gui/screen1_screen/Screen1View.hpp>
#include <cstring>


extern uint16_t asdUART1[3];
extern uint16_t asdGPS[3];
extern uint16_t asdCZUJNIKI[3];
extern int8_t zapisekran;


Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
	if(zapisekran==0){
		toggleButton1.forceState(false);
	}
	else{
		toggleButton1.forceState(true);
	}
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::handleTickEvent()
{


	Unicode::snprintf(tA6_0Buffer,TA6_0_SIZE,"%d",asdGPS[0]);
	Unicode::snprintf(tA6_1Buffer,TA6_1_SIZE,"%d",asdGPS[1]);
	Unicode::snprintf(tA6_2Buffer,TA6_2_SIZE,"%d",asdGPS[2]);
	Unicode::snprintf(tA7_0Buffer,TA7_0_SIZE,"%d",asdUART1[0]);
	Unicode::snprintf(tA7_1Buffer,TA7_1_SIZE,"%d",asdUART1[1]);
	Unicode::snprintf(tA7_2Buffer,TA7_2_SIZE,"%d",asdUART1[2]);
	Unicode::snprintf(tA10_0Buffer,TA10_0_SIZE,"%d",asdCZUJNIKI[0]);
	Unicode::snprintf(tA10_1Buffer,TA10_1_SIZE,"%d",asdCZUJNIKI[1]);

	tA6_0.invalidate();
	tA6_1.invalidate();
	tA6_2.invalidate();
	tA7_0.invalidate();
	tA7_1.invalidate();
	tA7_2.invalidate();
	tA10_0.invalidate();
	tA10_1.invalidate();
	toggleButton1.invalidate();

	if(toggleButton1.getState()==false){
		zapisekran=0;
		toggleButton1.forceState(false);
	}
	else{
		zapisekran=1;
		toggleButton1.forceState(true);
	}

}
