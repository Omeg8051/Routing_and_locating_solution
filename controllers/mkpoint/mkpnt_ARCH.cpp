// File:          mkpoint.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#define W_RAD 0.009
#define W_DST 0.0408
#define PI 3.1415926

#define _VEL 0.15
#define _TH 13
#define _SHMITT_CYC 30
#define _MARG 0.002
#define ORE_TOL 0.001
#define PRLL_MARG 5

#define D_NOISE_BFR_LEN 16 //powers of 2
#define D_NOISE_SFT_LEN 4 //2 to the this power == D_NOISE_BFR_LEN

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Pen.hpp>
#include "diffPosLib/diffPos.h"
#include "diffPosLib/diffPos.cpp"
#include <math.h>

#define R 0
#define L 1

#define SMPR 2

//avoid obs const

#define TURN_BIAS 0.15
#define SLOW_DOWN 0.12
#define WALL_BIAS 0.08
#define WALL_TAR 30 //Jessie We need to COOK!!!!!
#define CORN_BIAS 0.01
#define CORN_LIM 30
#define WALL_FL_VEL 2.0f
#define CRU_SLO 0.01
#define CRU_SLO_TH 10

/*programe in :
Length  Meters
Angle   Rads
Time    Seconds

*/

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
using namespace diffpos;

double trd = 0;
PositionSensor* wp[2];
Motor* mot[2];
Pos2D posdat[2];
float buff_enco[2][2] = {
	{0.0f,0.0f},
	{0.0f,0.0f}

};
float diff_enco[2];
float vel[2];
float buff_time[2] = { 0.0f,0.0f };
float diff_time;
bool prime = 0;
unsigned long pp = 0;


//obs avoid
const char* px_name[8]{
	"ps0",
	"ps1",
	"ps2",
	"ps3",
	"ps4",
	"ps5",
	"ps6",
	"ps7"

};
const char* led_name[9]{
	"led0",
	"led1",
	"led2",
	"led3",
	"led4",
	"led5",
	"led6",
	"led7",
	"ledrgb"
};
DistanceSensor* px[8];
float PX_STAT = 0;
long pxdat_raw[8][D_NOISE_BFR_LEN];//{each PX DAT:{denoised data,{raw data shift}}}
long pxdat[8];//{each PX DAT:{denoised data,{raw data shift}}}

float wtd = 0;
float vR = 0, vL = 0;
LED* led[9];
Pen* pen;
Robot* robot;
unsigned long jnw = 0;
bool wallSide = R;


/*
FUNCTIONS
*/
//void readPx(DistanceSensor* px[8], float* pxdat);

//char DriveWheel(float pxdat[8], float* oR, float* oL, float TH);

bool detectWall(DistanceSensor* px[8], long* pxdat);

float getDtoOrg();

void updatePose();

void deNoiseTick(long* output, long raw[][D_NOISE_BFR_LEN], DistanceSensor* sensor[8], unsigned char bufferLen);
/*
main
*/
int main(int argc, char** argv) {

// create the Robot instance.
	robot = new Robot();

// get the time step of the current world.
	int timeStep = (int)robot->getBasicTimeStep();

//get and initiallize all sensors
	wp[L] = robot->getPositionSensor("left wheel sensor");
	wp[R] = robot->getPositionSensor("right wheel sensor");
	wp[L]->enable(1);
	wp[R]->enable(1);

	mot[R] = robot->getMotor("right wheel motor");
	mot[L] = robot->getMotor("left wheel motor");

	mot[R]->setPosition(INFINITY);
	mot[L]->setPosition(INFINITY);
	
	mot[R]->setVelocity(0);
	mot[L]->setVelocity(0);

//pen initiallize
	pen = robot->getPen("pen");
	pen->setInkColor(0xFF0000, 1.0);
	pen->write(1);

//enable sensors
	for (int i = 0; i < 8; i++) {
		px[i] = robot->getDistanceSensor(px_name[i]);
		px[i]->enable(1);
		led[i] = robot->getLED(led_name[i]);

	}
	led[8] = robot->getLED(led_name[8]);
	led[8]->set(0xFF0000);

	//posdat[0].X = W_DST / 2;
   

//prime sensor de noise buffer

	for (int i = 0; (i < D_NOISE_BFR_LEN*3) && (robot->step(timeStep) != -1); i++)deNoiseTick(pxdat, pxdat_raw, px, D_NOISE_BFR_LEN);
	wtd += robot->getTime();

	while (robot->step(timeStep) != -1) {

		/*
		collect data
		*/
		updatePose();

		/*
		path generation
		*/
		trd = sqrt(W_DST * _VEL * (buff_time[0] - wtd) / PI) + W_DST / 2;
		trd /= 2;
		float vel_ratio = (trd + W_DST / 2) / (trd - W_DST / 2);  //Transfer from L to R

		float VL = _VEL / (1 + vel_ratio) / W_RAD;
		float VR = VL * vel_ratio;

		PX_STAT = 0;
		for (char i = 0; i < 8; i++)PX_STAT += pxdat[i];

		//PX_STAT = exp(-PX_STAT);

		mot[R]->setVelocity(VR);
		mot[L]->setVelocity(VL);

		/*
		wall encountered
		*/
		if (detectWall(px, pxdat) && jnw==0) {
			float NTRD = getDtoOrg();
			float dst_diff[2];
			dst_diff[0] = abs(getDtoOrg() - NTRD);
			led[8]->set(wallSide ? 0x00FF00 : 0x0000FF);
			jnw = _SHMITT_CYC;
			pen->setInkColor(0x00ff00, 1.0);

			/*wall handler*/
			float wts = buff_time[0];
			cout << wallSide << endl;

			/*wall follower loop*/
			while (robot->step(timeStep) != -1 && ((dst_diff[0] > _MARG || ((dst_diff[1] - dst_diff[0]) < 0)) || jnw != 1)) {
				//shift data
				dst_diff[1] = dst_diff[0];
				
				float VL_ , VR_;
				updatePose();
				dst_diff[0] = abs(getDtoOrg() - NTRD);

				/*debug stdout*/
				//print(posdat[0]);
				cout << "WALL!!" << dst_diff[0] << '\t' << _MARG << '\t' << jnw << endl;

				
				deNoiseTick(pxdat, pxdat_raw, px, D_NOISE_BFR_LEN);

				char pxd_lkup = wallSide ? 6 : 2;
				if (jnw == _SHMITT_CYC) {

					mot[R]->setVelocity(wallSide ? -1 : 1);
					mot[L]->setVelocity(wallSide ? 1 : -1);

					if (abs(pxdat[pxd_lkup + 1] - pxdat[pxd_lkup - 1]) < PRLL_MARG)jnw -= 1;
					else continue;
				}
				else {
					jnw -= (jnw != 1);
				}
				
				/*for (int i = 0; i < 8; i++) {
					pxdat[i] = px[i]->getValue();

				}*/
				
				if (wallSide) {
					//VR_ += (-pxdat[0] * SLOW_DOWN - pxdat[6] * WALL_BIAS);
					//VL_ += ((pxdat[7] - pxdat[5]) * TURN_BIAS + pxdat[6] * WALL_BIAS + (pxdat[0] * SLOW_DOWN));
					VR_ = VL - (pxdat[0] + pxdat[1] + pxdat[7]) * SLOW_DOWN;
					VL_ = VR_ + ((pxdat[7] - pxdat[5]) * TURN_BIAS + (((pxdat[7] + pxdat[6]) >> 1) - WALL_TAR) * WALL_BIAS);
					VL_ *= ((pxdat[6] + pxdat[7]) < CORN_LIM) ? CORN_BIAS : 1;

				}
				else {
					//VL += (-pxdat[0] * SLOW_DOWN - (pxdat[1] + pxdat[3]) * WALL_BIAS - (pxdat[1] - pxdat[3]) * TURN_BIAS);
					VL_ = VL - (pxdat[0] + pxdat[1] + pxdat[7]) * SLOW_DOWN;
					VR_ = VL_ + ((pxdat[1] - pxdat[3]) * TURN_BIAS + (((pxdat[1] + pxdat[2]) >> 1) - WALL_TAR) * WALL_BIAS);
					VR_ *= ((pxdat[2] + pxdat[1]) < CORN_LIM) ? CORN_BIAS : 1;

				}

				/*motor Drive*/
				mot[R]->setVelocity(VR_);
				mot[L]->setVelocity(VL_);

				//get new value for distance diff op
				
			} 
			led[8]->set(0xFF0000);

			/*exit wall follower and rotate back to correct orientation*/
			wtd += buff_time[0] - wts;

			/*calc orientation*/
			float ore = acos(posdat[0].X / getDtoOrg());
			ore += (posdat[0].Y < 0) * PI;
			//ore += 2 * PI * (ore < 0) - (ore > 2 * PI) * 2 * PI;
			cout << "new DIR: " << ore << endl;
			//while (1);
			/*mov to orientation*/

			pen->setInkColor(0xff0000, 1.0);
			float orediff = abs(posdat[0].dir - ore);
			while (robot->step(timeStep) != -1 && orediff > ORE_TOL) {
				mot[R]->setVelocity(orediff+0.2);
				mot[L]->setVelocity(-1 * (orediff + 0.2));
				updatePose();
				orediff = abs(posdat[0].dir - ore);
				cout << "ORE DIFF: " << (abs(posdat[0].dir - ore)) << endl;
			}

			
		} else if (!detectWall(px, pxdat)) {
			jnw = 0;
		}
		
		/*
		print result
		*/
		if (pp++ % SMPR == 0) {
			//cout << "LV: " << vel[L] << "\tRV: " << vel[R] << endl;
			//cout << px_sum << endl;
			//print(posdat[0]);
			//cout << "trd: " << trd << '\t' << getDtoOrg() << '\t' << posdat[0].X << '\t' << posdat[0].Y << "\tPX_STAT: " << PX_STAT << endl;
			cout << "trd: " << trd << endl;
			/*for (char i = 0; i < 8; i++) {
				cout << pxdat[i] << '\t';
			}*/
			//print(posdat[0]);
			//printf("%f\n",diff_time);
			//cout << endl;
		}
	};

  // Enter here exit cleanup code.

	delete robot;
	return 0;
}

bool detectWall(DistanceSensor* px[8], long* pxdat)
{
	/*
	legacy px sensor code
	*/
	/*
	for (int i = 0; i < 8; i++) {
		pxdat[i] = px[i]->getValue();

	}*/
	deNoiseTick(pxdat, pxdat_raw, px,D_NOISE_BFR_LEN);
	char i = 0;
	for (; i < 3; i++) {
		if (pxdat[i + 5*(i==2)] > _TH) {
			wallSide = (i == 2);
			return 1;

		}
	}
	//led[8]->set(nearWall ? 0x00FF00 : 0xFF0000);
	return 0;
}

float getDtoOrg()
{

	return sqrt(posdat[0].X* posdat[0].X + posdat[0].Y* posdat[0].Y);
}

void updatePose()
{
	//shift posdat
	posdat[1] = posdat[0];
	// get VR & VL
	buff_enco[1][L] = buff_enco[0][L];
	buff_enco[1][R] = buff_enco[0][R];
	buff_enco[0][L] = wp[L]->getValue();
	buff_enco[0][R] = wp[R]->getValue();
	diff_enco[R] = buff_enco[0][R] - buff_enco[1][R];
	diff_enco[L] = buff_enco[0][L] - buff_enco[1][L];
	buff_time[1] = buff_time[0];
	buff_time[0] = robot->getTime();
	diff_time = buff_time[0] - buff_time[1];
	diff_enco[R] /= diff_time;
	diff_enco[L] /= diff_time;

	vel[R] = W_RAD * diff_enco[R];
	vel[L] = W_RAD * diff_enco[L];

	//position intigration;
	if (prime)diffpos::transform(&posdat[0], posdat[1], vel[R], vel[L], diff_time);
	else {
		prime = 1;
		posdat[0].X = W_DST / 2;
	}

}

void deNoiseTick(long* output, long raw[][D_NOISE_BFR_LEN], DistanceSensor* sensor[8], unsigned char bufferLen)
{
	// shift in from position 0!!
	for (unsigned char i = 0; i < 8; i++) {//PX read 1 by 1 layer
		output[i] = 0;
		for (unsigned char j = D_NOISE_BFR_LEN - 1; j > 0; j--) {//shift reg and calc average layer
			raw[i][j] = raw[i][j - 1];
			output[i] += raw[i][j];

		}
		raw[i][0] = ((long)sensor[i]->getValue())>>D_NOISE_SFT_LEN;
		output[i] += raw[i][0];
	}
}
