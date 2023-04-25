#include "diffPos.h"
#include <iostream>

typedef struct diffpos::Pos2D {
	float X;
	float Y;
	float dir;
};

int diffpos::diff2rad(float* rad, float vr, float vl) {
	/*
	vl * r = vr * r + vr * x
	(vl - vr)*r = vr * x
	*/
	if (vr == vl)return -1;                 // same velocity, same dir
	else if (vr == (vl * -1)) {
		*rad = 0;
		return -2;                          // same velocity, oppsite dir
	}
	else {
		*rad = (vr + vl) / (vl - vr) * W_DST / 2;
		return 0;                           //normal operation
	}
}

int diffpos::diff2V(float* vol, float vr, float vl) {

	*vol = (vr + vl) / 2;                     //no execption in velocity
	return 0;
}

void diffpos::switchOrig(Pos2D* out, Pos2D in, Pos2D newOrg) {
	out->X = in.X - newOrg.X;
	out->Y = in.Y - newOrg.Y;
	out->dir = in.dir;
}

void diffpos::rad2rotCent(Pos2D* cent, Pos2D pos_cur, float radius) {
	cent->X = pos_cur.X + radius * cos(pos_cur.dir);
	cent->Y = pos_cur.Y + radius * sin(pos_cur.dir);
	cent->dir = pos_cur.dir;
}

void diffpos::rotPosByAngle(Pos2D* out, Pos2D pos_cur, float angle) {
	out->X = pos_cur.X * cos(angle) + pos_cur.Y * sin(angle) * -1;
	out->Y = pos_cur.X * sin(angle) + pos_cur.Y * cos(angle);
	out->dir = pos_cur.dir + angle;
}

int diffpos::transform(Pos2D* out, Pos2D in, float vr, float vl, float time_diff) {
	float vel = 0, rad = 0, omega = 0;
	int stat = 0;

	stat = diff2rad(&rad, vr, vl);
	diff2V(&vel, vr, vl);
	
	if (stat == -1) {
		out->X = in.X + vel * cos(in.dir + PI / 2) * time_diff/2;
		out->Y = in.Y + vel * sin(in.dir + PI / 2) * time_diff/2;
		out->dir = in.dir;
	}
	else if (stat == -2) {
		omega = vr / (W_DST);
		out->X = in.X;
		out->Y = in.Y;
		out->dir = in.dir + omega * time_diff;


	}
	else {
		omega = vel / rad * -0.5 * time_diff;
		Pos2D cent;
		rad2rotCent(&cent, in, rad);
		Pos2D posp;
		switchOrig(&posp, in, cent);
		rotPosByAngle(out, posp, omega);
		cent.X = cent.X * -1;
		cent.Y = cent.Y * -1;
		switchOrig(out, *out, cent);
		//out->dir *= (2 / 1.94719707);

	}

	out->dir += out->dir > 2 * PI ? -2 * PI : out->dir < 0 ? 2 * PI : 0;
	//std::cout << "rad: " << rad << std::endl;

	return stat;
}

int diffpos::print(Pos2D pos) {
	std::cout << '[' << pos.Y << ',' << pos.X << ',' << pos.dir << ']';
	return 0;
}
