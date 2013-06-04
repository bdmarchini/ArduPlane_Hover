/*
#ifndef QUATERNION_H
#define QUATERNION_H

#include "AP_AHRS.h"

// Quaternion class definition
class quaternion {
public: 
	float q1, q2, q3, q4;
	quaternion () {};
	quaternion (float a, float b, float c, float d) {
		q1 = a;
		q2 = b;
		q3 = c;
		q4 = d;
	}
	quaternion operator - (quaternion);
	quaternion qerror (quaternion);
	void disp_quaternion ();
	friend class AP_AHRS;	
	AP_AHRS quaternion2euler ();
};

#endif
*/