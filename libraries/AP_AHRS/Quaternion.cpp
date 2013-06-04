/*
#include <iostream>
#include <cmath>
using namespace std;


#include "Quaternion.h"

/////////////////////////////////////////////////////////////////// Quaternion functions

// Quaternion state difference function creates new quaternion
quaternion quaternion::operator- (quaternion B) {
	quaternion temp;
	temp.q1 = q1 - B.q1;
	temp.q2 = q2 - B.q2;
	temp.q3 = q3 - B.q3;
	temp.q4 = q4 - B.q4;
	return (temp);
}

// Quaternion error calculation based on Hamilton equation
// qc is commanded quaternion
quaternion quaternion::qerror (quaternion qc) {
	quaternion temp;
	temp.q1 =  q4*qc.q1 + q3*qc.q2 - q2*qc.q3 - q1*qc.q4;
	temp.q2 = -q3*qc.q1 + q4*qc.q2 + q1*qc.q3 - q2*qc.q4;
	temp.q3 =  q2*qc.q1 - q1*qc.q2 + q4*qc.q3 - q3*qc.q4;
	temp.q4 =  q1*qc.q1 + q2*qc.q2 + q3*qc.q3 + q4*qc.q4;
	return (temp);
}

AP_AHRS quaternion::quaternion2euler () {
	AP_AHRS temp;
	temp.roll = atan2(2*(q2*q3 + q4*q1) , (1 - 2*(pow(q1,2) + pow(q2,2))));
	temp.pitch = asin(-2*(q1*q3 - q4*q2));
	temp.yaw = atan2(2*(q1*q2 + q4*q3) , (1 - 2*(pow(q2,2) + pow(q3,2))));
	return (temp);
}
*/