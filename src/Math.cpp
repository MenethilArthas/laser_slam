#include "Math.h"

const double PI	=  3.14159265358979323846;  // The value of PI

double DegreesToRadians(double degrees)
{

	return degrees * PI/180.0;
}

double NormalizeAngle(double angle)
{
	
	while(angle<-PI)
		angle+=2*PI;
	while(angle>PI)
		angle-=2*PI;
	return angle;
}
