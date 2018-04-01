#include "pid.h"

//uint16_t test = 324;

void PID_init(ptrPIDdata pPd) {
	pPd->Kp = 0.0f;
	pPd->Ki = 0.0f;
	pPd->Kd = 0.0f;

	pPd->Perr = 0.0f;
	pPd->Ierr = 0.0f;
	pPd->Derr = 0.0f;

	pPd->Perrmin = -100000.0f;
	pPd->Perrmax = 100000.0f;
	pPd->Ierrmin = -100000.0f;
	pPd->Ierrmax = 100000.0f;
}

/*
 * set pid terms
 */
void PID_setpid(ptrPIDdata pPd, float pidP, float pidI, float pidD) {
	pPd->Kp = pidP;
	pPd->Ki = pidI;
	pPd->Kd = pidD;
}


/*
 * set P term limits
 */
void PID_setlimitsPerr(ptrPIDdata pPd, float Perr_min, float Perr_max) {
	pPd->Perrmin = Perr_min;
	pPd->Perrmax = Perr_max;
}


/*
 * set I term limits
 */
void PID_setlimitsIerr(ptrPIDdata pPd, float Ierr_min, float Ierr_max) {
	pPd->Ierrmin = Ierr_min;
	pPd->Ierrmax = Ierr_max;
}


/*
 * reset I term limit
 */
void PID_resetIerr(ptrPIDdata pPd) {
	pPd->Ierr = 0;
}


/*
 * pid control algorithm
 */
float PID_update(ptrPIDdata pPd, float setpoint, float input) {
	//if this function get called always at the same period, dt = 1 can be used
	//otherwise dt should be calculated
	static float inputprev = 0;

	//compute P error
	pPd->Perr = setpoint - input;
	if (pPd->Perr < pPd->Perrmin)
		pPd->Perr = pPd->Perrmin;
	else if (pPd->Perr > pPd->Perrmax)
		pPd->Perr = pPd->Perrmax;

	//compute I error
	pPd->Ierr += pPd->Perr;
	if (pPd->Ierr < pPd->Ierrmin)
		pPd->Ierr = pPd->Ierrmin;
	else if (pPd->Ierr > pPd->Ierrmax)
		pPd->Ierr = pPd->Ierrmax;

	//compute D error
	pPd->Derr = (inputprev - input);

	//record last value
	inputprev = input;

	//compute output
	float output = (pPd->Kp*pPd->Perr) + (pPd->Ki*pPd->Ierr) + (pPd->Kd*pPd->Derr);

	return output;
}
