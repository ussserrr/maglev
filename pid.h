#ifndef CONST_LIGHT_FLOW_WITH_STRUCT_PID_H_
#define CONST_LIGHT_FLOW_WITH_STRUCT_PID_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <limits.h>

typedef struct _PIDdata {

    //pid factors
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;

    //pid terms
    volatile float Perr;
    volatile float Ierr;
    volatile float Derr;

    //pid terms limits
    volatile float Perrmin;
    volatile float Perrmax;
    volatile float Ierrmin;
    volatile float Ierrmax;

} PIDdata, *ptrPIDdata;


//functions
extern void PID_init(ptrPIDdata pPd);
extern void PID_setpid(ptrPIDdata pPd, float pidP, float pidI, float pidD);
extern void PID_setlimitsPerr(ptrPIDdata pPd, float Perr_min, float Perr_max);
extern void PID_setlimitsIerr(ptrPIDdata pPd, float Ierr_min, float Ierr_max);
extern void PID_resetIerr(ptrPIDdata pPd);
extern float PID_update(ptrPIDdata pPd, float setpoint, float input);

#endif
