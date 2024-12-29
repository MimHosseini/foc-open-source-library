/*
 * File: foc.c
 * Driver Name: [[ FOC Basic Functions ]]
 * Created on: Jun, 2024
 * Author:     Mohammad Hosseini
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.esfdrive.ir
 *
 */
 
//** NOTE1: This library is just a starting point for implementing the FOC method
//    and may need some modifications and improvements


//** NOTE2: You MUST consider using look-up tables and fixed-point arithmetic when using low stream microcontrollers
// this library would probably work on a MCU supporting floating-point unit and ARM CMSIS DSP library without any major problem.

//** NOTE3: The sliding mode obserever has been implemented based on:
//1. Contributions to Discrete-Time Sliding Mode
//Observers for Permanent Magnet Synchronous
//Motor Drive Systems( refer to IEEE)

//2. A High-Speed Sliding-Mode Observer for the
//Sensorless Speed Control of a PMSM( refer to IEEE)
 
#ifndef _FOC_TRANSFORMS_H_
#define _FOC_TRANSFORMS_H_

#include "math.h"
#include "stdint.h"

#define SQRT3        1.73205080757f
#define PI           3.141592653589f
#define PI_3         1.047197551196f


//CLARKE_TRANSFORM
typedef struct clarke_transfrom
{
	float ia;
	float ib;
	float iAlpha;
	float iBeta;
	void(*clarke_fcn_ptr)(struct clarke_transfrom*);	
}CLARKE_TYPEDEF;

#define CLARKE_DEFAULT { \
.ia              = 0.0f, \
.ib              = 0.0f, \
.iAlpha          = 0.0f, \
.iBeta           = 0.0f, \
.clarke_fcn_ptr = Clarke_Transform\
}
//PARK_TRANSFORM
typedef struct park_transfrom
{
	float iAlpha;
	float iBeta;
	float iD;
	float iQ;
	float theta;
	void(*park_fcn_ptr)(struct park_transfrom*);
}PARK_TYPEDEF;

#define PARK_DEFAULT {\
.iAlpha       = 0.0f, \
.iBeta        = 0.0f, \
.iD           = 0.0f, \
.iQ           = 0.0f, \
.theta        = 0.0f, \
.park_fcn_ptr = Park_Transform \
}

//INVERSE_PARK_TRANSFORM
typedef struct inv_park_transform
{
	float vD;
	float vQ;
	float theta;
	float vAlpha;
	float vBeta;
	void(*inv_park_fcn_ptr)(struct inv_park_transform*);	
}INV_PARK_TYPEDEF;

#define INV_PARK_DEFAULT {\
.vD               = 0.0f, \
.vQ               = 0.0f, \
.theta            = 0.0f, \
.vAlpha           = 0.0f, \
.vBeta            = 0.0f, \
.inv_park_fcn_ptr = Inverse_Park_Transform \
}
//FIIR FILTER
typedef struct FIIR_filter
{
	float alpha;
	float filterInput;
	float filterOut;
	void(*LPF_fcn_ptr)(struct FIIR_filter*);
}LPF_TYPEDEF;

#define LPF_DEFAULT {\
.alpha       = 0.0f,\
.filterInput = 0.0f,\
.filterOut   = 0.0f,\
.LPF_fcn_ptr = LPF_Filter\
}


//PI CONTROLLER WITH ANTI-WINDUP
typedef struct pi_controller
{
	float input;
	float reference;
	float integralTerm;
	float outMin;
	float outMax;
	float kp;
	float ki;
	float kc;
	float controllerOut;
	void (*pi_fcn_ptr)(struct pi_controller*);
}PI_TYPEDEF;

#define PI_DEFAULT {\
.input         = 0.0f,\
.reference     = 0.0f,\
.integralTerm  = 0.0f,\
.outMin        = 0.0f,\
.outMax        = 0.0f,\
.kp            = 0.0f,\
.ki            = 0.0f,\
.controllerOut = 0.0f,\
.kc            = 0.0f,\
.pi_fcn_ptr    = PI_Controller \
}

//SPACE VECTOR MODULATION
typedef struct svmHandler
{
	float vAlpha;
	float vBeta;
	float maxMa;
	uint16_t VDC;
	uint32_t timerARR;
	uint32_t dutyA;
	uint32_t dutyB;
	uint32_t dutyC;
	void (*svm_fnc_ptr)(struct svmHandler*);
}SVM_TYPEDEF;


#define SVM_DEFAULT {       \
.vAlpha      = 0.0f,        \
.vBeta       = 0.0f,        \
.maxMa       = 0.0f,        \
.VDC         = 0,           \
.timerARR    = 0,           \
.dutyA       = 0,           \
.dutyB       = 0,           \
.dutyC       = 0,           \
.svm_fnc_ptr = SVM_Calculate\
}

//SMO OBSERVER FOR SENSORLESS APPLICATIONS
typedef struct smoHandler
{
		float k;
		float a;
		float Rs;
		float Ls;
		float iAlpha;
		float iBeta;
		float vAlpha;
		float vBeta;
		float iAlphaHat;
		float iBetaHat;
		float diAlphaHat;
		float diBetaHat;
		float Ts;
		float eAlpha;
		float eBeta;
		float theta;
		void(*smo_fcn_ptr)(struct smoHandler*);	
}SMO_TYPEDEF;



#define SMO_DEFAULT {\
.k = 0.0f,\
.a = 0.0f,\
.Rs = 0.0f,\
.Ls = 0.0f,\
.iAlpha = 0.0f,\
.iBeta = 0.0f,\
.vAlpha = 0.0f,\
.vBeta = 0.0f,\
.iAlphaHat = 0.0f,\
.iBetaHat = 0.0f,\
.diAlphaHat = 0.0f,\
.diBetaHat = 0.0f,\
.Ts = 0.0f,\
.eAlpha = 0.0f,\
.eBeta = 0.0f,\
.theta = 0.0f,\
.smo_fcn_ptr = SMO_Observer\
}

//FUNCTION PROTOTYPES 
void Clarke_Transform(CLARKE_TYPEDEF* clarke_handler);
void Park_Transform(PARK_TYPEDEF* park_handler);
void Inverse_Park_Transform(INV_PARK_TYPEDEF* inv_park_handler);
void LPF_Filter(LPF_TYPEDEF* lpf);
void PI_Controller(PI_TYPEDEF* pi);
void SVM_Calculate(SVM_TYPEDEF* svm);
void SMO_Observer(SMO_TYPEDEF* smo);
#endif


