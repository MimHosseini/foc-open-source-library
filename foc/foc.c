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



#include "foc.h"

//CLARKE_TRANSFORM
void Clarke_Transform(CLARKE_TYPEDEF* clarke_handler)
{
	clarke_handler->iAlpha = clarke_handler->ia;
	clarke_handler->iBeta = (clarke_handler->ia + (2 * clarke_handler-> ib)) / SQRT3;
}
//PARK_TRANSFORM
void Park_Transform(PARK_TYPEDEF* park_handler)
{
	park_handler->iD = (park_handler->iAlpha * cosf(park_handler->theta)) + (park_handler->iBeta * sinf(park_handler->theta));
	park_handler->iQ = (-park_handler->iAlpha *  sinf(park_handler->theta)) + (park_handler->iBeta * cosf(park_handler->theta));
}
//INVERSE_PARK_TRANSFORM
void Inverse_Park_Transform(INV_PARK_TYPEDEF* inv_park_handler)
{
	inv_park_handler->vAlpha = (inv_park_handler->vD * cosf(inv_park_handler->theta)) + (-inv_park_handler->vQ * sinf(inv_park_handler->theta));
	inv_park_handler->vBeta = (inv_park_handler->vD * sinf(inv_park_handler->theta)) +  (inv_park_handler->vQ * cosf(inv_park_handler->theta));
}
//FIIR FILTER
void LPF_Filter(LPF_TYPEDEF* lpf)
{
	lpf->filterOut = (lpf->alpha * lpf->filterInput) + ((1 - lpf->alpha) * lpf->filterOut);
}
//PI CONTROLLER WITH ANTI-WINDUP
void PI_Controller(PI_TYPEDEF* pi)
{
	
	float error = pi->reference - pi->input;
	float effort = pi->integralTerm + (pi->kp * error);
	
	if(effort > pi->outMax) pi->controllerOut = pi->outMax;
	else if(effort < pi->outMin) pi->controllerOut = pi->outMin;
	else pi->controllerOut = effort;
	
	float excess = effort - pi->controllerOut;
	
	pi->integralTerm += (pi->ki * error) - (pi->kc * excess);	
}
//SPACE VECTOR MODULATION
void SVM_Calculate(SVM_TYPEDEF* svm)
{
	float vRef = sqrtf(svm->vAlpha * svm->vAlpha + svm->vBeta * svm->vBeta );
	float alpha = atan2f(svm->vBeta,svm->vAlpha);
	if(alpha < 0)
		alpha += 6.28318530717956f;
	if (alpha == 6.28318530717956f)
		alpha = 0;
	
	uint8_t sector = 3 * alpha / PI;
	sector++;
	
	float ma = vRef / svm->VDC;
	
	if(ma > svm->maxMa) ma = svm->maxMa;
	
	float T1,T2,T0;
	T1 = ma * sinf(sector * PI_3 - alpha);
	T2 = ma * sinf(alpha - (sector - 1) * PI_3);
	T0 = 1 - T1 - T2;
	
	float sa,sb,sc;
	
	sa = T1 * svm->timerARR;
	sb = T2 * svm->timerARR; 
	sc = T0 * svm->timerARR; 
	
	switch (sector)
	{
		case 1:
		svm->dutyA = sa + sb + sc / 2;
		svm->dutyB = sb + sc / 2;
		svm->dutyC = sc / 2;
		break;
		case 2:
		svm->dutyA = sa + sc / 2;
		svm->dutyB = sa + sb + sc / 2;
		svm->dutyC = sc / 2;
		break;
		case 3:
		svm->dutyA = sc / 2;;
		svm->dutyB = sa + sb + sc / 2;
		svm->dutyC = sb + sc / 2;
		break;
		case 4:
		svm->dutyA = sc / 2;
		svm->dutyB = sa + sc / 2;
		svm->dutyC = sa + sb + sc / 2;
		break;
		case 5:
		svm->dutyA = sb + sc / 2;
		svm->dutyB = sc / 2;
		svm->dutyC = sa + sb + sc / 2;
		break;
		case 6:
		svm->dutyA = sa + sb + sc / 2;
		svm->dutyB = sc / 2;
		svm->dutyC = sa + sc / 2;
		break;		
	}
}
//SMO OBSERVER FOR SENSORLESS APPLICATIONS
//NOTE : determining the k coefficient of the sliding mode observer must be considered in such a way 
//       that reduces chattering and oscillation and gives a fairly quick response
void SMO_Observer(SMO_TYPEDEF* smo)
{
	smo->iAlphaHat += (smo->diAlphaHat * smo->Ts);
	smo->iBetaHat  +=  (smo->diBetaHat * smo->Ts);
	
	float errorAlpha = smo->iAlphaHat - smo->iAlpha;
	float errorBeta = smo->iAlphaHat - smo->iAlpha;
	
	float H_Alpha = smo->k * ((2 / (1 + exp(-smo->a * errorAlpha))) - 1);
	float H_Betta = smo->k * ((2 / (1 + exp(-smo->a * errorBeta))) - 1);
	
	smo->diAlphaHat = ((-smo->Rs / smo->Ls) * smo-> iAlphaHat) + ((1 / smo->Ls) * smo->vAlpha) - ((1 / smo->Ls) * H_Alpha);
	smo->diBetaHat = ((-smo->Rs / smo->Ls) * smo-> iBetaHat) + ((1 / smo->Ls) * smo->vBeta) - ((1 / smo->Ls) * H_Betta);
	
	smo->eAlpha =  H_Alpha;
	smo->eBeta  =   H_Betta;
	
	smo-> theta = - atan2f(smo->eAlpha,smo->eBeta);

	// Now we can filter theta if needed.
	// Omega is derived from theta derivative.	
	//If needed, we need to get it digitally
}

