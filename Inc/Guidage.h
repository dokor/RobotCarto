

#ifndef 	H_GUIDAGE
#define		H_GUIDAGE

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"



typedef struct {// Structure asservissement position 
	float 	Angle_Origine;
	float		Delta_Angle;
	float 	CosDelta_Angle;
	float		CoefP;
	float		CoefI;
	float		CoefD;
	float		NotUsed3;
} T_Suivi_De_Mur;

extern T_Suivi_De_Mur  Suivi_De_Mur;




extern float Guidage_Suivi_Mur (void) ;









#endif

