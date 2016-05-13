


#ifndef H_HMC5883L 
#define H_HMC5883L 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "typegen.h"

typedef struct {	
	float Compas_Xsf_Gain ; 	
	float Compas_X_Offset ;
	float Compas_Ysf_Gain ;
	float	Compas_Y_Offset ;
	float Compas_Zsf_Gain ;	
	float Compas_Z_Offset ;
	float	Compas_Position_Offset ; 
} T_HMC5883L_Compensation ;

extern T_HMC5883L_Compensation HMC5883L_Compensation ;

typedef struct {
	float Compas_X ;
	float Compas_Y ;	
} T_HMC5883L_CompasXY ;			/* utile lorsque HMC5883L_VALIDATION défini */ 
#define		MAX_COMPAS_XY_DANS_BUFFER	250


	/* Les états de l'automate de gestion Lidar */
typedef enum {
			HMC5883L_MESURE,													// paramètre de HMC5883L_Gestion et Requête Wifi
			HMC5883L_CALIBRATION,											// paramètre de HMC5883L_Gestion
			HMC5883L_CALCULE_CALIBRATION,
			HMC5883L_END_CALIBRATION, 
#ifdef HMC5883L_VALIDATION 
			HMC5883L_COLLECTE_SANS_CALIBRATION =  9,		// paramètre de Requête Wifi
			HMC5883L_COLLECTE_AVEC_CALIBRATION = 10			// paramètre de Requête Wifi
#endif
} E_Etat_Automate_HMC5883L ;

typedef volatile uint8_t T_Etat_Automate_HMC5883L ;




extern float Compas ; 


extern void HMC5883L_Start ( void ) ;


extern T_Etat_Automate_HMC5883L HMC5883L_Gestion ( T_Etat_Automate_HMC5883L Request ) ;

#ifdef HMC5883L_VALIDATION 	
extern void HMC5883L_Request (uint8_t	Command ) ;  
#endif
extern void HMC5883L_Emission_IT ( void ) ;

extern void HMC5883L_Reception_IT ( void );

extern void	HMC5883L_Event_IT (void); 

#endif


