/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "i2c.h"
#include "typegen.h"
#include "Tempo.h"
#include "Moteurs.h"
#include "WifiCom.h"
#include "HMC5883L.h"
#include "stdlib.h"
#include "Math.h"



/* Renomme le bus I2C utilisé */
#define	HMC5883L_I2c				hi2c1
	/* en cas de modification, modifier également le fichier stm32f4xx_it.c */
#define	HMC5883L_ADDRESS		0x3C

/* à décocher pour utiliser XYZ sinon XY (réduction taille code et + rapide )*/
//#define USE_AXEL_Z


typedef enum {
		CONFIGURATION_A, 
		CONFIGURATION_B, 
		MODE,
		DATA_X_MSB,
		DATA_X_LSB, 
		DATA_Z_MSB,
		DATA_Z_LSB, 
		DATA_Y_MSB,
		DATA_Y_LSB, 
		STATUS,
		IDENTIFICATION_A,
		IDENTIFICATION_B,
		IDENTIFICATION_C	
} E_HMC5883L_Register ;

typedef	struct {	
	uint8_t						 		I2CTx_Buf[2];			// buffer émission d'un message I2C pour envoi config
	uint8_t						 		I2CRx_Buf[6] ;		// les 6 datas
	uint8_t								If_Req_Request ;	// si !=0 It émission lance réception
	volatile T_Boolean		Done ;						// VRAI lorsqu'une transaction I2C est terminée
	volatile T_Boolean		DRDY_Ready ;			// VRAI lorsque les datas sont prêt à être lu 
	volatile T_Boolean	  Data_Ready ; 			// VRAI lorsque les datas ont été lus par It réception 
} T_HMC5883L_Communication ;
T_HMC5883L_Communication HMC5883L_Communication ; 

/* Valeurs lues par it réception */
int16_t	Compas_X ;
int16_t	Compas_Z ;
int16_t	Compas_Y ;

	/* définition pour CONFIGURATION_A */
#define	MESURE_NORMALE		0x00 
#define	MESURE_BIAS_PLUS	0x01 
#define	MESURE_BIAS_MOINS	0x02

#define	OUTPUT_RATE_15HZ				(0x04 << 2)
#define	OUTPUT_RATE_30HZ				(0x05 << 2)
#define	OUTPUT_RATE_75HZ				(0x06 << 2)

#define	SAMPLE_1								(0x00 << 5)
#define	SAMPLE_2								(0x01 << 5)
#define	SAMPLE_4								(0x02 << 5)
#define	SAMPLE_8								(0x03 << 5)
		// à mettre dans registre CONFIGURATION_A
#define	CONFIG_CONFIGURATION_A  (MESURE_NORMALE | OUTPUT_RATE_30HZ | SAMPLE_8 )
// si besoin test mode alors faire CONFIG_CONFIGURATION_A | MODE_MESURE_BIAS_PLUS ou MOINS

		// à mettre dans registre CONFIGURATION_B
	/* Sélection du Gain selon définition ci-dessous */
#define		HMC5883L_GAIN											2		

#if HMC5883L_GAIN == 1					//+- 1.3 Gauss
	#define		COMPAS_NOMINAL_GAIN 			     0.92f 		// mGauss / bit   
	#define		COMPAS_BITS_PAR_GAUSS				    1090	  // pour 1 Gauss  
#elif HMC5883L_GAIN == 2					//+- 1.9 Gauss			
	#define		COMPAS_NOMINAL_GAIN 			     1.22f 		// mGauss / bit   
	#define		COMPAS_BITS_PAR_GAUSS				     820	  // pour 1 Gauss  
#elif HMC5883L_GAIN == 3					//+- 2.5 Gauss
	#define		COMPAS_NOMINAL_GAIN 			     1.52f 		// mGauss / bit   
	#define		COMPAS_BITS_PAR_GAUSS				     660	  // pour 1 Gauss  
#elif HMC5883L_GAIN == 4					//+- 4.0 Gauss
	#define		COMPAS_NOMINAL_GAIN 			     2.27f 		// mGauss / bit   
	#define		COMPAS_BITS_PAR_GAUSS				     440	  // pour 1 Gauss  
#elif HMC5883L_GAIN == 5					//+- 4.7 Gauss
	#define		COMPAS_NOMINAL_GAIN 			     2.56f 		// mGauss / bit   
	#define		COMPAS_BITS_PAR_GAUSS				     390	  // pour 1 Gauss 
#endif

	/* Pour test calibration du Gain */
#define		COMPAS_EXCITATION_BIAS_XY		1160.00f 		// mGauss
#define		COMPAS_EXCITATION_BIAS_Z		1080.00f		// mGauss

	/* valeurs d'usine si sauvegarde HS */
T_HMC5883L_Compensation HMC5883L_Compensation = {  1.0f, 0.0f, 
																									 1.0f, 0.0f, 
																									 1.0f, 0.0f, 
																									+ 90.0f } ;		// pour capteur placé à l'arriere du Robot 

T_Etat_Automate_HMC5883L Etat_Automate_HMC5883L = HMC5883L_MESURE ;
																									
/* Orientation */
float Compas ; 

#ifdef HMC5883L_VALIDATION 	
	/* 360° de scan pour envoi sur le WIFI */
T_BufMessageLong 		BufMessageCompasXY ;
uint16_t 						Idc_BufMessageCompasXY = 0 ; 		

																								
void 	HMC5883L_Charge_Data_Wifi ( float Cp_X, float Cp_Y ) 
{	
	if ( BufMessageCompasXY.Etat_Transport == DISPO )
	{
		BufMessageCompasXY.Data.CompasXY[Idc_BufMessageCompasXY].Compas_X = Cp_X ;
		BufMessageCompasXY.Data.CompasXY[Idc_BufMessageCompasXY].Compas_Y = Cp_Y ;	
		if ( ++ Idc_BufMessageCompasXY >= MAX_COMPAS_XY_DANS_BUFFER )
		{
			BufMessageCompasXY.Total_Octet	= MAX_COMPAS_XY_DANS_BUFFER * sizeof ( T_HMC5883L_CompasXY ) ; 
			BufMessageCompasXY.Type 				= TYPE_MSG_COMPAS_XY ; 
			WifiCom_Transmit_Transport (&BufMessageCompasXY);
			Idc_BufMessageCompasXY = 0 ; 
		}
	}
}				


void HMC5883L_Request (uint8_t	Command ) 
{
	Etat_Automate_HMC5883L = Command ; 
}  
#endif 																									
																									

static HAL_StatusTypeDef  HMC5883L_DataInterrogation ( void ) 
{
	HMC5883L_Communication.Done	= FAUX ;
	HMC5883L_Communication.If_Req_Request = 6 ; 
	HMC5883L_Communication.I2CTx_Buf[0] = DATA_X_MSB ;	// select register to read 6 bytes
	return ( HAL_I2C_Master_Transmit_IT(&HMC5883L_I2c, HMC5883L_ADDRESS, HMC5883L_Communication.I2CTx_Buf, 1) );	
}

static HAL_StatusTypeDef HMC5883L_Envoyer_Config ( uint8_t Registre, uint8_t Data )
{	
	HMC5883L_Communication.Done	= FAUX;
	HMC5883L_Communication.If_Req_Request = 0 ; 
	HMC5883L_Communication.I2CTx_Buf[0] = Registre ;
	HMC5883L_Communication.I2CTx_Buf[1] = Data ;	
	return ( HAL_I2C_Master_Transmit_IT(&HMC5883L_I2c, HMC5883L_ADDRESS, HMC5883L_Communication.I2CTx_Buf, 2) ) ;	
}

/* Fonction à appeler périodiquement avec les seuls arguments suivants :
			HMC5883L_MESURE,							pour mettre à jour Compas 
			HMC5883L_CALIBRATION					pour la procédure de calibration 
																		à maintenir tant que la sortie de fonction 
																		est différente de HMC5883L_END_CALIBRATION */
T_Etat_Automate_HMC5883L HMC5883L_Gestion ( T_Etat_Automate_HMC5883L Request )
{
	static uint32_t delay ;
	static float x_max ;
	static float x_min ;	
	static float y_max ;
	static float y_min ;
#ifdef USE_AXEL_Z		
	static float z_max ;
	static float z_min ;		
	float compass_Z_scalled ;	
#endif
	float compass_X_scalled ;
	float compass_Y_scalled ;
	
	
	if ( HMC5883L_Communication.Done == VRAI )
	{	
		if ( HMC5883L_Communication.Data_Ready  == VRAI )
		{	
			HMC5883L_Communication.Data_Ready = FAUX ;
			switch ( Etat_Automate_HMC5883L )
			{			
				case  HMC5883L_MESURE :
				{
					float orientation  ;
					compass_X_scalled	=	((float)Compas_X	*	COMPAS_NOMINAL_GAIN + HMC5883L_Compensation.Compas_X_Offset) * HMC5883L_Compensation.Compas_Xsf_Gain;
					compass_Y_scalled	=	((float)Compas_Y	*	COMPAS_NOMINAL_GAIN + HMC5883L_Compensation.Compas_Y_Offset) * HMC5883L_Compensation.Compas_Ysf_Gain;
#ifdef USE_AXEL_Z					
					compass_Z_scalled	=	((float)Compas_Z	*	COMPAS_NOMINAL_GAIN + HMC5883L_Compensation.Compas_Z_Offset) * HMC5883L_Compensation.Compas_Zsf_Gain;	
#endif
					orientation = atan2 ( compass_Y_scalled, compass_X_scalled ); 
					if ( orientation < 0 )
						orientation += ( 2.0f * 3.14159f ) ;
					orientation *= 180.0f / 3.14159f ; 
					orientation +=  HMC5883L_Compensation.Compas_Position_Offset ;
					if ( orientation > 360.0f )
						orientation -= 360.0f ;
					Compas = orientation ; 
					Etat_Automate_HMC5883L = Request ;
#ifdef HMC5883L_VALIDATION 					
					Idc_BufMessageCompasXY = 0 ; 
#endif
				}	
				break ; 
									
				case HMC5883L_CALIBRATION :
					Robot_Commande ( 0.0f, -45.0f ) ; 
					x_max = -4000.0f ; 
					y_max = -4000.0f ;
#ifdef USE_AXEL_Z							
					z_max = -4000.0f ; 
					z_min =  4000.0f ; 
#endif
					x_min =  4000.0f ; 	 
					y_min =  4000.0f ; 	
					delay = HAL_GetTick () + 30000 ;				// Le robot tourne sur lui-même pendant 30 secondes 				
					Etat_Automate_HMC5883L = HMC5883L_CALCULE_CALIBRATION; 
				break ;	
				
				case HMC5883L_CALCULE_CALIBRATION :		/* pour calculer Offset et compensation du gain en fonction de l'environnement */
					compass_X_scalled = (float)Compas_X * COMPAS_NOMINAL_GAIN;
					compass_Y_scalled = (float)Compas_Y * COMPAS_NOMINAL_GAIN;
#ifdef USE_AXEL_Z					
					compass_Z_scalled = (float)Compas_Z * COMPAS_NOMINAL_GAIN;  
#endif			
					x_max = fmax(x_max,compass_X_scalled);
					x_min = fmin(x_min,compass_X_scalled);				
					y_max = fmax(y_max,compass_Y_scalled);
					y_min = fmin(y_min,compass_Y_scalled);	
#ifdef USE_AXEL_Z				
					z_max = fmax(z_max,compass_Z_scalled);      
					z_min = fmin(z_min,compass_Z_scalled);
#endif			
					if ( delay <= HAL_GetTick () )
					{
						Robot_Commande ( 0.0, 0.0 ) ; 				// arrêt du robot 
						
						/* Compensation selon environnement pour s'approcher du cercle idéal */
						HMC5883L_Compensation.Compas_Xsf_Gain = (y_max - y_min) / (x_max - x_min);
						HMC5883L_Compensation.Compas_Ysf_Gain = (x_max - x_min) / (y_max - y_min);						
						if ( HMC5883L_Compensation.Compas_Xsf_Gain < 1.0f )
							HMC5883L_Compensation.Compas_Xsf_Gain = 1.0f ; 
						if ( HMC5883L_Compensation.Compas_Ysf_Gain < 1.0f )
							HMC5883L_Compensation.Compas_Ysf_Gain = 1.0f ; 
						
						/* Compensation de l'offset pour recentrer le cercle */ 
						HMC5883L_Compensation.Compas_X_Offset = ((x_max-x_min)/2)-x_max;
						HMC5883L_Compensation.Compas_Y_Offset = ((y_max-y_min)/2)-y_max;
#ifdef USE_AXEL_Z						
						HMC5883L_Compensation.Compas_Z_Offset = ((z_max-z_min)/2)-z_max;	
#endif					
						Etat_Automate_HMC5883L = HMC5883L_END_CALIBRATION ; // Terminé 
					}					
				break ;
				
				case HMC5883L_END_CALIBRATION :
					if ( delay <= HAL_GetTick () )
						Etat_Automate_HMC5883L = HMC5883L_MESURE ;
				break ; 
				
#ifdef HMC5883L_VALIDATION 				
				case HMC5883L_COLLECTE_AVEC_CALIBRATION :
					compass_X_scalled	=	((float)Compas_X	*	COMPAS_NOMINAL_GAIN + HMC5883L_Compensation.Compas_X_Offset) * HMC5883L_Compensation.Compas_Xsf_Gain;
					compass_Y_scalled	=	((float)Compas_Y	*	COMPAS_NOMINAL_GAIN + HMC5883L_Compensation.Compas_Y_Offset) * HMC5883L_Compensation.Compas_Ysf_Gain;		
					HMC5883L_Charge_Data_Wifi ( compass_X_scalled, compass_Y_scalled ) ;
				break ;

				case HMC5883L_COLLECTE_SANS_CALIBRATION :	
					compass_X_scalled = (float)Compas_X * COMPAS_NOMINAL_GAIN;
					compass_Y_scalled = (float)Compas_Y * COMPAS_NOMINAL_GAIN;
					HMC5883L_Charge_Data_Wifi ( compass_X_scalled, compass_Y_scalled ) ;						
				break ; 					
#endif									
			}
		}
		else
		{
			if ( HMC5883L_Communication.DRDY_Ready == VRAI )
			{	/* si nouvelles datas, il faut les lire */
				if ( HMC5883L_DataInterrogation () == HAL_OK )
					HMC5883L_Communication.DRDY_Ready = FAUX ; 			
			}
		}
	}
	return ( Etat_Automate_HMC5883L );
}


/* Fonction à appeler impérativement une seule fois au début de l'application */
void HMC5883L_Start ( void ) 
{
	HMC5883L_Communication.Done 			= VRAI ;
	HMC5883L_Communication.DRDY_Ready = FAUX ; 
	HMC5883L_Communication.Data_Ready = FAUX ;	
	HMC5883L_Envoyer_Config ( CONFIGURATION_A, CONFIG_CONFIGURATION_A ) ; 	
	while ( HMC5883L_Communication.Done == FAUX ) {} ; 
	HMC5883L_Envoyer_Config ( CONFIGURATION_B, HMC5883L_GAIN << 5) ;  
	while ( HMC5883L_Communication.Done == FAUX ) {} ; 
	HMC5883L_Envoyer_Config ( MODE, 0x00 ) ;   								// Continuous measurement mode 		
	while ( HMC5883L_Communication.Done == FAUX ) {} ; 	
}


/* Interruption émission I2C 
	 fonction appelée par call back HAL_I2C_MasterTxCpltCallback dans fichier stm32f4xx_it.c	*/		
void HMC5883L_Emission_IT ( void )
{
	if ( HMC5883L_Communication.If_Req_Request != 0 )
	{ // si réception demandée 
		if ( HAL_I2C_Master_Receive_IT(&HMC5883L_I2c, HMC5883L_ADDRESS, HMC5883L_Communication.I2CRx_Buf, HMC5883L_Communication.If_Req_Request) != HAL_OK )
			HMC5883L_Communication.Done = VRAI ;	// pas possible de recevoir I2C occupé ? !!
	}
	else
		HMC5883L_Communication.Done = VRAI ; 
}


/* Interruption réception I2C 
	 fonction appelée par call back HAL_I2C_MasterRxCpltCallback dans fichier stm32f4xx_it.c	*/	
void HMC5883L_Reception_IT ( void )
{
	Compas_X = HMC5883L_Communication.I2CRx_Buf[0] << 8 | HMC5883L_Communication.I2CRx_Buf[1] ;  
//#ifdef USE_AXEL_Z		
	Compas_Z = HMC5883L_Communication.I2CRx_Buf[2] << 8 | HMC5883L_Communication.I2CRx_Buf[3] ;	
//#endif	
	Compas_Y = HMC5883L_Communication.I2CRx_Buf[4] << 8 | HMC5883L_Communication.I2CRx_Buf[5] ;	
	HMC5883L_Communication.Done 			= VRAI ; 
	HMC5883L_Communication.Data_Ready = VRAI ;
}


/* Interruption sur data ready */
void	HMC5883L_Event_IT (void)
{
	HMC5883L_Communication.DRDY_Ready = VRAI ;
}

