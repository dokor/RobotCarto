

#ifndef H_TYPEDEF 
#define	H_TYPEDEF

typedef enum {
			OUI			= 0,  
			NON			= 1, 
			ERREUR	= 0x80 
} E_OuiNonErreur ;
	
#define	T_OuiNonErreur	uint8_t		// OUI ou NON ou  ERREUR

typedef enum {
			ARRET		= 0,
			MARCHE	= 1 
} E_MarcheArret ; 

#define	T_MarcheArret		uint8_t		// ARRET ou MARCHE 	


typedef enum {
			FAUX		= 0,
			VRAI		= 1 
} E_Boolean  ; 

#define	T_Boolean				uint8_t		// FAUX ou VRAI	

#define			TENSION_ANALOGIQUE			3311							// mv  tension à mesurer selon hard 

//#define		DEBUG_WIFI
//#define			V_ANGULAIRE_AUTO    // a mettre en comm si man
//#define		LCD16X2 
#define 	SERVO
#define 	US_CAPTEUR
//#define 	GYRO
#define 	RPLIDAR
#define 	WIFI
#define		COMPAS
#define		HMC5883L_VALIDATION 
#endif

