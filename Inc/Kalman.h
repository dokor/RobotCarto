/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef _Kalman_h_
#define _Kalman_h_

extern void Kalman_Initialise ( void ); 

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
extern float Kalman_Get_Angle (float newAngle, float newRate, float dt);

extern	void Kalman_Set_Angle (float angle); // Used to set angle, this should be set as the starting angle
extern float Kalman_Get_Rate ( void ) ; 		// Return the unbiased rate

extern void Kalman_Set_QAngle (float QAngle ) ; 


extern void Kalman_Set_QBias (float QBias ) ;
	

extern void Kalman_Set_Rmeasure (float Rmeasure );


extern float Kalman_Get_QAngle ( void ) ;


extern float Kalman_Get_QBias ( void ) ;


extern float Kalman_Get_Rmeasure ( void ) ; 





#endif
