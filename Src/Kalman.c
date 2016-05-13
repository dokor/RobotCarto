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

#include "Kalman.h"
    /* Kalman filter variables */
    float Q_Angle; // Process noise variance for the accelerometer
    float Q_Bias; // Process noise variance for the gyro bias
    float R_Measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float Angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float Bias;  // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float Rate;  // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
		
void Kalman_Initialise ( void ) 
{
    /* We will set the variables like so, these can also be tuned by the user */
    Q_Angle 	= 0.001f;
    Q_Bias 		= 0.003f;
    R_Measure = 0.03f;

    Angle 		= 0.0f; // Reset the angle
    Bias 			= 0.0f; // Reset bias

    P[0][0] 	= 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] 	= 0.0f;
    P[1][0] 	= 0.0f;
    P[1][1] 	= 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman_Get_Angle (float NewAngle, float NewRate, float dt) 
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    Rate = NewRate - Bias;
    Angle += dt * Rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_Angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_Bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_Measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = NewAngle - Angle; // Angle difference
    /* Step 6 */
    Angle += K[0] * y;
    Bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return Angle;
}

void Kalman_Set_Angle (float Starting_Angle ) 
{ 
	Angle = Starting_Angle; 
} // Used to set angle, this should be set as the starting angle

float Kalman_Get_Rate ( void ) 
{ 
	return Rate; 
} // Return the unbiased rate


/* These are used to tune the Kalman filter */
void Kalman_Set_QAngle (float QAngle ) 
{
	Q_Angle = QAngle  ; 
}	

void Kalman_Set_QBias (float QBias ) 
{
	Q_Bias = QBias  ; 
}	

void Kalman_Set_Rmeasure (float Rmeasure ) 
{
	R_Measure = Rmeasure  ; 
}	

float Kalman_Get_QAngle ( void ) 
{
	return ( Q_Angle )    ; 
}	

float Kalman_Get_QBias ( void ) 
{
	return ( Q_Bias ) ;
}	

float Kalman_Get_Rmeasure ( void ) 
{
	return ( R_Measure ); 
}	


