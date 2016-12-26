#ifndef __MOTIONGENERATOR_H__
#define __MOTIONGENERATOR_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/**
 * Generates the analytical solution for the trapezoidal motion.
 *
 * <p>
 * Usage:
 * // Includes
 * #include "MotionGenerator.h"
 *
 * Initialization
 *
 * @param int aVelMax maximum velocity (units/s)
 * @param int aAccMax maximum acceleration (units/s^2)
 * @param int aInitPos initial position (units)
 *

 // Define the MotionGenerator object
 MotionGenerator *trapezoidalProfile = new MotionGenerator(100, 400, 0);

 // Retrieve calculated position
 float positionRef = 1000;
 float position = trapezoidalProfile->update(positionRef)

 // Retrieve current velocity
 float velocity = trapezoidalProfile->getVelocity();

 // Retrieve current acceleration
 float acceleration = trapezoidalProfile->getAcceleration();

 // Check if profile is finished
 if (trapezoidalProfile->getFinished()) {};

 // Reset internal state
 trapezoidalProfile->reset();

 *
 * @author      AerDronix <aerdronix@gmail.com>
 * @web		https://aerdronix.wordpress.com/
 * @version     1.0 
 * @since       2016-12-22
 */

class MotionGenerator {		
	public:	
		/**	
		 * Constructor
		 * 
		 * @param int aVelocityMax maximum velocity
		 * @param int aAccelerationMax maximum acceleration
		 */
		MotionGenerator(float aMaxVel, float aMaxAcc, float aInitPos);
			
		void init();
		
		/**	
		 * Updates the state, generating new setpoints
		 *
		 * @param aSetpoint The current setpoint.
		 */
		float update(float aPosRef);
		float getVelocity();
		float getAcceleration();
		
		bool getFinished();				
		void setMaxVelocity(float aMaxVel);
		void setMaxAcceleration(float aMaxAcc);
		void setInitPosition(float aInitPos);				
		void reset();		
		
	private:
		/** 	
		 * Increments the state number.
		 * 
		 * @see
		  currentState
		 */						
		void calculateTrapezoidalProfile(float);	
		short int sign(float aVal);		
		
		float maxVel;
		float maxAcc;		
		float initPos;
        	float pos;
        	float vel;
        	float acc;
        	float oldPos;
        	float oldPosRef;
        	float oldVel;
        
        	float dBrk;
        	float dAcc;
        	float dVel;
        	float dDec;
        	float dTot;
        
        	float tBrk;
        	float tAcc;
        	float tVel;
        	float tDec;
		
		float velSt;
        
        	unsigned long oldTime;
        	unsigned long lastTime;
        	unsigned long deltaTime;
        
        	short int signM;      	// 1 = positive change, -1 = negative change
        	bool shape;      	// true = trapezoidal, false = triangular
		
		bool isFinished;	
};
#endif
