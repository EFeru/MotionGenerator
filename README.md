# MotionGenerator
This is a library which generates a smooth motion profile while considering a maximum velocity and acceleration. The profile is based on the trapezoidal speed motion.

Matlab and cpp (tested in Arduino) libraries are available. Conversion to other platforms should be straight-forward.

Note: The reason why I wrote my own library is because the existing ones are NOT analytical-based solution. Therefore, they depend on the used sampling rate, which can lead to oscillations and even instability. The library presented here, works no matter the sampling rate. The library was tested on a Robotic Arm that I build from servos https://www.youtube.com/watch?v=B4DPD9VfmHA

# Features
* On the fly profile generation
* Analytical-based solution
* Independent of the sampling period used
* Smooth position profile based on the trapezoidal motion

# Usage
```cpp
// Includes
#include "MotionGenerator.h"
/*
 * Initialization
 *
 * @param int aVelMax maximum velocity (units/s)
 * @param int aAccMax maximum acceleration (units/s^2)
 * @param int aInitPos initial position (units)
 */

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
 ``` 
## Example plot
### Trapezoidal motion generator
![Simulation of a trapezoidal motion profile](https://github.com/AerDronix/MotionGenerator/blob/master/matlab/simPhoto.png)
Limited velocity at 200 units/s and acceleration at 500 units/s^2.
