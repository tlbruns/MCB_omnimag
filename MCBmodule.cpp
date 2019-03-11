/*=========================================================================//

	Motor Class
	
	This class handles the control of an individual motor
	
	Designed for use with Teensy 3.1 and Motor Control Board (Rev 1.1+)
	
	
	Trevor Bruns		
		
//=========================================================================*/

#include "MCBmodule.h"
#include <stdint.h>
#include <SPI.h>
#include "LS7366R.h"
#include "PID_f32.h"
#include <math.h>

MCBmodule::MCBmodule(uint8_t csEnc)
	: enc_(csEnc) // create encoder interface
{
}

bool MCBmodule::init(float kp, float ki, float kd)
{
    // set up encoder IC (LS7366R)
    int maxAttempts = 5;
    int attempts = 0;
    while (attempts < maxAttempts) {
        attempts++;
        if (enc_.init()) {
            configured_ = true;
        }
    }
    return configured_;
}

bool MCBmodule::init(void)
{
    // set up encoder IC (LS7366R)
    int maxAttempts = 5;
    int attempts = 0;
    while (attempts < maxAttempts) {
        attempts++;
        if (enc_.init()) {
            configured_ = true;
            break;
        }
    }
    return configured_;
}

uint16_t MCBmodule::effortToDacCommand(float effort)
{
	float effortTemp = effort;
    
	// check for saturation
	if (effort > dacRange_[1]) { 
        effortTemp = dacRange_[1]; 
    }
	else if (effort < dacRange_[0]) { 
        effortTemp = dacRange_[0]; 
    }

    // encode effort to 16-bit DAC code
    // DAC code = (2^16)*(effort - Vmin)/(Vmax - Vmin)
    return static_cast<uint16_t>( 65535.0f * (effortTemp - dacRange_[0]) / (dacRange_[1] - dacRange_[0]) );
}

MCBmodule::~MCBmodule(void)
{
}
