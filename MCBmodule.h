/*=========================================================================//
	
	MCBmodule Class
	
	This class handles the control of an individual daughterboard module
	
	Designed for use with Teensy 3.1 and Motor Control Board (Rev 1.2)
	
	
	Trevor Bruns
	
	Changelog-
		2/10/2016: Initial Creation
		3/13/2017: Compiles and appears to run successfully
		
//=========================================================================*/

#ifndef MCBmodule_h
#define MCBmodule_h

#include <stdint.h>
#include "LS7366R.h"
#include "PID_f32.h"

class MCBmodule
{
public:
	MCBmodule(uint8_t csEnc); // needs to know SPI chip-select pin for it's encoder
	~MCBmodule(void);
	
	bool init(float kp, float ki, float kd); // initializes encoder/PID controller and enables module
	bool init(void); // ^^ except initializes with PID gains all set to 0.0
    bool isConfigured(void) { return configured_; } // returns true after proper initializationa

	uint16_t effortToDacCommand(float effort); // converts a motor effort [volts] to a DAC command [0,2^16]   
    float getEffort() { return effort_; }

private:
    bool configured_ = false;

    // Motor
    bool motorPolarity_ = 1; // used to make sure positive current -> positive encoder counts

	// DAC
    float dacRange_[2] = { -10.0, 10.0 }; // [volts] DAC output range; PID effort will be scaled/saturated based on these values

	// Encoder
	LS7366R enc_; // Quadrature encoder interface
	
	float effort_ = 0; // unsaturated, computed effort from controller in Amps
};

#endif
