#include "MCB_FSM.h"
#include "MCB.h"
#include "MCBmodule.h"
#include "McbRosConfiguration.h"
#include <IntervalTimer.h>
#include <ros.h>
#include <Ethernet.h>
#include "WiznetHardware.h"
#include <medlab_motor_control_board/EnableMotor.h>
#include <medlab_motor_control_board/McbStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>
#include <EEPROM.h>

/****************** GLOBALS *********************/

// Motor Control Board
MCB MotorBoard; // construct motor control board
int8_t currentMotorSelected = 0; // for manual control using Up/Down buttons
IntervalTimer timerMotorSelectLed;
McbRosConfiguration rosConfig;
MCBstate stateCurrent;
enum modeSwitchState { Manual, Ros };
volatile modeSwitchState modeState;
bool ROSenable = false; // ROS must set this true via 'enable_ros_control' topic to control motors

// Manual Control
//IntervalTimer timerManualControl; // Button read timer interrupt
volatile bool timerManualControlFlag = false; // indicates timerManualControl has been called
float frequencyManualControl = 500.0; // [Hz]
uint32_t timeStepManualControl = static_cast<uint32_t>(1000000.0 / frequencyManualControl); // [us]
uint32_t countStepManualControl = 50; // [counts] step size for each up/down button press

// ROS
ros::NodeHandle_<WiznetHardware> nh;
String rosNameEncoderCurrent;
String rosNameEffortCommand;
String rosNameLimitSwitchEvent;
String rosNameStatus;
String rosNameGetStatus;
String rosNameEncoderZeroSingle;
String rosNameEncoderZeroAll;
String rosNameResetDacs;
String rosNameEnableMotor;
String rosNameEnableAllMotors;
String rosNameSetGains;
String rosNameEnableRosControl;
    
medlab_motor_control_board::McbStatus msgStatus; // stores MCB status message
medlab_motor_control_board::EnableMotor msgLimitSwitchEvent; // stores message that is sent whenever a limit switch is triggered
ros::Publisher pubStatus("tmp", &msgStatus); // publishes MCB status
ros::Publisher pubLimitSwitchEvent("tmp", &msgLimitSwitchEvent); // publishes each time a limit switch is triggered
ros::Subscriber<medlab_motor_control_board::EnableMotor> subEnableMotor("tmp", &subEnableMotorCallback); // enables or disables power to a specific motor
ros::Subscriber<std_msgs::Bool>                          subEnableAllMotors("tmp", &subEnableAllMotorsCallback); // enables or disables all motors
ros::Subscriber<geometry_msgs::Vector3>                  subEffortCommand("tmp", &subEffortCommandCallback); // receives motor commands
ros::Subscriber<std_msgs::Empty>                         subResetDacs("tmp", &subResetDacsCallback); // re-initializes DACs
ros::Subscriber<std_msgs::Bool>                          subEnableRosControl("tmp", &subEnableRosControlCallback); // used to move between RosIdle and RosControl states
ros::Subscriber<std_msgs::Empty>                         subGetStatus("tmp", &subGetStatusCallback); // tells MCB to publish pubStatus

IntervalTimer timerRos; // ROS timer interrupt
volatile bool timerRosFlag = false; // indicates timerRos has been called
float frequencyRos = 500.0; // [Hz]
uint32_t timeStepRos = uint32_t(1000000.0 / frequencyRos); // [us]

MCBstate stepStateMachine(MCBstate stateNext) 
{
	switch (stateNext)
	{
	case statePowerUp:
		return PowerUP();

	case stateManualIdle:
		return ManualIdle();

	case stateManualControl:
		return ManualControl();

    case stateRosInit:
		return RosInit();

	case stateRosIdle:
		return RosIdle();

	case stateRosControl:
		return RosControl();

	default:
		Serial.println("Error: Unrecognized State");
        while (1) {
            // blink at 0.5 Hz
            MotorBoard.toggleLEDG();
            delay(1000);

            //MotorBoard.setLEDG(HIGH);
            //delay(500);
            //MotorBoard.setLEDG(LOW);
            //delay(500);
        }
	}
}

MCBstate PowerUP(void)
{
	stateCurrent = statePowerUp;

	// start serial port
	Serial.begin(115200);
    Serial.setTimeout(100);

	// initialize motor control board
    Serial.println("\n********************************");
    Serial.println("Initializing Motor Control Board");
    Serial.print("Firmware Version ");
    Serial.println(MCB_VERSION);
    Serial.println("********************************\n");
	
    // initialization error
    if (MotorBoard.init() == -1) 
    {
        MCB::ErrorCode errorCode = MotorBoard.getErrorCode();
        blinkErrorCode(errorCode); // (1 Hz = WRONG_MODULE_ORDER; 2 Hz = ESTOP_TRIGGERED; 4 Hz = LIMIT_SWITCH_TRIGGERED_ON_STARTUP)
        printErrorCode(errorCode);



        // halt unless user override via serial commands or holding menu button for 2 seconds
        bool halt = true;
        bool youAreReckless = false;
        Serial.println("\nEnter 'y' to ignore and force initialization");

        while (halt) {
            // check for serial commands
            if (Serial.available() > 0) {
                char cmd = Serial.read();

                if (cmd == 'y') {
                    if (youAreReckless) {
                        halt = false; // ignore errors and continue
                    }
                    else {
                        youAreReckless = true;
                        Serial.println("\n!!! THIS MAY CAUSE UNDESIRED MOTOR MOVEMENT !!!");
                        Serial.println("\nAre you absolutely sure you know what you are doing?!?!");
                    }
                }
            }

            // check if menu button is held
            static ulong goalTime = 0;
            MotorBoard.readButtons();
            if (MotorBoard.isMenuPressed()) {
                if (goalTime == 0) {
                    goalTime = millis() + 2000; // initialize
                }
                else if (millis() > goalTime) {
                    halt = false; // ignore errors and continue
                }
            }
            else {
                goalTime = 0; // reset
            }

            delay(10); // no reason to loop crazy fast
        }

        blinkErrorCode(MCB::ErrorCode::NO_ERROR); // stop blinking

        Serial.println("\nForcing initialization...\n");
        MotorBoard.init(youAreReckless);
    }

    Serial.print(MotorBoard.numModules());
    Serial.println(" motor modules detected and configured");

    delay(10);

	// create pin change interrupt for mode switch
	attachInterrupt(MotorBoard.pins.modeSelect, modeSwitchCallback, CHANGE);
	modeSwitchCallback(); // run once to initialize modeState

    // create pin change interrupt for amplifier control/limit switch monitoring (active low)
    attachInterrupt(MotorBoard.pins.i2cInt, ampEnableISR, FALLING);
    
	// advance based on mode switch position 
	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosInit;

    default: // shouldn't ever reach here
        return stateManualIdle;
	}
}

MCBstate RosInit(void)
{
	stateCurrent = stateRosInit;

    Serial.println("\n\n******************");
    Serial.println("ROS Initialization");
    Serial.println("******************");

    ROSenable = false;

    // read ROS configuration parameters from EEPROM
    rosConfig.getSettingsFromEeprom();

    // set to defaults if none found
    if (!rosConfig.wasSaved()) {
        rosConfig.setDefaults();
    }

    // update ROS node handle with the new config parameters
    IPAddress mcbIp(192, 168, 0, rosConfig.getIP());
    nh.getHardware()->setIP(mcbIp);
    uint8_t mcbMac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, rosConfig.getMac() };
    nh.getHardware()->setMAC(mcbMac);

    // create topic names
    String rosNamespace = rosConfig.getNamespace();
    rosNameEffortCommand     = rosNamespace + "/dac_voltages";
    rosNameLimitSwitchEvent  = rosNamespace + "/limit_switch_event";
    rosNameStatus            = rosNamespace + "/status";
    rosNameGetStatus         = rosNamespace + "/get_status";
    rosNameResetDacs         = rosNamespace + "/reset_dacs";
    rosNameEnableMotor       = rosNamespace + "/enable_amp";
    rosNameEnableAllMotors   = rosNamespace + "/enable_all_amps";
    rosNameEnableRosControl  = rosNamespace + "/enable_ros_control";

    // setup topics
    subEffortCommand      = ros::Subscriber<geometry_msgs::Vector3>(rosNameEffortCommand.c_str(), &subEffortCommandCallback);
    pubLimitSwitchEvent   = ros::Publisher(rosNameLimitSwitchEvent.c_str(), &msgLimitSwitchEvent);
    pubStatus             = ros::Publisher(rosNameStatus.c_str(), &msgStatus);
    subGetStatus          = ros::Subscriber<std_msgs::Empty>(rosNameGetStatus.c_str(), &subGetStatusCallback);
    subResetDacs          = ros::Subscriber<std_msgs::Empty>(rosNameResetDacs.c_str(), &subResetDacsCallback);
    subEnableMotor        = ros::Subscriber<medlab_motor_control_board::EnableMotor>(rosNameEnableMotor.c_str(), &subEnableMotorCallback);
    subEnableAllMotors    = ros::Subscriber<std_msgs::Bool>(rosNameEnableAllMotors.c_str(), &subEnableAllMotorsCallback);
    subEnableRosControl   = ros::Subscriber<std_msgs::Bool>(rosNameEnableRosControl.c_str(), &subEnableRosControlCallback);

	// set up Wiznet and connect to ROS server
	Serial.print("Configuring Ethernet Connection ... ");
	nh.initNode();

	// repeatedly attempt to setup the hardware, loop on fail, stop on success
	while ((nh.getHardware()->error() < 0) && (modeState == Ros)) {
		Serial.print("WIZnet error = ");
		Serial.println(nh.getHardware()->error());

		nh.initNode();
	}

	if (modeState == Manual) {
		return stateManualIdle;
	}

	Serial.println("Success!");

	// initialize ROS
	Serial.print("Connecting to ROS Network ... ");
    nh.advertise(pubStatus);
    nh.advertise(pubLimitSwitchEvent);
	nh.subscribe(subEffortCommand);
    nh.subscribe(subResetDacs);
    nh.subscribe(subEnableMotor);
    nh.subscribe(subEnableAllMotors);
    nh.subscribe(subEnableRosControl);
    nh.subscribe(subGetStatus);
	

    // wait until connection established or mode switched to 'Manual'
	while (!nh.connected() && (modeState == Ros)) {
        noInterrupts();
		nh.spinOnce();
        interrupts();
	}

	if (modeState == Manual) {
		return stateManualIdle;
	}

	Serial.println("Success!");

	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosIdle;

    default: // shouldn't ever reach here
        return stateRosInit;
	}
}

MCBstate RosIdle(void)
{
	stateCurrent = stateRosIdle;

    Serial.println("\n\n*************************");
	Serial.println("\nEntering ROS Idle state");
    Serial.println("*************************");
    Serial.println("\nwaiting for enable signal via enable_controller");

    // ensure amps are off
    MotorBoard.disableAllAmps();
    MotorBoard.setGlobalInhibit(true);
    MotorBoard.updateAmpStates();

    // start ROS update timer
    timerRos.begin([]() {timerRosFlag = true; }, timeStepRos);

	// wait for ROS enable command via service call
	while (!ROSenable && nh.connected() && (modeState == Ros)) {
        noInterrupts(); // prevent interrupts during SPI communication

        // process pending ROS communications
        nh.spinOnce();

        timerRosFlag = false;

        interrupts();
	}

	if (!nh.connected()) {
		return stateRosInit;
	}

	switch (modeState) {
	case Manual:
		return stateManualIdle;

	case Ros:
		return stateRosControl;

    default: // shouldn't ever reach here
        return stateRosInit;
	}
}

MCBstate RosControl(void)
{
	stateCurrent = stateRosControl;

    Serial.println("\n\n**************************");
    Serial.println("Entering ROS Control state");
    Serial.println("**************************");

    // ensure that e-stop is not engaged
    if (MotorBoard.initLimitSwitchStates() == MCB::ErrorCode::ESTOP_TRIGGERED) {
        Serial.println("\nE-Stop engaged! \nMust disengage before entering control state.");
        Serial.println("\nReturning to idle state...");

        // publish limit switch event
        msgLimitSwitchEvent.motor = 6; // 6 => E-STOP
        msgLimitSwitchEvent.enable = MotorBoard.eStopState();
        pubLimitSwitchEvent.publish(&msgLimitSwitchEvent);

        // return to RosIdle state
        ROSenable = false;
        return stateRosIdle;
    }

	// reset DACs to ensure currents are set to zero
	for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
		MotorBoard.initDACs();
	}
    geometry_msgs::Vector3 msg_temp;
    msg_temp.x = 0.0f;
    msg_temp.y = 0.0f;
    msg_temp.z = 0.0f;
    subEffortCommandCallback(msg_temp);

	// start ROS update timer
    timerRos.begin([]() {timerRosFlag = true; }, timeStepRos);

    // disable globalInhibit
    MotorBoard.setGlobalInhibit(false);
    MotorBoard.updateAmpStates();


	// loop until disconnected OR ROS 'disable' command OR mode switched to 'Manual'
	while (ROSenable && nh.connected() && (modeState == Ros)) 
    { 
    // NOTE: this while loop must be able to run at least twice as fast as fastest InterruptTimer

        // process any triggered limit switches
        if (MotorBoard.ampEnableFlag())
        {
            // update current states of limit switches and ampEnable pins, and determine what was triggered
            if (MotorBoard.updateAmpStates())
            {
                for (uint8_t ii = 0; ii < MotorBoard.triggeredLimitSwitches().size(); ii++)
                {
                    if (MotorBoard.triggeredLimitSwitches().at(ii) == MCB::LimitSwitch::ESTOP)
                    {   // e-stop was triggered

                        // publish limit switch event
                        msgLimitSwitchEvent.motor = 6; // 6 => E-STOP
                        msgLimitSwitchEvent.enable = MotorBoard.eStopState();
                        pubLimitSwitchEvent.publish(&msgLimitSwitchEvent);

                        // exit ROS control state
                        ROSenable = false;
                    }
                    else
                    {   // limit switch was triggered
                        uint8_t modulePosition = MotorBoard.limitSwitchToPosition(MotorBoard.triggeredLimitSwitches().at(ii)); // convert MCB::LimitSwitch to uint8_t
                        if (modulePosition <= MotorBoard.numModules()) {
                            // ensure this motor is disabled
                            MotorBoard.disableAmp(modulePosition);

                            // publish limit switch event
                            msgLimitSwitchEvent.motor = modulePosition;
                            msgLimitSwitchEvent.enable = MotorBoard.limitSwitchState(modulePosition);
                            pubLimitSwitchEvent.publish(&msgLimitSwitchEvent);
                        }
                    }
                }

                // reset now that we have processed
                MotorBoard.resetTriggeredLimitSwitches();
            }
        }

        noInterrupts(); // prevent interrupts during functions using SPI      

        // process pending ROS communications
        nh.spinOnce();

        timerRosFlag = false;

        interrupts(); // process any interrupts here
	}

	// power off motors, disable PID controller, and stop ROS timer
	MotorBoard.disableAllAmps();
	timerRos.end();
	ROSenable = false;
    interrupts(); // now safe to re-enable since timer interrupts are stopped

	if (!nh.connected()) {
		nh.getHardware()->error() = WiznetHardware::ERROR_CONNECT_FAIL;
		Serial.println("ROS connection lost");
		return stateRosInit;
	}

	if (modeState == Manual) {
		return stateManualIdle;
	}
	else { // 'stop' command must have been received
		return stateRosIdle; 
	}
}

MCBstate ManualIdle(void)
{
    stateCurrent = stateManualIdle;

    Serial.println("\n\n**************************");
    Serial.println("Entering Manual Idle State");
    Serial.println("**************************");

    // ensure amps are off and controller is not running
    MotorBoard.disableAllAmps();
    MotorBoard.setGlobalInhibit(true);

    uint32_t holdTime = 2000; // [ms] how long buttons must be held before function returns
    uint32_t timeButtonsPressed = 0; // [ms] how long buttons have been held
    uint32_t timeStart = 0;
    MotorBoard.setLEDG(false); // turn off green LEDs
    bool configFinished = false;

    // wait until gains have been set via serial OR user overrides by holding buttons
    Serial.println("\nROS Configuration");
    rosConfig.printMenu();
    rosConfig.printWaitCommand();
    while ((modeState == Manual) && !configFinished) {
        // check for serial commands
        if (!rosConfig.runOnce()) {
            // user has selected an exit command
            configFinished = true;
        }

        // check buttons
        if (timeButtonsPressed < holdTime)
        {
            MotorBoard.readButtons();
            if (MotorBoard.isEverythingPressed()) {
                // if just pressed
                if (timeButtonsPressed == 0)
                {
                    timeStart = millis();
                    delayMicroseconds(1000); // ensure next millis() call will be different
                }

                // light LEDs in sequence for user feedback
                if (timeButtonsPressed < (holdTime / 7)) {
                    MotorBoard.setLEDG(0, HIGH);
                }
                else if (timeButtonsPressed < (2 * holdTime / 7)) {
                    MotorBoard.setLEDG(1, HIGH);
                }
                else if (timeButtonsPressed < (3 * holdTime / 7)) {
                    MotorBoard.setLEDG(2, HIGH);
                }
                else if (timeButtonsPressed < (4 * holdTime / 7)) {
                    MotorBoard.setLEDG(3, HIGH);
                }
                else if (timeButtonsPressed < (5 * holdTime / 7)) {
                    MotorBoard.setLEDG(4, HIGH);
                }
                else if (timeButtonsPressed < (6 * holdTime / 7)) {
                    MotorBoard.setLEDG(5, HIGH);
                }
                else {
                    MotorBoard.toggleLEDG();
                    delayMicroseconds(50000);

                    //MotorBoard.setLEDG(LOW);
                    //delayMicroseconds(50000);
                    //MotorBoard.setLEDG(HIGH);
                    //delayMicroseconds(50000);
                }

                timeButtonsPressed = millis() - timeStart;
            }
            else {
                timeButtonsPressed = 0;
                MotorBoard.setLEDG(LOW);
            }
        }
        else {
            // user override -> use default gains and current limits
            configFinished = true;
            delay(1000);
        }
    }

    // advance based on mode switch position
    switch (modeState) {
    case Manual:
        return stateManualControl;

    case Ros:
        return stateRosInit;

    default: // shouldn't ever reach here
        return stateManualIdle;
    }
}

MCBstate ManualControl(void)
{
    stateCurrent = stateManualControl;

    Serial.println("\n\n*****************************");
    Serial.println("Entering Manual Control State");
    Serial.println("*****************************\n");

    //// set desired motor position to current position (prevents unexpected movement)
    //for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
    //    MotorBoard.setCountDesired(ii, MotorBoard.readCountCurrent(ii));
    //}

    //// start manual control timer
    //timerManualControl.begin([]() {timerManualControlFlag = true; }, timeStepManualControl);

    //// flash led of currently selected motor
    ////timerMotorSelectLed.begin(motorSelectLedCallback, 350000);
    //timerMotorSelectLed.begin([]() {MotorBoard.toggleLEDG(currentMotorSelected); }, 350000);

    //// ensure that e-stop is not engaged
    //if (MotorBoard.initLimitSwitchStates() == MCB::ErrorCode::ESTOP_TRIGGERED) {
    //    Serial.println("\nE-Stop engaged! Must disengage before entering control state.");
    //    Serial.println("\nReturning to idle state...");
    //    return stateManualIdle;
    //}

    //// disable globalInhibit for amps
    //MotorBoard.setGlobalInhibit(false);
    //MotorBoard.updateAmpStates();

    //// power on first motor
    //MotorBoard.enableAmp(currentMotorSelected);

    //MotorBoard.setLEDG(LOW);

    // keep running until mode switch changed to Ros OR serial command detected
    while (modeState == Manual) // NOTE: this while loop must be able to run at least twice as fast as the fastest InterruptTimer (usually timerPid)
    {  
        //// process any triggered limit switches
        //if (MotorBoard.ampEnableFlag()) 
        //{
        //    // update current states of limit switches and ampEnable pins, and determine what was triggered
        //    if (MotorBoard.updateAmpStates()) 
        //    {
        //        for (uint8_t ii = 0; ii < MotorBoard.triggeredLimitSwitches().size(); ii++) 
        //        {
        //            if (MotorBoard.triggeredLimitSwitches().at(ii) == MCB::LimitSwitch::ESTOP) 
        //            {   // e-stop was triggered
        //                
        //                Serial.println("\nE-Stop Engaged! \nExiting Manual Control State");

        //                // stop timers and disable amps
        //                timerManualControl.end();
        //                timerMotorSelectLed.end();
        //                MotorBoard.disableAllAmps();

        //                // leave control state
        //                return stateManualIdle; 
        //            }
        //            else 
        //            {   // limit switch was triggered
        //                uint8_t modulePosition = MotorBoard.limitSwitchToPosition(MotorBoard.triggeredLimitSwitches().at(ii)); // convert MCB::LimitSwitch to uint8_t
        //                if (modulePosition <= MotorBoard.numModules()) {
        //                    Serial.print("limit switch ");
        //                    Serial.print(modulePosition);
        //                    Serial.print(" triggered (switch is currently ");
        //                    if (MotorBoard.limitSwitchState(modulePosition)) {
        //                        Serial.println("closed)");
        //                    }
        //                    else {
        //                        Serial.println("open)");
        //                    }

        //                    // disable this motor
        //                    MotorBoard.disableAmp(modulePosition);
        //                }
        //            }
        //        }

        //        // reset now that we have processed
        //        MotorBoard.resetTriggeredLimitSwitches();
        //    }
        //}
        //
        //noInterrupts(); // prevents interruption during critical functions

        //// run manual control on a timer so a held button produces a constant velocity
        //if (timerManualControlFlag) {
        //    runManualControl();
        //    timerManualControlFlag = false;
        //}

        //interrupts(); // now safe to process any interrupts
    }

    // stop checking buttons
    //timerManualControl.end();
    timerMotorSelectLed.end();

    // power off motors and disable PID controller
    MotorBoard.disableAllAmps();

    // turn off green LEDs
    MotorBoard.setLEDG(LOW);

    return stateRosInit; // while() only exits if mode switched to 'Ros'
}

//--------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------//

void ampEnableISR(void)
{
    MotorBoard.setAmpEnableFlag();
}

void printErrorCode(MCB::ErrorCode errorCode)
{
    switch (errorCode)
    {
    case MCB::ErrorCode::NO_ERROR:
        Serial.println("\nThere is currently no error :)");
        break;

    case MCB::ErrorCode::ESTOP_TRIGGERED:
        Serial.println("\nERROR: E-STOP IS CURRENTLY TRIGGERED");
        Serial.println("\nPower off, reset e-stop, then power back on");
        break;

    case MCB::ErrorCode::LIMIT_SWITCH_TRIGGERED_ON_STARTUP:
        Serial.println("\nWARNING: A LIMIT SWITCH IS CURRENTLY TRIGGERED");
        break;

    case MCB::ErrorCode::WRONG_MODULE_ORDER:
        Serial.println("\nERROR: INCORRECT MODULE CONFIGURATION");
        Serial.println("\nPower off and ensure there are no gaps between daughterboard modules");
        break;

    default:
        Serial.println("\nSomething might be wrong, but I'm not sure...");
        break;
    }
}

void blinkErrorCode(MCB::ErrorCode errorCode)
{
    // 1 Hz = WRONG_MODULE_ORDER
    // 2 Hz = ESTOP_TRIGGERED
    // 4 Hz = LIMIT_SWITCH_TRIGGERED_ON_STARTUP

    switch (errorCode) {
    case MCB::ErrorCode::WRONG_MODULE_ORDER:
        timerMotorSelectLed.begin([]() {MotorBoard.toggleLEDG();}, 500000); // 1 Hz
        break;
    case MCB::ErrorCode::ESTOP_TRIGGERED:
        timerMotorSelectLed.begin([]() {MotorBoard.toggleLEDG();}, 250000); // 2 Hz
        break;
    case MCB::ErrorCode::LIMIT_SWITCH_TRIGGERED_ON_STARTUP:
        timerMotorSelectLed.begin([]() {MotorBoard.toggleLEDG();}, 125000); // 4 Hz
        break;
    default:
        timerMotorSelectLed.end(); // stop blinking
        break;
    }
}


void modeSwitchCallback(void)
{
    // HIGH = ROS; LOW = Manual
	modeState = (digitalReadFast(MotorBoard.pins.modeSelect) ? Ros: Manual);
}

void subEnableRosControlCallback(const std_msgs::Bool & msg)
{
    if (msg.data) {
        if (!ROSenable) {
            if (stateCurrent == stateRosIdle) {
                ROSenable = true;
            }
        }
    }
    else { // 'stop' command
        ROSenable = false;
    }
}

void subEnableMotorCallback(const medlab_motor_control_board::EnableMotor & msg)
{
    // MUST be in ROS Control state before enabling (otherwise PID isn't running)
    if (stateCurrent != stateRosControl) {
        return;
    }

    // check that requested motor has been configured
    if (!MotorBoard.isModuleConfigured(msg.motor)) {
        return;
    }

    if (msg.enable) 
    {
        // enable motor
        MotorBoard.enableAmp(msg.motor);
    }
    else
    {
        // disable motor
        MotorBoard.disableAmp(msg.motor);
    }
}

void subEnableAllMotorsCallback(const std_msgs::Bool & msg)
{
    // MUST be in ROS Control state before enabling
    if (stateCurrent != stateRosControl) {
        return;
    }

    if (msg.data)
    {
        // enable all motors
        MotorBoard.enableAllAmps();
    }
    else
    {
        // disable all motors
        MotorBoard.disableAllAmps();
    }
}

void subEffortCommandCallback(const geometry_msgs::Vector3& msg)
{
    // lambda function to convert desired DAC voltages to DAC commands
    auto voltageToDacCommand = [](float effort) -> uint16_t {
        float effortTemp = effort;
        float dacRange_[2] = { -10.0, 10.0 };

        // check for saturation
        if (effort > dacRange_[1]) {
            effortTemp = dacRange_[1];
        }
        else if (effort < dacRange_[0]) {
            effortTemp = dacRange_[0];
        }

        // encode effort to 16-bit DAC code
        // DAC code = (2^16)*(effort - Vmin)/(Vmax - Vmin)
        return static_cast<uint16_t>(65535.0f * (effortTemp - dacRange_[0]) / (dacRange_[1] - dacRange_[0]));
    };

    // MUST be in ROS Control state
    if (stateCurrent != stateRosControl) {
        return;
    }

    // check that three modules have been configured
    if (MotorBoard.numModules() != 3) {
        return;
    }

	// create vector of DAC commands
    Int16Vec dac_cmds;
    dac_cmds.push_back(static_cast<int16_t>(voltageToDacCommand(msg.x)));
    dac_cmds.push_back(static_cast<int16_t>(voltageToDacCommand(msg.y)));
    dac_cmds.push_back(static_cast<int16_t>(voltageToDacCommand(msg.z)));

    // set DACs
    MotorBoard.setDACs(dac_cmds);
}

void subResetDacsCallback(const std_msgs::Empty & msg)
{
    MotorBoard.disableAllAmps(); // briefly disable motors to prevent sudden movements
    MotorBoard.initDACs();
}

void subGetStatusCallback(const std_msgs::Empty & msg)
{
    // assemble status message
    msgStatus.number_modules = MotorBoard.numModules();
    msgStatus.current_state = MCBstateToString(stateCurrent);
    uint32_t tmpIP = (uint32_t)nh.getHardware()->wiznet_ip;
    memcpy(msgStatus.ip, &tmpIP, 4);
    memcpy(msgStatus.mac, nh.getHardware()->wiznet_mac, 6);
    for (int ii = 0; ii < 6; ii++) {
        msgStatus.count_commanded[ii] = 0;
        msgStatus.count_current[ii] = 0;
        msgStatus.control_effort[ii] = MotorBoard.getEffort(ii);
        msgStatus.motor_enabled[ii] = MotorBoard.isAmpEnabled(ii);
        msgStatus.limit_switch[ii] = MotorBoard.limitSwitchState(ii);
        msgStatus.p[ii] = 0;
        msgStatus.i[ii] = 0;
        msgStatus.d[ii] = 0;
    }
    
    // publish status
    pubStatus.publish(&msgStatus);
}


//void runManualControl(void)
//{
//    // check buttons
//	MotorBoard.readButtons();
//
//	if (MotorBoard.isMenuPressed()) {
//		MotorBoard.disableAllAmps(); // stop motors during user selection
//		for (int ii = 0; ii < MotorBoard.numModules(); ii++) {
//			MotorBoard.setLEDG(ii, false);
//		}
//		MotorBoard.setLEDG(currentMotorSelected, true);
//
//		while (MotorBoard.isMenuPressed()) {
//			if (MotorBoard.isUpPressed()) {
//				MotorBoard.setLEDG(currentMotorSelected, false);
//				currentMotorSelected++;
//				if (currentMotorSelected > (MotorBoard.numModules()-1)) {
//					currentMotorSelected = 0; }
//				MotorBoard.setLEDG(currentMotorSelected, true);
//			}
//			else if (MotorBoard.isDownPressed()) {
//				MotorBoard.setLEDG(currentMotorSelected, false);
//				currentMotorSelected--;
//				if (currentMotorSelected < 0) { 
//					currentMotorSelected = (MotorBoard.numModules()-1); }
//				MotorBoard.setLEDG(currentMotorSelected, true);
//			}
//
//            MotorBoard.setCountDesired(currentMotorSelected, MotorBoard.getCountLast(currentMotorSelected)); // ensure we drive relative to current position
//
//			delayMicroseconds(400000); // wait for human's slow reaction time
//			MotorBoard.readButtons();
//		}
//		MotorBoard.setGlobalInhibit(false); // ensure globalInhibit is false
//    MotorBoard.enableAmp(currentMotorSelected);
//	}
//	else if (MotorBoard.isUpPressed()) {
//        MotorBoard.setCountDesired(currentMotorSelected, MotorBoard.getCountDesired(currentMotorSelected) + countStepManualControl);
//    }
//	else if (MotorBoard.isDownPressed()) {
//        MotorBoard.setCountDesired(currentMotorSelected, MotorBoard.getCountDesired(currentMotorSelected) - countStepManualControl);
//	}
//}

const char* MCBstateToString(MCBstate currentState) {
    const char* stateName;

    switch (currentState)
    {
    case statePowerUp:
        stateName = "Power Up";
        break;
    case stateManualIdle:
        stateName = "Manual Idle";
        break;
    case stateManualControl:
        stateName = "Manual Control";
        break;
    case stateRosInit:
        stateName = "ROS Init";
        break;
    case stateRosIdle:
        stateName = "ROS Idle";
        break;
    case stateRosControl:
        stateName = "ROS Control";
        break;
    default:
        stateName = "Error determining state";
        break;
    }

    return stateName;
}
