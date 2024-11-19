#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Left1 = motor(PORT10, ratio6_1, true);
motor Left2 = motor(PORT9, ratio6_1, true);
motor Left3 = motor(PORT8, ratio6_1, true);
motor Right1 = motor(PORT7, ratio6_1, false);
motor Right2 = motor(PORT6, ratio6_1, false);
motor Right3 = motor(PORT5, ratio6_1, false);
motor Roller1 = motor(PORT3, ratio18_1, false);
motor Roller2 = motor(PORT4, ratio18_1, true);
motor Arm = motor(PORT2, ratio36_1, true);
digital_out A = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);
inertial InertialSensor = inertial(PORT20);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}