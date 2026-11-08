#include "vex.h"
#include "motor-control.h"
#include "../include/autonomous.h"


bool auto_started = false;
// Modify autonomous, driver, or pre-auton code below
int auton_selected = 3;
void runAutonomous() {
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "running auton: %d", auton_selected);
  auto_started = true;
  switch(auton_selected) {
    case 0:
      linearTuning();
      break;
    case 1:
      turnTuning();
      break;
    case 2:
      skillsAuto();
      break;  
    case 3:
      rightQuadrantAuto();
      break;
    case 4:
      leftQuadrantAuto();
      break; 
    case 5:
      boomerangTest();
      break;
    case 6:
      intakeBlocks();
      break;
    case 7:
      scoreHigh();
      break;
    case 8:
      left4_3();
      break;
    case 9:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;


void intakeC(){
  if (r1) {
    intake_motor.setVelocity(100, percent);
    intake_motor.spin(forward);
  } else if (r2) {
    intake_motor.setVelocity(100, percent);
    intake_motor.spin(reverse);
  } else {
    intake_motor.stop();
  }
}

void liftC(){
  if (l1) {
      cascade_motor.spin(fwd,10,volt);
    } else if (l2) {
      cascade_motor.spin(fwd,-6,volt);
    } else {
      cascade_motor.stop(hold);
    }
}

bool button_b_previous = false;
bool button_y_previous = false;
bool button_right_arrow_previous = false;

void pistonCs(){
  // Toggle only on the moment of press
  if (button_b && !button_b_previous) {
    clawGrab.set(!clawGrab.value());
  }
  button_b_previous = button_b;

  if (button_y && !button_y_previous) {
    clawPivot.set(!clawPivot.value());
  }
  button_y_previous = button_y;

  if (button_right_arrow && !button_right_arrow_previous) {
    clawRotate.set(!clawRotate.value());
  }
  button_right_arrow_previous = button_right_arrow;
}


void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.position();
    ch2 = controller_1.Axis2.position();
    ch3 = controller_1.Axis3.position();
    ch4 = controller_1.Axis4.position();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    //drivesmoothing
    //DriveSmoothing(ch3,ch1);

    //default drive code 
    driveChassis((ch1 * 0.12) + (ch3 * .12), (ch3 * 0.12) - (ch1 * .12));
    intakeC();
    liftC();
    pistonCs();
    wait(10, msec); 
  }
}



void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
  while(!auto_started){
    Brain.Screen.printAt(5, 20, "PiRA-RW");
    Brain.Screen.printAt(5, 50, "Battery Percentage:");
    Brain.Screen.printAt(5, 70, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 90, "Chassis Heading Reading:");
    double current_heading = inertial_sensor.heading();
    Brain.Screen.printAt(5, 110, "%d", current_heading);
    Brain.Screen.printAt(5, 130, "Selected Auton:");
    Brain.Screen.printAt(5, 150, "%d", auton_selected);
    printf("%d",auton_selected);
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      auton_selected++;
    } else if (auton_selected == 8){
      auton_selected = 0;
    }
    wait(10, msec);
  }
}