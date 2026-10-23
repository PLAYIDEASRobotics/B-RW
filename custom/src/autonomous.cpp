#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

// 750 for 45 deg turns, 1000 for 90 deg turns, 1500 for 180 deg turns

void linearTuning() {

  driveTo(5, 2000, true, 12);
  cascade_motor.spin(fwd,12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0, volt);
  driveTo(-5, 1000, true, 12);
  wait(250, msec);
  driveTo(5, 1000, true, 12);
  wait(250, msec);
  driveTo(-5, 1000, true, 12);
  wait(250, msec);
  driveTo(12, 1000);
  clawPivot.set(true);
  wait(250, msec);
  turnToAngle(270, 2500);
  wait(250, msec);
  driveTo(12, 1000);
  wait(250, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(250, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);

  driveTo(-7.5, 1000, true, 6);
  wait(250, msec);
  turnToAngle(215, 2500);
  wait(250, msec);
  driveTo(19, 1000, true, 4);
  wait(100, msec);
  clawGrab.set(false);
  wait(100, msec);
  driveTo(-19, 1000, true, 4);
  wait(250, msec);
  turnToAngle(270, 2500);
  wait(250, msec);

  cascade_motor.spin(fwd,12,volt);
  wait(750, msec);
  cascade_motor.spin(fwd,0, volt);

  driveTo(7, 1000, true, 6);
  wait(250, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(250, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);

  /*driveTo(8, 2000);
  cascade_motor.spin(fwd,12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0, volt);
  driveTo(-10, 1000);
  wait(500, msec);
  driveTo(10, 1000);
  wait(500, msec);
  driveTo(-10, 1000);
  clawPivot.set(true);
  driveTo(13, 1000);
  wait(500, msec);
  turnToAngle(270, 2500);
  wait(500, msec);
  driveTo(12, 1000);
  wait(500, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);*/

  /*driveTo(6, 3000);
  wait(100, msec);
  driveTo(-6, 3000);
  wait(5,sec);
  driveTo(-6, 3000);
  driveTo(-12, 3000);
  driveTo(-24, 3000);
  wait(5,sec);
  driveTo(60, 3000);
  wait(5,sec);
  driveTo(-60, 3000);
  */
}

void turnTuning() {

  driveTo(5, 2000, true, 12);
  cascade_motor.spin(fwd,12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0, volt);
  driveTo(-5, 1000, true, 12);
  wait(250, msec);
  driveTo(5, 1000, true, 12);
  wait(250, msec);
  driveTo(-5, 1000, true, 12);
  wait(250, msec);
  driveTo(11.5, 2000, true, 6);
  clawPivot.set(true);
  wait(250, msec);
  turnToAngle(90, 2500);
  wait(250, msec);
  driveTo(12, 2000, true, 6);
  wait(250, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(250, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);

  driveTo(-10, 1000, true, 4);
  wait(250, msec);
  turnToAngle(35, 2500);
  wait(250, msec);
  driveTo(20, 3000, true, 4);
  wait(50, msec);
  clawGrab.set(false);
  wait(100, msec);
  driveTo(-21, 3000, true, 4);
  wait(250, msec);
  turnToAngle(90, 2500);
  wait(250, msec);

  cascade_motor.spin(fwd,12,volt);
  wait(1500, msec);
  cascade_motor.spin(fwd,0, volt);

  driveTo(10, 2000, true, 6);
  wait(250, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(250, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);

  /*clawPivot.set(true);
  cascade_motor.spin(fwd,12,volt);
  wait(300, msec);
  cascade_motor.spin(fwd,0, volt);
  driveTo(5, 1000);
  wait(1000, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(200, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);
  wait(1500, msec);
  cascade_motor.spin(fwd,12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0, volt);
  wait(500, msec);
  turnToAngle(55, 2500);
  wait(1500, msec);
  driveTo(-15, 2000);
  wait(500, msec);
  turnToAngle(325, 2500);
  wait(1500, msec);
  driveTo(-20, 2000);
  wait(500, msec);
  driveTo(20, 2000, true, 5);*/

 /* turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  */
}

void odomExample() {

  clawPivot.set(true);
  cascade_motor.spin(fwd,12,volt);
  wait(300, msec);
  cascade_motor.spin(fwd,0, volt);
  driveTo(5, 1000);
  wait(1000, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(200, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);
  wait(2000, msec);
  cascade_motor.spin(fwd,12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0, volt);
  wait(500, msec);
  turnToAngle(55, 2500);
  wait(3000, msec);
  driveTo(-16, 2000);
  wait(500, msec);
  turnToAngle(325, 2500);
  wait(200, msec);
  cascade_motor.spin(fwd,-12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0,volt);
  wait(2000, msec);
  driveTo(-20, 2000);
  wait(500, msec);
  driveTo(25, 10000, true, 4);
  wait(200, msec);
  clawGrab.set(false);
  wait(200, msec);
  cascade_motor.spin(fwd,12,volt);
  wait(1000, msec);
  cascade_motor.spin(fwd,0, volt);
  clawRotate.set(true);
  wait(500, msec);
  driveTo(25, 10000, true, 6);
  cascade_motor.spin(fwd,-12,volt);
  wait(500, msec);
  cascade_motor.spin(fwd,0,volt);
  clawGrab.set(true);
  

  /*moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);*/
}

void boomerangTest(){
  boomerang(24,0,1,90,.6,3000,false,12,false);
  boomerang(36,36,1,0,.6,3000,true,12,false);
  boomerang(0,0,-1,0,.6,3000,true,12,false);
}

/*
Start of Actual Autons
*/

void intakeBlocks(){

  driveTo(24, 2500);
  driveTo(-12, 2000);
  driveTo(-12, 2000);

  //intake_motor.spin(fwd,12,volt);
  //scoringM.spin(fwd,1,volt);
}

void scoreHigh(){

  turnToAngle(45, 1250);
  turnToAngle(90, 1250);
  turnToAngle(180, 1500);
  turnToAngle(0, 2000);

  //intake_motor.spin(fwd,12,volt);
  //scoringM.spin(fwd,12,volt);
}

void scoreMid(){
  intake_motor.spin(fwd,-6,volt);
  //scoringM.spin(fwd,11,volt);
  wait(200,msec);
  intake_motor.spin(fwd,12,volt);
  //scoringM.spin(fwd,1,volt);
}

void scoreLow(){
  intake_motor.spin(fwd,-12,volt);
  //scoringM.spin(fwd,-12,volt);
}

void pauseIntake(){
  intake_motor.stop();
  //scoringM.stop();
}



void rightMatchload(){


  /*
  intakeBlocks();                                       //start intake
  boomerang(5.5,24,1,70,.3,3000);                       //move to collect 3 blocks
  scraper.set(true);                                    //open scraper
  boomerang(37.5,-20,1,180,.3,3000);                    //move to matchloader
  
  //wait(3,sec);

  correct_angle = 180;
  driveTo(6,3000,true,6);                               //move forward to unload matchloader
  wait(1,sec);                                          //wait for blocks
  moveToPoint(37.5,10,-1,3000);                         //move to long goal
  scraper.set(false);                                   //bring scraper up
  scoreHigh();                                          //score in long goal
  wait(2,sec);                                          //timer for scoring
  pauseIntake();                                        //stop intaking

  wait(3,sec);

  moveToPoint(31.5,16.223,1,3000);                      //move to set up for wing
  wing.set(true);                                       //open wings
  moveToPoint(31.5,45,-1,3000);                         //move backwards to push for control bonus
  stopChassis(hold); 
  */
  //hold position
}

void rightLowHigh(){
  intakeBlocks();                                       //start intake
  driveTo(15,2000);
  turnToAngle(30,2000);
  scraper.set(true);
  driveTo(11,3000,true,3.5);
  wait(500,msec);
  scraper.set(false);
  //driveTo(-4,3000,true,5);
  turnToAngle(-45,2000);
  driveTo(14,2000);
  scoreLow();
  wait(2000, msec);
  driveTo(-47, 3000, true, 4);
  turnToAngle(-180,1000);
  scraper.set(true);
  wait(500, msec);
  intakeBlocks();
  driveTo(10,1000);
  wait(2500, msec);
  driveTo(-22,2000);
  scoreHigh();
  

}

void left4_3(){

  cascade_motor.spin(fwd, 10, volt);
  wait(1000, msec);
  cascade_motor.spin(fwd, -10, volt);
  wait(750, msec);
  cascade_motor.spin(fwd, 0, volt);
  driveTo(-0.5, 3000, true, 2);
  wait(500, msec);
  cascade_motor.spin(fwd, 10, volt);
  wait(1000, msec);
  cascade_motor.spin(fwd, -10, volt);
  wait(750, msec);
  cascade_motor.spin(fwd, 0, volt);


  /*intakeBlocks();                                       //start intake
  boomerang(-1,31,1,-50,.3,3000,false);                 //move to collect 3 blocks
  scraper.set(true);                                    //open scraper to trap blocks
  wait(200,msec);                                       //wait .2s
  scraper.set(false);                                   //close scraper
  boomerang(-25,51,1,-80,.3,3000,false);                //move to matchloader
  scraper.set(true);                                    //open scraper to trap blocks
  wait(200,msec);                                       //wait .2s
  scraper.set(false);                                   //close scraper
  
  wait(3,sec);

  correct_angle = -80;                                  //heading for pid straight
  driveTo(3,3000,true,6);                               //move forward to unload matchloader
  boomerang(5,44,-1,-135,3000,true);                    //move to mid goal
  scoreMid();                                           //score in mid goal
  wait(1,sec);                                          //timer for scoring
  intakeBlocks();                                       //go back to intake mode

  wait(3,sec);

  scraper.set(true);                                    //open scraper
  boomerang(-30,0,1,180,.3,3000);                       //move to matchloader
  correct_angle = 180;                                  //heading for matchload
  driveTo(6,3000);                                      //move forward to unload matchloader
  wait(1,sec);                                          //wait for blocks

  wait(3,sec);

  moveToPoint(-30,24,-1,3000);                          //move to long goal
  scraper.set(false);                                   //bring scraper up
  scoreHigh();                                          //score in long goal
  wait(2,sec);                                          //timer for scoring
  pauseIntake();                                        //stop intaking

  wait(3,sec);

  moveToPoint(21.5,16.223,1,3000);                      //move to set up for wing
  wing.set(true);                                       //open wings
  moveToPoint(21.5,45,-1,3000);                         //move backwards to push for control bonus
  stopChassis(hold); 
  */                                   //hold position
}
