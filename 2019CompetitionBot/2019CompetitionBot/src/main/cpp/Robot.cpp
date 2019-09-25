/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  compressor.SetClosedLoopControl(true);
  shifterValve.Set(true);
cargoIntakeValve.Set(frc::DoubleSolenoid::kForward);

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  elevatorEncoder.Reset();
  shifterValve.Set(true);
  armMotor.Set(ControlMode::PercentOutput, -0.1);
  frc::Wait(0.7);
  armMotor.Set(ControlMode::PercentOutput, 0.1);
  frc::Wait(0.5);
  armMotor.Set(ControlMode::PercentOutput, 0.0);
  while (encoderReading > -600){
    elevatorMotor.Set(-1.0);
  }
  elevatorMotor.Set(0.5);
  frc::Wait(0.5);
  elevatorEncoder.Reset();

}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  cargoIntakeValve.Set(frc::DoubleSolenoid::kForward);
  elevatorEncoder.Reset();
  shifterValve.Set(true);
  armMotor.Set(ControlMode::PercentOutput, -0.1);
  frc::Wait(0.7);
  armMotor.Set(ControlMode::PercentOutput, 0.1);
  frc::Wait(0.5);
  armMotor.Set(ControlMode::PercentOutput, 0.0);

}

void Robot::TeleopPeriodic() {

  //*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Intake
 /* if (armLimitSwitch.Get()) {
    //armMotor.SetPosition(0);
  }
    if (operatorController.GetRawButton(1)) {
      holdArm = true;
    } else if (operatorController.GetRawButton(2)) {
      holdArm = false;
    }*/
    oCRightYAxis = -operatorController.GetRawAxis(5);
    if (oCRightYAxis < -0.2) {
        armMotor.Set(ControlMode::PercentOutput, oCRightYAxis*0.1);
    } else if (oCRightYAxis > 0.2 ) {
        armMotor.Set(ControlMode::PercentOutput, oCRightYAxis*0.5);
    } else {
     //   if (holdArm) {
       // armMotor.Set(ControlMode::PercentOutput, 0.2);
       // } else {
        armMotor.Set(ControlMode::PercentOutput, 0);  //Talons do not automatically go back to 0
        //}
    }
   //std::cout << armMotor.GetSelectedSensorPosition(0) << std::endl;
  //*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Elevator

  if (operatorController.GetRawButton(5)){
    elevatorOverridden = true;
  }
  else if (operatorController.GetRawAxis(2) > 0.8){
    elevatorOverridden = false;
  }


  if (cargoIntakeActive) { //Decide elevator position
    elevatorLevel = cargoIntake;
  }
//else if (hatchIntakeActive) {
//  elevatorLevel = hatchIntake;
//}
  else if (operatorController.GetRawButton(1)){ 
    elevatorLevel = rocketCargoLow;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(2)){
    elevatorLevel = rocketCargoMid;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(3)){
    elevatorLevel = shipCargo;
    cargoPosition = true;
    hatchPosition = false;
  }
  else if (operatorController.GetRawButton(4)){
    elevatorLevel = rocketCargoHigh;
    cargoPosition = true;
    hatchPosition = false;
  }
  else {
    oCDPad = operatorController.GetPOV();
    if (oCDPad != -1){
      switch (oCDPad){
        case 0:
          elevatorLevel = rocketHatchHigh;
          hatchPosition = true;
          cargoPosition = false;
        break;
        case 90:
          elevatorLevel = rocketHatchMid;
          hatchPosition = true;
          cargoPosition = false;
        break;
        case 180:
          elevatorLevel = rocketHatchLow;
          hatchPosition = true;
          cargoPosition = false;
        break;
      }// end switch
    }// end inner if
  }// end if-else


  encoderReading = elevatorEncoder.Get();

//std::cout << encoderReading << "   ";

  if (elevatorOverridden){//                             >>> MANUAL ELEVATOR
    oCLeftYAxis = operatorController.GetRawAxis(1);
    if (oCLeftYAxis < -0.2 ) {
      if (encoderReading < -1000){
//      elevatorMotor.Set((oCLeftYAxis * 0.4) - 0.5);
        elevatorMotor.Set(oCLeftYAxis);
      }
      else{
        elevatorMotor.Set((oCLeftYAxis * 0.3) - 0.5);
      }
//    std::cout << "  Up" << std::endl;
    }
    else if (oCLeftYAxis > 0.2){
      if (encoderReading < -600){
        elevatorMotor.Set(oCLeftYAxis * 0.2);
      }
      else{
        elevatorMotor.Set(oCLeftYAxis * 0.5);
      }
//    std::cout << "       Down" << std::endl;
    }
    else {
      elevatorMotor.Set(-0.3); 
//    std::cout << "              Hold" << std::endl;  
    }
  }
  else{//                                           >>> AUTO ELEVATOR
    if (elevatorLevel > encoderReading) {
      if (encoderReading < -1000){
        elevatorMotor.Set(oCLeftYAxis);
      }
      else{
        elevatorMotor.Set((oCLeftYAxis * 0.3) - 0.5);
      }
    }
    else if (elevatorLevel < encoderReading){
      if (encoderReading < -600){
        elevatorMotor.Set(oCLeftYAxis * 0.2);
      }
      else{
        elevatorMotor.Set(oCLeftYAxis * 0.5);
      }
    }
    else {
      elevatorMotor.Set(-0.3); 
    }
  }

//std::cout << elevatorEncoder.Get() << std::endl;
  //*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Pneumatics
  //shifter = driverController.POV();

  if (elevatorOverridden){
    if (driverController.GetRawButton(6)) {
       cargoIntakeValve.Set(frc::DoubleSolenoid::kForward);
       rollerMotor.Set(0.0);      
     } 
     else if (driverController.GetRawAxis(3) > 0.8) {
       cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse);
      rollerMotor.Set(0.7);
     } 
  }
  else {
    if (cargoPosition and carriageParked and (driverController.GetRawAxis(3) > 0.8)){
      cargoIntakeValve.Set(frc::DoubleSolenoid::kReverse);
    }
    else if (driverController.GetRawAxis(3) > 0.8) {
      cargoIntakeActive = false;
      rollerMotor.Set(0.0);
    } 
      else if (driverController.GetRawButton(6)) {
      cargoIntakeActive = true;
      cargoIntakeValve.Set(frc::DoubleSolenoid::kForward);
      rollerMotor.Set(0.5);
      }
  }

  if (driverController.GetRawButton(5)) {
    hatchIntakeValve.Set(frc::DoubleSolenoid::kForward);
  } else if (driverController.GetRawAxis(2) > 0.8) {
    hatchIntakeValve.Set(frc::DoubleSolenoid::kReverse);
  } 


  /*if (shifter == 0) {
    shifterValve.Set(true);
  } else if (shifter == 180) {
    shifterValve.Set(false);
  } */

  //*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Drive Code
  dCLeftYAxis = driverController.GetRawAxis(1);
  dCRightYAxis = -driverController.GetRawAxis(5);

  shifter = driverController.GetPOV();

  if (shifter == 0){
    manualShift = false;
  }
  else if (shifter == 180){
    manualShift = true;
  }

  if ((((dCLeftYAxis < -0.9) and (dCRightYAxis > 0.9)) or ((dCLeftYAxis > 0.9) and (dCRightYAxis < -0.9)))
      and (not manualShift)) {
    shifterValve.Set(false);
  }
  else if (not manualShift){
    shifterValve.Set(true);
  }
  else if (driverController.GetRawButton(1)){
    shifterValve.Set(true);
  }
  else if (driverController.GetRawButton(2)){
    shifterValve.Set(false);
  }

  if (driverController.GetRawButton(7)) {
    hatchIsFront = true;
  }
  else if (driverController.GetRawButton(8)) {
    hatchIsFront = false;
  }

  if (hatchIsFront) {
    tempLeftAxis = dCLeftYAxis;
    dCLeftYAxis = dCRightYAxis;
    dCRightYAxis = tempLeftAxis;
  }
  if ((dCLeftYAxis > 0.2) or (dCLeftYAxis < -0.2)) {
    leftTopMotor.Set(dCLeftYAxis);
    leftMidMotor.Set(dCLeftYAxis);
    leftLowMotor.Set(dCLeftYAxis);
   } else {
    leftTopMotor.Set(0);
    leftMidMotor.Set(0);
    leftLowMotor.Set(0);
   }
  if ((dCRightYAxis > 0.2) or (dCRightYAxis < -0.2)) {
    rightTopMotor.Set(dCRightYAxis);
    rightMidMotor.Set(dCRightYAxis);
    rightLowMotor.Set(dCRightYAxis);
  } else {
    rightTopMotor.Set(0);
    rightMidMotor.Set(0);
    rightLowMotor.Set(0);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
