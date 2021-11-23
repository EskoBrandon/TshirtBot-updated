// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/ArcadeDriveXboxController/cpp/Robot.cpp
//https://docs.wpilib.org/en/stable/docs/software/actuators/wpi-drive-classes.html?
#include "Robot.h"

#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

//shortens xboxController.GetX and xboxController.GetY 
#define XboxX Xbox.GetX(frc::XboxController::JoystickHand(0))
#define XboxY Xbox.GetY(frc::XboxController::JoystickHand(0))

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double deadzone = 0.4;
  //myRobot.ArcadeDrive(Xbox.GetY(frc::XboxController::JoystickHand(0)), XboxX, true);
  double bozo = 0.0;
  double hello = 0.0;
  if (IsOperatorControl() && IsEnabled()) {
    //myRobot.ArcadeDrive(XboxY, XboxX, true);
    //deadzone
    if(XboxX > deadzone || XboxX < -deadzone){
      bozo = XboxX;
    }
    if(XboxY > deadzone || XboxY < -deadzone){
      hello = XboxY;
    }
    myRobot.ArcadeDrive(hello, bozo, true);
    if(Xbox.GetYButton()){
      
    }
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
