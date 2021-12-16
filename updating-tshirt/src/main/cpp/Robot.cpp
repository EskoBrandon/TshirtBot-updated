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
#define XboxXL Xbox.GetX(frc::XboxController::JoystickHand(0))
#define XboxYL Xbox.GetY(frc::XboxController::JoystickHand(0))
#define XboxYR Xbox.GetY(frc::XboxController::JoystickHand(1))


void Robot::RobotInit() {
  //Right.SetInverted(true);
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
  double Xaxis = 0.0;
  double Yaxis = 0.0;
  double winchSpeed = -0.3;
  double grain = sensor.Get();
  frc::SmartDashboard::PutNumber("limit switch", grain);

  //xboxyr

  //drive code
  if (IsOperatorControl() && IsEnabled()) {
    //left Joystick
    if (XboxXL > deadzone || XboxXL < -deadzone ) {
      Xaxis = XboxXL;
    } else if (XboxYL > deadzone || XboxYL < -deadzone) {
      Yaxis = XboxYL; 
    }
    
    //right Joystick
    if(XboxYR > deadzone || XboxYR < -deadzone){
      winchSpeed = XboxYR;
      }
    else{
      winchSpeed = 0.0;
    }
    
    

  }
  //robot moving command
  myRobot.CurvatureDrive(Yaxis, Xaxis, true);
  //winch
    m_winch.Set(winchSpeed);
  //pneumatic
  if(Xbox.GetAButton()){
    Tshirt.Set(frc::DoubleSolenoid::Value(2));
  }else {
    Tshirt.Set(frc::DoubleSolenoid::Value(1));
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
