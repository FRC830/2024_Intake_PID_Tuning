// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // IntakeHAL.cpp
  m_LFTPvtMotor.RestoreFactoryDefaults();
  m_RGTPvtMotor.RestoreFactoryDefaults();

  m_RGTPvtMotor.Follow(m_LFTPvtMotor, false);

  m_LFTPvtMotor.EnableVoltageCompensation(VOLT_COMP);
  m_RGTPvtMotor.EnableVoltageCompensation(VOLT_COMP);

  m_LFTPvtPID.SetP(PVT_P);
  m_LFTPvtPID.SetI(PVT_I);
  m_LFTPvtPID.SetD(PVT_D);
  m_LFTPvtPID.SetFF(PVT_FF);

  m_LFTPvtMotor.SetSmartCurrentLimit(INTAKE_PVT_CURRENT_LIMIT);
  m_RGTPvtMotor.SetSmartCurrentLimit(INTAKE_PVT_CURRENT_LIMIT);

  m_LFTPvtPID.SetFeedbackDevice(m_LFTPvtAbsEncoder);

  // Set values
  frc::SmartDashboard::PutNumber("Set Point", SET_POINT);
  frc::SmartDashboard::PutNumber("Pivot P", PVT_P);
  frc::SmartDashboard::PutNumber("Pivot I", PVT_I);
  frc::SmartDashboard::PutNumber("Pivot D", PVT_D);
  frc::SmartDashboard::PutNumber("Pivot FF", PVT_FF);
  //m_RGTPvtMotor.SetInverted(RGTPvtMotorInverted);

  m_LFTPvtAbsEncoder.SetPositionConversionFactor(LFTPvtAbsEncPositionConversionFactor);
  m_LFTPvtMotor.SetInverted(LFTPvtMotorInverted);
  
  m_LFTPvtMotor.BurnFlash();
  m_RGTPvtMotor.BurnFlash();

  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("intake");
  pub_angle = table->GetDoubleTopic("intake").Publish();
  pub_profileState = table->GetStringTopic("intake").Publish();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
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
  fmt::print("Auto selected: {}\n", m_autoSelected);

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

void Robot::TeleopInit() 
{
  // frc::SmartDashboard::PutNumber("Set Point", m_LFTPvtAbsEncoder.GetPosition());
  SET_POINT = frc::SmartDashboard::GetNumber("Set Point", m_LFTPvtAbsEncoder.GetPosition());
}

void Robot::TeleopPeriodic() {
  bool burn = false;

  if (std::fabs(m_LFTPvtPID.GetP() - frc::SmartDashboard::GetNumber("Pivot P", 0.0)) > 0.0001)
  {
    m_LFTPvtPID.SetP(frc::SmartDashboard::GetNumber("Pivot P", 0.0));
    burn = true;
  }

  if (std::fabs(m_LFTPvtPID.GetI() - frc::SmartDashboard::GetNumber("Pivot I", 0.0)) > 0.0001)
  {
    m_LFTPvtPID.SetI(frc::SmartDashboard::GetNumber("Pivot I", 0.0));
    burn = true;
  }
  
  if (std::fabs(m_LFTPvtPID.GetD() - frc::SmartDashboard::GetNumber("Pivot D", 0.0)) > 0.0001)
  {
    m_LFTPvtPID.SetD(frc::SmartDashboard::GetNumber("Pivot D", 0.0));
    burn = true;
  }
  
  if (std::fabs(m_LFTPvtPID.GetFF() - frc::SmartDashboard::GetNumber("Pivot FF", 0.0)) > 0.0001)
  {
    m_LFTPvtPID.SetFF(frc::SmartDashboard::GetNumber("Pivot FF", 0.0));
    burn = true;
  }

  if (burn)
  {
    m_LFTPvtMotor.BurnFlash();
  }

  m_LFTPvtPID.SetReference(SET_POINT, rev::ControlType::kPosition);
  frc::SmartDashboard::PutNumber("Left Pivot Encoder Position", m_LFTPvtAbsEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Left Pivot Encoder Zero Offset", m_LFTPvtAbsEncoder.GetZeroOffset());
  frc::SmartDashboard::PutBoolean("Left Pivot Encoder Inverted", m_LFTPvtAbsEncoder.GetInverted());
  frc::SmartDashboard::PutNumber("Left Pivot Encoder Position Conversion Factor", m_LFTPvtAbsEncoder.GetPositionConversionFactor());
  frc::SmartDashboard::PutBoolean("Left Pivot Motor Inverted", m_LFTPvtMotor.GetInverted());
  frc::SmartDashboard::PutBoolean("Right Pivot Motor Inverted", m_RGTPvtMotor.GetInverted());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
