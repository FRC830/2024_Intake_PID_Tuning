// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/StringTopic.h>
#include "rev/CANSparkMax.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/XboxController.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // SubSystemConfig.h
  const double VOLT_COMP = 10.5;

  const int LFT_INTAKE_PVT_MTR = 10;
  const int RGT_INTAKE_PVT_MTR = 1;

  const int INTAKE_PVT_ENCODER_ID = 8; 

  const double INTAKE_POS_TO_DEG = 1.0;

  const double INTAKE_INPUT_TO_DEG = 2.0;

  const int INTAKE_PVT_CURRENT_LIMIT = 20;

  // IntakeHAL.h
  rev::CANSparkMax m_RGTPvtMotor{RGT_INTAKE_PVT_MTR, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_LFTPvtMotor{LFT_INTAKE_PVT_MTR, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkAbsoluteEncoder m_LFTPvtAbsEncoder = m_LFTPvtMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
  rev::SparkPIDController m_LFTPvtPID = m_LFTPvtMotor.GetPIDController();

  //ctre::phoenix6::hardware::CANcoder m_PvtCanCoder{INTAKE_PVT_ENCODER_ID, "rio"};

  nt::DoublePublisher pub_angle;
  nt::DoublePublisher pub_speed;
  nt::StringPublisher pub_profileState;

  // Values
  double SET_POINT = 0.0;
  double PVT_P = 0.15;
  double PVT_I = 0.0;
  double PVT_D = 0.0;
  double PVT_FF = 0.0;
  double LFT_PVT_ABS_ENC_ZERO_OFFSET = 0.0;
  bool LFTPvtAbsEncInverted = false;
  double LFTPvtAbsEncPositionConversionFactor = 360.0;
  bool LFTPvtMotorInverted = true;
  // bool RGTPvtMotorInverted = false;
};
