// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include "frc/smartdashboard/SmartDashboard.h"

DriveTrain::DriveTrain()
    : SubsystemBase(),
      m_RightMotorMain{kRightMotorMain},
      m_RightMotorFollower{kRightMotorFollower},
      m_LeftMotorMain{kLeftMotorMain},
      m_LeftMotorFollower{kLeftMotorFollower},
      m_Right{m_RightMotorMain, m_RightMotorFollower},
      m_Left{m_LeftMotorMain, m_LeftMotorFollower},
      m_drive{m_Left, m_Right},
      m_Compressor{kCompressor, frc::PneumaticsModuleType::CTREPCM},
      m_GearSwitcher{frc::PneumaticsModuleType::CTREPCM, kGearSwitcher0, kGearSwitcher1},
      m_xEntry{nt::NetworkTableInstance::GetDefault().GetTable("troubleshooting")->GetEntry("X")},
      m_yEntry{nt::NetworkTableInstance::GetDefault().GetTable("troubleshooting")->GetEntry("Y")}
    //   leftRef{nt::NetworkTableInstance::GetDefault().GetTable("troubleshooting")->GetEntry("left_reference")},
    //   leftMeas{nt::NetworkTableInstance::GetDefault().GetTable("troubleshooting")->GetEntry("left_measurement")},
    //   rightRef{nt::NetworkTableInstance::GetDefault().GetTable("troubleshooting")->GetEntry("right_reference")},
    //   rightMeas{nt::NetworkTableInstance::GetDefault().GetTable("troubleshooting")->GetEntry("right_measurement")}

{   
    m_RightMotorMain.ConfigFactoryDefault();
    m_RightMotorFollower.ConfigFactoryDefault();
    m_LeftMotorMain.ConfigFactoryDefault();
    m_LeftMotorFollower.ConfigFactoryDefault();

    m_RightMotorMain.SetNeutralMode(NeutralMode::Brake);
    m_RightMotorFollower.SetNeutralMode(NeutralMode::Brake);
    m_LeftMotorMain.SetNeutralMode(NeutralMode::Brake);
    m_LeftMotorFollower.SetNeutralMode(NeutralMode::Brake);

    m_RightMotorMain.SetSensorPhase(false);
    m_RightMotorFollower.SetSensorPhase(false);
    m_LeftMotorMain.SetSensorPhase(false);
    m_LeftMotorFollower.SetSensorPhase(false);

    m_RightMotorFollower.Follow(m_RightMotorMain);
    m_LeftMotorFollower.Follow(m_LeftMotorMain);

    m_RightMotorMain.SetInverted(true);
    m_RightMotorFollower.SetInverted(true);
    m_LeftMotorMain.SetInverted(false);
    m_LeftMotorFollower.SetInverted(false);

    ResetEncoders();

    m_RightMotorMain.SelectProfileSlot(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kPIDLoopIdx);
    m_RightMotorMain.Config_kF(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveF, DriveTrainContsants::kTimeoutMs);
    m_RightMotorMain.Config_kP(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveP, DriveTrainContsants::kTimeoutMs);
    m_RightMotorMain.Config_kI(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveI, DriveTrainContsants::kTimeoutMs);
    m_RightMotorMain.Config_kD(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveD, DriveTrainContsants::kTimeoutMs);

    m_RightMotorMain.ConfigMotionCruiseVelocity(DriveTrainContsants::kDriveMaxSensorVelocity, DriveTrainContsants::kTimeoutMs);
    m_RightMotorMain.ConfigMotionAcceleration(DriveTrainContsants::kDriveMaxSensorAcceleration, DriveTrainContsants::kTimeoutMs);

    m_LeftMotorMain.SelectProfileSlot(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kPIDLoopIdx);
    m_LeftMotorMain.Config_kF(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveF, DriveTrainContsants::kTimeoutMs);
    m_LeftMotorMain.Config_kP(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveP, DriveTrainContsants::kTimeoutMs);
    m_LeftMotorMain.Config_kI(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveI, DriveTrainContsants::kTimeoutMs);
    m_LeftMotorMain.Config_kD(DriveTrainContsants::kSlotIdx, DriveTrainContsants::kDriveD, DriveTrainContsants::kTimeoutMs);

    m_LeftMotorMain.ConfigMotionCruiseVelocity(DriveTrainContsants::kDriveMaxSensorVelocity, DriveTrainContsants::kTimeoutMs);
    m_LeftMotorMain.ConfigMotionAcceleration(DriveTrainContsants::kDriveMaxSensorAcceleration, DriveTrainContsants::kTimeoutMs);

}

void DriveTrain::ResetEncoders()
{
    m_RightMotorMain.SetSelectedSensorPosition(0);
    m_LeftMotorMain.SetSelectedSensorPosition(0);
}

void DriveTrain::MoveSrtaight(double distance) {    
    ResetEncoders();
    m_LeftMotorMain.Set(ControlMode::MotionMagic, distance / kDriveDistPerTick);
    m_RightMotorMain.Set(ControlMode::MotionMagic, distance / kDriveDistPerTick);
}

void DriveTrain::TurnOnBrakeMode()
{
    m_RightMotorMain.SetNeutralMode(NeutralMode::Brake);
    m_RightMotorFollower.SetNeutralMode(NeutralMode::Brake);
    m_LeftMotorMain.SetNeutralMode(NeutralMode::Brake);
    m_LeftMotorFollower.SetNeutralMode(NeutralMode::Brake);
}

void DriveTrain::TurnOnCoastMode()
{
    m_RightMotorMain.SetNeutralMode(NeutralMode::Coast);
    m_RightMotorFollower.SetNeutralMode(NeutralMode::Coast);
    m_LeftMotorMain.SetNeutralMode(NeutralMode::Coast);
    m_LeftMotorFollower.SetNeutralMode(NeutralMode::Coast);
}

void DriveTrain::Periodic()
{
    FeedTalons();
    OutputData();

    //calc max velocity
    // double hoodVel = m_LeftMotorMain.GetSelectedSensorVelocity();
    // static double maxVel = 0.0;
    // if (maxVel < std::abs(hoodVel))
    // {
    //     maxVel = std::abs(hoodVel);
    //     frc::SmartDashboard::PutNumber("DT/Max Left Velocity", maxVel);
    // }  
}

void DriveTrain::OutputData()
{
    // frc::SmartDashboard::PutNumber("DT/RightPosition", m_RightMotorMain.GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("DT/RightVelocity", m_RightMotorMain.GetSelectedSensorVelocity());
    // frc::SmartDashboard::PutNumber("DT/Right error", m_RightMotorMain.GetClosedLoopError());

    // frc::SmartDashboard::PutNumber("DT/LeftPosition", m_LeftMotorMain.GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("DT/LeftVelocity", m_LeftMotorMain.GetSelectedSensorVelocity());
    // frc::SmartDashboard::PutNumber("DT/Left error", m_LeftMotorMain.GetClosedLoopError());
}

void DriveTrain::ArcadeDrive(double fwd)
{
    
    m_LeftMotorMain.Set(fwd);
    m_RightMotorMain.Set(fwd);

}

void DriveTrain::FeedTalons()
{
    m_LeftMotorMain.Feed();
    m_RightMotorMain.Feed();
    m_drive.Feed();
}

void DriveTrain::DRCDrive(double RT, double LT, double rot)
{
    m_drive.ArcadeDrive(pow(RT, 3) - pow(LT, 3), pow(rot, 3), false);
}

void DriveTrain::SwitchHighGear()
{
    m_GearSwitcher.Set(frc::DoubleSolenoid::kForward);
}

void DriveTrain::SwitchLowGear()
{
    m_GearSwitcher.Set(frc::DoubleSolenoid::kReverse);
}

void DriveTrain::EnableCompressor()
{
    m_Compressor.EnableDigital();
}

void DriveTrain::DisableCompressor()
{
    m_Compressor.Disable();
}
