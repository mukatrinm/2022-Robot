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
}

void DriveTrain::TurnOnBreakMode()
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
