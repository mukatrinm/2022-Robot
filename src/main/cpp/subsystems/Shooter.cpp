#include "subsystems/Shooter.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

Shooter::Shooter() : m_ShooterMotor{ShooterConstants::kShootermotor},
      m_ShooterFollowerMotor{ShooterConstants::kShooterFollowermotor}
{
    m_ShooterMotor.ConfigFactoryDefault();
    m_ShooterFollowerMotor.ConfigFactoryDefault();
    // m_HoodMotor.ConfigFactoryDefault();

    m_ShooterFollowerMotor.Follow(m_ShooterMotor);

    m_ShooterMotor.SetInverted(false);
    m_ShooterFollowerMotor.SetInverted(true);
    // m_HoodMotor.SetInverted(false);

    m_ShooterMotor.ConfigNominalOutputForward(0, ShooterConstants::kTimeoutMs);
    m_ShooterMotor.ConfigNominalOutputReverse(0, ShooterConstants::kTimeoutMs);
    m_ShooterMotor.ConfigPeakOutputForward(1, ShooterConstants::kTimeoutMs);
    m_ShooterMotor.ConfigPeakOutputReverse(-1, ShooterConstants::kTimeoutMs);
    m_ShooterMotor.Config_kF(ShooterConstants::kPIDLoopIdx, ShooterConstants::kShooterF, ShooterConstants::kTimeoutMs);
    m_ShooterMotor.Config_kP(ShooterConstants::kPIDLoopIdx, ShooterConstants::kShooterP, ShooterConstants::kTimeoutMs);
    m_ShooterMotor.Config_kI(ShooterConstants::kPIDLoopIdx, ShooterConstants::kShooterI, ShooterConstants::kTimeoutMs);
    m_ShooterMotor.Config_kD(ShooterConstants::kPIDLoopIdx, ShooterConstants::kShooterD, ShooterConstants::kTimeoutMs);

    // m_HoodMotor.ConfigNominalOutputForward(0, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.ConfigNominalOutputReverse(0, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.ConfigPeakOutputForward(1, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.ConfigPeakOutputReverse(-1, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.Config_kF(ShooterConstants::kPIDLoopIdx, ShooterConstants::kHoodF, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.Config_kP(ShooterConstants::kPIDLoopIdx, ShooterConstants::kHoodP, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.Config_kI(ShooterConstants::kPIDLoopIdx, ShooterConstants::kHoodI, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.Config_kD(ShooterConstants::kPIDLoopIdx, ShooterConstants::kHoodD, ShooterConstants::kTimeoutMs);
 
    // m_HoodMotor.ConfigMotionCruiseVelocity(ShooterConstants::kHoodMaxSensorVelocity, ShooterConstants::kTimeoutMs);
    // m_HoodMotor.ConfigMotionAcceleration(ShooterConstants::kHoodMaxSensorAcceleration, ShooterConstants::kTimeoutMs);

    m_ShooterMotor.EnableVoltageCompensation(12);
    m_ShooterFollowerMotor.EnableVoltageCompensation(12);
    // m_HoodMotor.EnableVoltageCompensation(12);

    m_ShooterMotor.SetSensorPhase(false);
    // m_HoodMotor.SetSensorPhase(true);

    m_ShooterMotor.SetNeutralMode(NeutralMode::Coast);
    m_ShooterFollowerMotor.SetNeutralMode(NeutralMode::Coast);
    // m_HoodMotor.SetNeutralMode(NeutralMode::Brake);

    // m_HoodMotor.ConfigPeakCurrentLimit(30);
    // m_HoodMotor.ConfigPeakCurrentDuration(100);
    // m_HoodMotor.ConfigContinuousCurrentLimit(20);
    // m_HoodMotor.EnableCurrentLimit(true);
}

void Shooter::Periodic()
{
    OutputData();
}

void Shooter::OutputData()
{
    frc::SmartDashboard::PutNumber("Shooter/Velocity", GetShooterVelocity());
    frc::SmartDashboard::PutNumber("Shooter/Velocity", m_ShooterMotor.GetClosedLoopError());
    frc::SmartDashboard::PutNumber("Shooter/RPM", (m_ShooterMotor.GetSelectedSensorVelocity(0) * 600) / ShooterConstants::kCountsPerRev);
    frc::SmartDashboard::PutNumber("Shooter/Target Velocity", m_targetVelocity);
    // frc::SmartDashboard::PutNumber("Hood/m_HoodMotorSensorPos", m_HoodMotor.GetSelectedSensorPosition(0));
    // frc::SmartDashboard::PutNumber("Hood/Target Angle", m_reference);

    // calc max velocity
    // double hoodVel = m_HoodMotor.GetSelectedSensorVelocity();
    // static double maxVel = 0.0;
    // if (maxVel < std::abs(hoodVel))
    // {
    //     maxVel = std::abs(hoodVel);
    //     // frc::SmartDashboard::PutNumber("Hood/Max Velocity", maxVel);
    // }  
}

void Shooter::FeedTalons()
{
    m_ShooterMotor.Feed();
    m_ShooterFollowerMotor.Feed();
}

double Shooter::GetRevolutionsPerMinute()
{
    return (m_ShooterMotor.GetSelectedSensorVelocity(0) * 600) / ShooterConstants::kCountsPerRev;
}

void Shooter::SetShooterVelocity(double velocity)
{
    m_targetVelocity = velocity;
    // double targetVelocity_UnitsPer100ms = velocity * ShooterConstants::kShooterEncoderCPR / 600.0; // convert RPM to sensor velocity
    // m_ShooterMotor.Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms);

    m_ShooterMotor.Set(ControlMode::Velocity, velocity);
}

bool Shooter::IsInTargetVel()
{
    return (std::abs(GetTargetVelocity() - GetShooterVelocity()) <= 50);
}

double Shooter::GetTargetVelocity()
{
    return m_targetVelocity;
}

double Shooter::GetShooterVelocity()
{
    return m_ShooterMotor.GetSelectedSensorVelocity(0);
}

void Shooter::RunShooter(double speed)
{
    m_ShooterMotor.Set(ControlMode::PercentOutput, speed);
    // m_ShooterMotor.Set(frc::SmartDashboard::GetNumber("Shooter/Speed", 1000));
}

void Shooter::StopShooter()
{
    m_ShooterMotor.Set(ControlMode::PercentOutput, 0.0);
}
