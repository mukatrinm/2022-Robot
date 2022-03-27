#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>


class Shooter : public frc2::SubsystemBase
{
public:
    Shooter();

    void Periodic() override;
    void FeedTalons();

    void RunShooter(double speed);
    void StopShooter();

    double GetRevolutionsPerMinute();
    void SetShooterVelocity(double velocity);
    double GetShooterVelocity();
    double GetTargetVelocity();
    bool IsInTargetVel();

private:
    WPI_TalonFX m_ShooterMotor;
    WPI_TalonFX m_ShooterFollowerMotor;

    double m_targetVelocity = 0.0;

    void OutputData();
};
