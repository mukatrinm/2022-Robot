#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <subsystems/DriveTrain.h>
#include <subsystems/Limelight.h>

class Turret : public frc2::SubsystemBase
{
public:
    Turret(DriveTrain* driveTrain, Limelight *limelight);

    void Periodic() override;
    void FeedTalons();
    double GetAngle();
    void RunTurretCCW();
    void RunTurretCW();
    void StopTurret();
    void TurnToAngle(double target);
    void TurnToAngleNotConstraint(double target); // balayez em7ane baleez
    void InitClosedLoop();
    void ZeroEncoder();
    void OutputToDashboard();
    double GetTargetAngle();
    bool InSafeZone();

private:
    DriveTrain *m_driveTrain;
    Limelight *m_limelight;
    WPI_TalonSRX m_turretMotor;
    double m_target;
    void OutputData();
    double ConstrainToAcceptableRange(double angle);
    

    double maxVel = 0.0;
    double m_currentAngle = 0.0;
};