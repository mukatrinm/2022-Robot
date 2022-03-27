#pragma once

#include "util/VisionState.h"
#include <frc2/command/SubsystemBase.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class Limelight : public frc2::SubsystemBase
{
public:
    Limelight();
    void Periodic() override;
    // VisionState GetState();
    bool HasTarget();
    void TurnOffLEDs();
    void TurnOnLEDs();
    double GetTargetAngle();
    double GetDistanceToTarget();

    double DegreesToRadians(double degree)
    {
        double pi = 3.14159265359;
        return (degree * (pi / 180));
    }

private:
    void Updater();
    double CalculateDistanceToTargetMeters(
        double cameraHeightMeters,
        double targetHeightMeters,
        double cameraPitchRadians,
        double targetPitchRadians);
    nt::NetworkTableInstance m_instance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> m_limelightTable = m_instance.GetTable("limelight");
    double m_target;
    double xOffset;
    double yOffset;
    double distance1;
    double distance2;
    double latency;

    double m_CurrentAngle = 0.0;
    double target_angle = 0.0;
    double h = 1.4; // TODO: change to correct height
    double theta = 35;
    volatile VisionState m_state{0, 0, 0};
    bool m_targetFound = false;
};