#include "subsystems/Limelight.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <cmath>
#include <frc/Timer.h>

// void Limelight::Updater() {
//     if (m_limelightTable->GetEntry("tv").GetDouble(0) == 1) {
//         double xOffset = m_limelightTable->GetEntry("tx").GetDouble(0);
//         double yOffset = m_limelightTable->GetEntry("ty").GetDouble(0);
//         double distance = h/(std::tan(DegreesToRadians(yOffset+theta))*std::cos(DegreesToRadians(xOffset)));
//         frc::SmartDashboard::PutNumber("Limelight/Distance", distance);
//         double latency = m_limelightTable->GetEntry("tl").GetDouble(0) / 1000.0 + 0.011;
//         m_state = new VisionState(xOffset, distance, frc::Timer::GetFPGATimestamp() - latency);
//       }
// }

Limelight::Limelight()
{
}

void Limelight::Periodic()
{
    if (m_limelightTable->GetEntry("tv").GetDouble(0) == 1) {
        xOffset = m_limelightTable->GetEntry("tx").GetDouble(0);
        yOffset = m_limelightTable->GetEntry("ty").GetDouble(0);
        distance1 = h / (std::tan(DegreesToRadians(yOffset + theta)) * std::cos(DegreesToRadians(xOffset)));
        distance2 = CalculateDistanceToTargetMeters(h, 2.59, DegreesToRadians(theta), DegreesToRadians(yOffset));

        frc::SmartDashboard::PutNumber("Limelight/Distance Method 1", distance1);
        frc::SmartDashboard::PutNumber("Limelight/Distance Method 2", distance2);
        // frc::SmartDashboard::PutNumber("Limelight/tX", xOffset);
        // frc::SmartDashboard::PutNumber("Limelight/ty", yOffset);
        frc::SmartDashboard::PutBoolean("Limelight/Target Found", HasTarget());
        frc::SmartDashboard::PutBoolean("Limelight/Target In Range", IsInRange());

        latency = m_limelightTable->GetEntry("tl").GetDouble(0) / 1000.0 + 0.011;
        // m_state = VisionState{xOffset, distance, frc::Timer::GetFPGATimestamp() - latency};
    } else {
        frc::SmartDashboard::PutBoolean("Limelight/Target Found", HasTarget());
        frc::SmartDashboard::PutBoolean("Limelight/Target In Range", false);
    }
}

double Limelight::GetTargetAngle()
{
    if (HasTarget()) {
        return xOffset;
    } else {
        return 0.0;
    }
}

double Limelight::CalculateDistanceToTargetMeters(
    double cameraHeightMeters,
    double targetHeightMeters,
    double cameraPitchRadians,
    double targetPitchRadians)
{
    return (targetHeightMeters - cameraHeightMeters) / std::tan(cameraPitchRadians + targetPitchRadians);
}

double Limelight::GetDistanceToTarget() {
    return distance2;
}

// VisionState Limelight::GetState()
// {
//     return m_state;
// }

bool Limelight::HasTarget()
{
    m_targetFound = m_limelightTable->GetEntry("tv").GetDouble(0) == 1.0;
    return m_targetFound;
}

bool Limelight::IsInRange()
{
    return (distance2 >=1.81 && distance2 <= 2.25);
}

void Limelight::TurnOffLEDs()
{
    m_limelightTable->GetEntry("ledMode").SetDouble(1);
}

void Limelight::TurnOnLEDs()
{
    m_limelightTable->GetEntry("ledMode").SetDouble(3);
}