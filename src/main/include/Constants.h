// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "wpi/numbers"
#include "units/math.h"
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>

namespace OI
{
    constexpr int kMainController = 0;
    constexpr int kSecondaryController = 1;
}

namespace DriveTrainContsants
{
    constexpr int kLeftMotorMain = 0;
    constexpr int kLeftMotorFollower = 1;

    constexpr int kRightMotorMain = 2;
    constexpr int kRightMotorFollower = 3;

    constexpr int kCompressor = 0;
    constexpr int kGearSwitcher0 = 4;
    constexpr int kGearSwitcher1 = 5;

    constexpr bool kGyroReversed = true;

    constexpr double kLeftEncoderCPR = 2048.0;
    constexpr double kRightEncoderCPR = 2048.0;

    constexpr double kWheelCircumference = 0.1524 * wpi::numbers::pi;

    constexpr double kDriveDistPerTick = (3 * kWheelCircumference) / 79000.0; // kWheelCircumference / kLeftEncoderCPR / kDriveGearing; // meters per encoder tick

    // motion magic constants
    // constexpr int kSlotIdx = 0;
    // constexpr int kTimeoutMs = 30;
    // constexpr int kPIDLoopIdx = 0;
    // constexpr double kDriveF = 0.04513235294;
    // constexpr double kDriveP = 0.04204685573*8; //0.1217857143;
    // constexpr double kDriveI = 0.00001;
    // constexpr double kDriveD = kDriveP*10;
    // constexpr double kDriveMaxSensorVelocity = 17000;
    // constexpr double kDriveMaxSensorAcceleration =12000;

    constexpr auto ks = 0.59885_V;
    constexpr auto kv = 2.9253 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.50718 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kP = 0.0074247;
    constexpr double kD = 0.00069987;
    constexpr double kI = 0;

    constexpr auto kTrackwidth = 0.625_m;

    extern const frc::DifferentialDriveKinematics kDriveKinematics;

    constexpr auto kMaxSpeed = 2.0_mps;
    constexpr auto kMaxAcceleration = 1.0_mps_sq;

    constexpr auto kRamseteB = 2 * 1_rad * 1_rad / (1_m * 1_m);
    constexpr auto kRamseteZeta = 0.7 / 1_rad;
}

namespace IntakeConstants
{
    constexpr int kIntakemotor = 4;

    constexpr int kIntakePusher = 1;
    constexpr int kIntakePuller = 0;
}

namespace IndexerConstants
{
    constexpr int kIndexerMotor = 8;
    constexpr int kFeederMotor = 9;
}

namespace ShooterConstants
{
    constexpr int kShootermotor = 5;
    constexpr int kShooterFollowermotor = 6;

    constexpr double kLowerHubSpeed = 0.35;
    constexpr double kUpperHubSpeed = 0.73;

    constexpr double kCountsPerRev = 2048.0;
    constexpr int kTimeoutMs = 30;
    constexpr int kPIDLoopIdx = 0;
    constexpr double kShooterF = 0.05719310822;
    constexpr double kShooterP = 0.14594; // 0.4447826087; //0.1217857143;
    constexpr double kShooterI = 0.00001;
    constexpr double kShooterD = 1.4594; // kShooterP*10;
    
    constexpr double kP = 0.000054827;
    constexpr double kI = 0;
    constexpr double kD = 0;

    constexpr double kS = 0.73996;
    constexpr double kV = 0.11616;
    constexpr double kA = 0.0059513;

}

namespace ClimberConstants
{
    constexpr int kClimberMotor = 10;
    constexpr int kClimberPusher = 2;
    constexpr int kClimberPuller = 3;
}

namespace TurretConstants
{
    constexpr int kTurret = 11;
    constexpr double kTurretSpeed = 0.55;
    constexpr double kMaxSensorVelocity = 22000;
    constexpr double kMaxSensorAcceleration = 20000;

    constexpr double kPTurret = 0.444;
    constexpr double kITurret = 0.002;
    constexpr double kDTurret = 4.44;
    constexpr double kFTurret = 0.0465;

    constexpr double kTicksPerAngle = (602716) / 360.0;
    constexpr int kSlotIdx = 0;
    constexpr int kPIDLoopIdx = 0;
    constexpr int kTimeoutMs = 10;

    constexpr int kTicksPerRev = 2048;
    constexpr int kSmoothing = 4;

    constexpr double kAcceptableStartRange = 0;
    constexpr double kAcceptableEndRange = 360;
} // namespace TurretConstants
