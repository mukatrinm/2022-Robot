// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace OI {
    constexpr int kMainController = 0;
    constexpr int kSecondaryController = 1;
}

namespace DriveTrainContsants {
    constexpr int kLeftMotorMain = 0;
    constexpr int kLeftMotorFollower = 1;

    constexpr int kRightMotorMain = 2;
    constexpr int kRightMotorFollower = 3;

    constexpr int kCompressor = 0;
    constexpr int kGearSwitcher0 = 4;
    constexpr int kGearSwitcher1 = 5;

    constexpr bool kGyroReversed = true;
}


namespace IntakeConstants {
    constexpr int kIntakemotor = 4;

    constexpr int kIntakePusher = 1;
    constexpr int kIntakePuller = 0;
}

namespace IndexerConstants {
    constexpr int kIndexerMotor = 8;
    constexpr int kFeederMotor = 9;
}

namespace ShooterConstants {
    constexpr int kShootermotor = 5;
    constexpr int kShooterFollowermotor = 6;

    constexpr double kLowerHubSpeed = 0.35;
    constexpr double kUpperHubSpeed = 0.73;

    constexpr double kCountsPerRev = 2048.0;
    constexpr int kTimeoutMs = 30;
    constexpr int kPIDLoopIdx = 0;
    constexpr double kShooterF = 0.05719310822 ;
    constexpr double kShooterP = 0.15;//0.4447826087; //0.1217857143;
    constexpr double kShooterI = 0.000;
    constexpr double kShooterD = 0.0;//kShooterP*10;
}

namespace ClimberConstants {
    constexpr int kClimberMotor = 10;
    constexpr int kClimberPusher = 2;
    constexpr int kClimberPuller = 3;
}

namespace TurretConstants {
    constexpr int kTurret = 11;
    constexpr double kTurretSpeed = 0.3;
    constexpr double kMaxSensorVelocity = 8000;
    constexpr double kMaxSensorAcceleration = 11000;

    constexpr double kPTurret = 0.01229604255*128;
    constexpr double kITurret = 0.002;
    constexpr double kDTurret = kPTurret*10;
    constexpr double kFTurret = 0.12148125;

    constexpr double kTicksPerAngle = 33279/360.0;
    constexpr int kSlotIdx = 0;
    constexpr int kPIDLoopIdx = 0;
    constexpr int kTimeoutMs = 10;

    constexpr int kTicksPerRev = 4096;
    constexpr int kSmoothing = 4;

    constexpr double kAcceptableStartRange = 0;
    constexpr double kAcceptableEndRange = 360;
} // namespace TurretConstants