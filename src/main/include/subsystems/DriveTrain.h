// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "AHRS.h"
#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/Encoder.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/voltage.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

using namespace DriveTrainContsants;

class DriveTrain : public frc2::SubsystemBase
{
public:
    DriveTrain();
    /*
     * @param RT moving forward trigger
     * @param LT moving backward trigger
     */
    void DRCDrive(double RT, double LT, double rot);
    void TurnOnCoastMode();
    void TurnOnBrakeMode();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void EnableCompressor();
    void DisableCompressor();

    void SwitchHighGear(); // Slower Speed
    void SwitchLowGear();  // Faster Speed

    double GetAverageEncoderDistance();

    double GetRightEncoderDistance();
    double GetLeftEncoderDistance();

    void ArcadeDrive(double fwd);

    void MoveStraight(double distance);

    void SetMaxOutput(double maxOutput);
    void FeedTalons();
    void ResetEncoders();
    void ResetGyro();

    frc::Pose2d GetPose();

    double GetTurnRate();

    units::degree_t GetHeading() const ;

    void ResetOdometry(frc::Pose2d pose);

    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    void TankDriveVolts(units::volt_t left, units::volt_t right);

    double GetLeftEncoderSpeed();
    double GetRightEncoderSpeed();

private:
    WPI_TalonFX m_RightMotorMain;
    WPI_TalonFX m_RightMotorFollower;
    WPI_TalonFX m_LeftMotorMain;
    WPI_TalonFX m_LeftMotorFollower;

    frc::MotorControllerGroup m_Right;
    frc::MotorControllerGroup m_Left;
    frc::DifferentialDrive m_drive;

    frc::Compressor m_Compressor;
    frc::DoubleSolenoid m_GearSwitcher;

    nt::NetworkTableEntry m_xEntry;
    nt::NetworkTableEntry m_yEntry;

    void OutputData();
    // nt::NetworkTableEntry leftRef;// = table->GetEntry("left_reference");
    // nt::NetworkTableEntry leftMeas;// = table -> GetEntry("left_measurement");
    // nt::NetworkTableEntry rightRef;// = table -> GetEntry("right_reference");
    // nt::NetworkTableEntry rightMeas;// = table -> GetEntry("right_measurement");

    AHRS *m_gyro = nullptr;

    frc::DifferentialDriveOdometry m_odometry;

};
