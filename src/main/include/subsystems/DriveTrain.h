// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "AHRS.h"
#include "Constants.h"
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>

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

    void ArcadeDrive(double fwd);

    void MoveSrtaight(double distance);

    double SetMaxOutput();
    void FeedTalons();
    void ResetEncoders();

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
};
