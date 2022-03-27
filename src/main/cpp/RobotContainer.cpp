// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Command.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();
    m_Drive.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_Drive.DRCDrive(
                1.0 * m_MainJoystick.GetRawAxis(frc::XboxController::Axis::kRightTrigger),
                1.0 * m_MainJoystick.GetRawAxis(frc::XboxController::Axis::kLeftTrigger),
                0.9 * m_MainJoystick.GetLeftX());
        },
        {&m_Drive}));
}

void RobotContainer::ConfigureButtonBindings()
{
    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kY))
        .WhenHeld(frc2::SequentialCommandGroup{
            frc2::InstantCommand([this] { m_shooter.SetShooterVelocity(12290); }, {&m_shooter}),
            frc2::WaitUntilCommand([this] { return m_shooter.IsInTargetVel(); }).WithTimeout(1_s),
            frc2::InstantCommand([this] { m_indexer.FeedCargo(); }, {&m_indexer})})
        .WhenReleased(frc2::SequentialCommandGroup{
            frc2::InstantCommand{[this] { m_shooter.StopShooter(); }},
            frc2::InstantCommand{[this] { m_indexer.Stop(); }}});

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kX))
        .WhenPressed(&m_RunIntake);

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kB))
        .WhenPressed(&m_StopIntake);

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kRightBumper))
        .WhenPressed(&m_runTurretCW)
        .WhenReleased(&m_StopTurret);

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kLeftBumper))
        .WhenPressed(&m_runTurretCCW)
        .WhenReleased(&m_StopTurret);

    frc2::POVButton(&m_MainJoystick, 0)
        .WhenPressed(&m_RunClimberUp)
        .WhenReleased(&m_StopClimber);

    frc2::POVButton(&m_MainJoystick, 180)
        .WhenPressed(&m_RunClimberDown)
        .WhenReleased(&m_StopClimber);

    frc2::POVButton(&m_MainJoystick, 270)
        .WhenPressed(&m_PushClimber)
        .WhenReleased(&m_OffClimber);

    frc2::POVButton(&m_MainJoystick, 90)
        .WhenPressed(&m_PullClimber)
        .WhenReleased(&m_OffClimber);

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kStart))
        .WhenPressed(&m_switchHighGear);

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kBack))
        .WhenPressed(&m_switchLowGear);
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // An example command will be run in autonomous
    // return nullptr;
    return &Auto;
}
