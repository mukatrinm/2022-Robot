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
    // : shooter_speed{12330}
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();
    // m_Drive.SetDefaultCommand(frc2::RunCommand(
    //     [this] {
    //         m_Drive.DRCDrive(
    //             1.0 * m_MainJoystick.GetRawAxis(frc::XboxController::Axis::kRightTrigger),
    //             1.0 * m_MainJoystick.GetRawAxis(frc::XboxController::Axis::kLeftTrigger),
    //             0.9 * m_MainJoystick.GetLeftX());
    //     },
    //     {&m_Drive}));
    // frc::SmartDashboard::PutNumber("Shooter/Input Velocity", shooter_speed);
}

void RobotContainer::EnableDTDefaultCommand()
{
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
    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kA))
        .WhenHeld(frc2::SequentialCommandGroup{
            frc2::InstantCommand{[this] { m_turret.TurnToAngleNotConstraint(-m_limelight.GetTargetAngle()); }, {&m_turret}},
            // frc2::WaitUntilCommand([this] { return m_turret.IsInTargetAngle(); }).WithTimeout(1_s),
            frc2::WaitCommand(0.4_s),
            // frc2::InstantCommand([this] { m_shooter.SetShooterVelocity(13550); }, {&m_shooter})})
            // frc2::WaitUntilCommand([this] { return m_shooter.IsInTargetVel(); }).WithTimeout(1_s),
            // frc2::WaitCommand(1.2_s),
            frc2::InstantCommand([this] { m_indexer.FeedCargo(); }, {&m_indexer})})
        .WhenReleased(
            frc2::InstantCommand{[this] { m_indexer.Stop(); }, {&m_indexer}});

    // frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kY))
    //     .WhenHeld(frc2::SequentialCommandGroup{
    //         frc2::InstantCommand{[this] { m_turret.TurnToAngleNotConstraint(-m_limelight.GetTargetAngle()); }, {&m_turret}},
    //         // frc2::WaitUntilCommand([this] { return m_turret.IsInTargetAngle(); }).WithTimeout(1_s),
    //         // frc2::WaitCommand(0.5_s),
    //         frc2::InstantCommand([this] { m_shooter.SetShooterVelocity(10290); }, {&m_shooter})})
    //         // frc2::WaitUntilCommand([this] { return m_shooter.IsInTargetVel(); }).WithTimeout(1_s),
    //         // frc2::WaitCommand(0.5_s),
    //         // frc2::InstantCommand([this] { m_indexer.FeedCargo(); }, {&m_indexer})})
    //     .WhenReleased(
    //         frc2::InstantCommand{[this] { m_shooter.StopShooter(); }});
    //         // frc2::InstantCommand{[this] { m_indexer.Stop(); }}});


    frc2::JoystickButton(&m_SecondaryJoystick, static_cast<int>(frc::XboxController::Button::kX))
        .WhenPressed(frc2::InstantCommand([this] { m_shooter.SetShooterVelocity(10310); }, {&m_shooter}));

    frc2::JoystickButton(&m_SecondaryJoystick, static_cast<int>(frc::XboxController::Button::kY))
        .WhenPressed(frc2::InstantCommand([this] { m_shooter.SetShooterVelocity(13550); }, {&m_shooter}));
    
    frc2::JoystickButton(&m_SecondaryJoystick, static_cast<int>(frc::XboxController::Button::kA))
        .WhenPressed(frc2::InstantCommand([this] { m_shooter.SetShooterVelocity(9210); }, {&m_shooter}));

    frc2::JoystickButton(&m_SecondaryJoystick, static_cast<int>(frc::XboxController::Button::kB))
        .WhenPressed(frc2::InstantCommand{[this] { m_shooter.StopShooter(); }, {&m_shooter}});


    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kX))
        .WhenPressed(&m_RunIntake);

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kB))
        .WhenPressed(&m_StopIntake);

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kRightBumper))
        .WhenPressed(frc2::InstantCommand{[this] { m_turret.RunTurretCW(); },
                                          {&m_turret}})
        .WhenReleased(frc2::InstantCommand{[this] { m_turret.StopTurret(); },
                                           {&m_turret}});

    frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kLeftBumper))
        .WhenPressed(frc2::InstantCommand{[this] { m_turret.RunTurretCCW(); },
                                          {&m_Drive}})
        .WhenReleased(frc2::InstantCommand{[this] { m_turret.StopTurret(); },
                                           {&m_turret}});

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

RobotContainer *RobotContainer::getInstance()
{
    static RobotContainer Instance;
    return &Instance;
}

DriveTrain *RobotContainer::GetDriveTrain()
{
    return &m_Drive;
}

Shooter *RobotContainer::GetShooter()
{
    return &m_shooter;
}

Indexer *RobotContainer::GetIndexer()
{
    return &m_indexer;
}

Intake *RobotContainer::GetIntake()
{
    return &m_intake;
}
Turret *RobotContainer::GetTurret()
{
    return &m_turret;
}
