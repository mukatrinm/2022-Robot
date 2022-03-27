// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "subsystems/Climber.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "subsystems/Limelight.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"

#include <frc/XboxController.h>

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>

class RobotContainer
{
public:
    RobotContainer();

    frc2::Command *GetAutonomousCommand();

    // frc2::SequentialCommandGroup m_ShootUpperHub{
    //   frc2::InstantCommand{[this]{m_shooter.SetShooterVelocity(12290);}},
    //   frc2::WaitUntilCommand{[this] { return m_shooter.IsInTargetVel(); }}.WithTimeout(2_s),
    //   frc2::InstantCommand{[this]{m_indexer.FeedCargo();}}
    //   };

    frc2::SequentialCommandGroup m_StopShooting{
        frc2::InstantCommand{[this] { m_shooter.StopShooter(); }},
        frc2::InstantCommand{[this] { m_indexer.Stop(); }}};

    frc2::ParallelCommandGroup m_RunIntake{
        frc2::InstantCommand{[this] { m_intake.IntakeDown(); }},
        frc2::InstantCommand{[this] { m_intake.RunIntake(); }},
        frc2::InstantCommand{[this] { m_indexer.RunIndexer(); }}};

    frc2::ParallelCommandGroup m_StopIntake{
        frc2::InstantCommand{[this] { m_intake.IntakeUp(); }},
        frc2::InstantCommand{[this] { m_intake.StopIntake(); }},
        frc2::InstantCommand{[this] { m_indexer.Stop(); }}};

    frc2::SequentialCommandGroup Auto{
        frc2::InstantCommand{[this] { m_shooter.RunShooter(0.68); }},
        frc2::WaitCommand(1_s),
        frc2::InstantCommand{[this] { m_indexer.FeedCargo(); }},
        frc2::WaitCommand(5_s),
        frc2::InstantCommand{[this] { m_shooter.StopShooter(); }},
        frc2::InstantCommand{[this] { m_indexer.Stop(); }},
        frc2::InstantCommand{[this] { m_Drive.DRCDrive(0, 0.7, 0); }, {&m_Drive}},
        frc2::WaitCommand(2_s),
        frc2::InstantCommand{[this] { m_Drive.DRCDrive(0, 0.0, 0); }, {&m_Drive}}};

    frc2::InstantCommand m_RunClimberUp{[this] { m_climber.RunClimberUp(); }, {}};
    frc2::InstantCommand m_RunClimberDown{[this] { m_climber.RunClimberDown(); }, {}};
    frc2::InstantCommand m_StopClimber{[this] { m_climber.StopClimber(); }, {}};
    frc2::InstantCommand m_PushClimber{[this] { m_climber.PushClimber(); }, {}};
    frc2::InstantCommand m_PullClimber{[this] { m_climber.PullClimber(); }, {}};
    frc2::InstantCommand m_OffClimber{[this] { m_climber.ClimberOff(); }, {}};

    frc2::InstantCommand m_switchHighGear{[this] { m_Drive.SwitchHighGear(); }, {}};
    frc2::InstantCommand m_switchLowGear{[this] { m_Drive.SwitchLowGear(); }, {}};

    frc2::InstantCommand m_runTurretCCW{[this] { m_turret.RunTurretCCW(); },
                                        {&m_turret}};

    frc2::InstantCommand m_runTurretCW{[this] { m_turret.RunTurretCW(); },
                                       {&m_turret}};

    frc2::InstantCommand m_StopTurret{[this] { m_turret.StopTurret(); },
                                      {&m_turret}};

    frc2::InstantCommand m_switch2Breakmode{[this] { m_Drive.TurnOnBreakMode(); },
                                            {}};

    frc2::InstantCommand m_switch2Coastmode{[this] { m_Drive.TurnOnCoastMode(); },
                                            {}};

    frc2::InstantCommand m_moveB{[this] { m_Drive.DRCDrive(0, 0.5, 0); }, {}};
    frc2::InstantCommand m_StopDriveTrain{[this] { m_Drive.DRCDrive(0, 0.0, 0); }, {}};

    static RobotContainer *getInstance();
    DriveTrain *GetDriveTrain();
    Shooter *GetShooter();
    Indexer *GetIndexer();
    Intake *GetIntake();
    Turret *GetTurret();

private:
    void ConfigureButtonBindings();

    DriveTrain m_Drive;
    Indexer m_indexer;
    Intake m_intake;
    Shooter m_shooter;
    Climber m_climber;
    Limelight m_limelight;
    Turret m_turret{&m_Drive, &m_limelight};

    frc::XboxController m_MainJoystick{OI::kMainController};
    frc::XboxController m_SecondaryJoystick{OI::kSecondaryController};
};
