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
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/RamseteCommand.h>
#include <units/time.h>
#include "Constants.h"

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>


#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer()
: TurnTurret90{&m_turret, 90}
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

    m_chooser.SetDefaultOption("null", nullptr);
    m_chooser.AddOption("RightPath", RightPath());
    m_chooser.AddOption("MiddlePath", MiddlePath());

    frc::SmartDashboard::PutData(&m_chooser);
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

    // frc2::JoystickButton(&m_MainJoystick, static_cast<int>(frc::XboxController::Button::kA))
        // .WhenHeld(&TurnTurret90);
        // .WhenHeld(frc2::InstantCommand{[this] { m_turret.TurnToAngleNotConstraint(90); }, {&m_turret}})
        // .WhenReleased(frc2::InstantCommand{[this] { m_turret.StopTurret(); }, {&m_turret}});


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
frc2::Command* RobotContainer::GetAutonomousCommand(){
    return MiddlePath();
}

frc2::Command* RobotContainer::RightPath() {

  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainContsants::ks, DriveTrainContsants::kv, DriveTrainContsants::ka),
      RobotContainer::kDriveKinematics, 10_V);

  // Set up config for trajectory
  frc::TrajectoryConfig config(DriveTrainContsants::kMaxSpeed,
                               DriveTrainContsants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(RobotContainer::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);


  // An example trajectory to follow.  All units in meters.
  
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "output" / "Path1.wpilib.json";
    frc::Trajectory trajectory1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
    deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "output" / "Path2.wpilib.json";
    frc::Trajectory trajectory2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
//   auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//       // Start at the origin facing the +X direction
//       frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//       // Pass through these two interior waypoints, making an 's' curve path
//       {frc::Translation2d(2_m,2_m),frc::Translation2d(4_m,0_m)},
//       // End 3 meters straight ahead of where we started, facing forward
//       frc::Pose2d(6_m, 2_m, frc::Rotation2d(0_deg)),
//       // Pass the config
//       config);
    

  frc2::RamseteCommand ramseteCommand1(
      trajectory1, [this]() { return m_Drive.GetPose(); },
      frc::RamseteController(DriveTrainContsants::kRamseteB,
                             DriveTrainContsants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainContsants::ks, DriveTrainContsants::kv, DriveTrainContsants::ka),
      RobotContainer::kDriveKinematics,
      [this] { return m_Drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      [this](auto left, auto right) { m_Drive.TankDriveVolts(left, right); },
      {&m_Drive});

  // Reset odometry to the starting pose of the trajectory.

  frc2::RamseteCommand ramseteCommand2(
      trajectory2, [this]() { return m_Drive.GetPose(); },
      frc::RamseteController(DriveTrainContsants::kRamseteB,
                             DriveTrainContsants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainContsants::ks, DriveTrainContsants::kv, DriveTrainContsants::ka),
      RobotContainer::kDriveKinematics,
      [this] { return m_Drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      [this](auto left, auto right) { m_Drive.TankDriveVolts(left, right); },
      {&m_Drive});

  // Reset odometry to the starting pose of the trajectory.
  m_Drive.ResetOdometry(trajectory1.InitialPose());
    

  // no auto
    return new frc2::SequentialCommandGroup(
        TurnTurret90,
        frc2::SequentialCommandGroup {
            frc2::InstantCommand{[this] { m_shooter.RunShooter(0.6); }},
            frc2::WaitCommand(1_s),
            frc2::InstantCommand{[this] { m_indexer.FeedCargo(); }}},
        frc2::InstantCommand{[this] { m_intake.IntakeDown(); }},
        frc2::WaitCommand(3_s),
        frc2::InstantCommand([this] { m_indexer.Stop(); }, {}),
        frc2::ParallelCommandGroup {
            frc2::InstantCommand{[this] { m_intake.RunIntake(); }},
            frc2::InstantCommand{[this] { m_indexer.RunIndexer(); }}},
        std::move(ramseteCommand1),
        frc2::InstantCommand([this] { m_Drive.TankDriveVolts(0_V, 0_V); }, {&m_Drive}),
        frc2::ParallelCommandGroup {
            frc2::InstantCommand{[this] { m_intake.IntakeUp(); }},
            frc2::InstantCommand{[this] { m_intake.StopIntake(); }},
            frc2::InstantCommand{[this] { m_indexer.Stop(); }}},
        std::move(ramseteCommand2),
        frc2::InstantCommand([this] { m_Drive.TankDriveVolts(0_V, 0_V); }, {&m_Drive}),
        frc2::SequentialCommandGroup {
            frc2::InstantCommand{[this] { m_shooter.RunShooter(0.6); }},
            frc2::WaitCommand(1_s),
            frc2::InstantCommand{[this] { m_indexer.FeedCargo(); }}},
        frc2::WaitCommand(3_s),
        frc2::InstantCommand([this] { m_indexer.Stop(); }, {})
);
}

frc2::Command* RobotContainer::MiddlePath() {

  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainContsants::ks, DriveTrainContsants::kv, DriveTrainContsants::ka),
      RobotContainer::kDriveKinematics, 10_V);

  // Set up config for trajectory
  frc::TrajectoryConfig config(DriveTrainContsants::kMaxSpeed,
                               DriveTrainContsants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(RobotContainer::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);


  // An example trajectory to follow.  All units in meters.
  
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "output" / "Path3.wpilib.json";
    frc::Trajectory trajectory1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
    deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "output" / "Path4.wpilib.json";
    frc::Trajectory trajectory2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
//   auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//       // Start at the origin facing the +X direction
//       frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//       // Pass through these two interior waypoints, making an 's' curve path
//       {frc::Translation2d(2_m,2_m),frc::Translation2d(4_m,0_m)},
//       // End 3 meters straight ahead of where we started, facing forward
//       frc::Pose2d(6_m, 2_m, frc::Rotation2d(0_deg)),
//       // Pass the config
//       config);
    

  frc2::RamseteCommand ramseteCommand1(
      trajectory1, [this]() { return m_Drive.GetPose(); },
      frc::RamseteController(DriveTrainContsants::kRamseteB,
                             DriveTrainContsants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainContsants::ks, DriveTrainContsants::kv, DriveTrainContsants::ka),
      RobotContainer::kDriveKinematics,
      [this] { return m_Drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      [this](auto left, auto right) { m_Drive.TankDriveVolts(left, right); },
      {&m_Drive});

  // Reset odometry to the starting pose of the trajectory.

  frc2::RamseteCommand ramseteCommand2(
      trajectory2, [this]() { return m_Drive.GetPose(); },
      frc::RamseteController(DriveTrainContsants::kRamseteB,
                             DriveTrainContsants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainContsants::ks, DriveTrainContsants::kv, DriveTrainContsants::ka),
      RobotContainer::kDriveKinematics,
      [this] { return m_Drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      frc2::PIDController(DriveTrainContsants::kP, DriveTrainContsants::kI, DriveTrainContsants::kD),
      [this](auto left, auto right) { m_Drive.TankDriveVolts(left, right); },
      {&m_Drive});

  // Reset odometry to the starting pose of the trajectory.
  m_Drive.ResetOdometry(trajectory1.InitialPose());
    
  // no auto
    return new frc2::SequentialCommandGroup(
        TurnTurret90,
        frc2::SequentialCommandGroup {
            frc2::InstantCommand{[this] { m_shooter.RunShooter(0.6); }},
            frc2::WaitCommand(1_s),
            frc2::InstantCommand{[this] { m_indexer.FeedCargo(); }}},
        frc2::InstantCommand{[this] { m_intake.IntakeDown(); }},
        frc2::WaitCommand(3_s),
        frc2::InstantCommand([this] { m_indexer.Stop(); }, {}),
        frc2::ParallelCommandGroup {
            frc2::InstantCommand{[this] { m_intake.RunIntake(); }},
            frc2::InstantCommand{[this] { m_indexer.RunIndexer(); }}},
        std::move(ramseteCommand1),
        frc2::InstantCommand([this] { m_Drive.TankDriveVolts(0_V, 0_V); }, {}),
        frc2::ParallelCommandGroup {
            frc2::InstantCommand{[this] { m_intake.IntakeUp(); }},
            frc2::InstantCommand{[this] { m_intake.StopIntake(); }},
            frc2::InstantCommand{[this] { m_indexer.Stop(); }}},
        std::move(ramseteCommand2),
        frc2::InstantCommand([this] { m_Drive.TankDriveVolts(0_V, 0_V); }, {}),
        frc2::SequentialCommandGroup {
            frc2::InstantCommand{[this] { m_shooter.RunShooter(0.6); }},
            frc2::WaitCommand(1_s),
            frc2::InstantCommand{[this] { m_indexer.FeedCargo(); }}},
        frc2::WaitCommand(3_s),
        frc2::InstantCommand([this] { m_indexer.Stop(); }, {})
);
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

