#include "subsystems/Turret.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

Turret::Turret(DriveTrain *driveTrain, Limelight *limelight) : m_driveTrain{driveTrain},
                                                               m_limelight{limelight},
                                                               m_turretMotor{TurretConstants::kTurret}
{
    m_turretMotor.SetInverted(true);
    m_turretMotor.SetSensorPhase(false);
    
    m_turretMotor.SetNeutralMode(NeutralMode::Brake);

    InitClosedLoop();

    ZeroEncoder();
}

void Turret::Periodic()
{
    // target_angle = m_visionTable->GetEntry("TargetAngle").GetDouble(0.0);
    // m_CurrentAngle = m_turretMotor.GetSelectedSensorPosition(0) * (90 / 5586);
    // TurnToAngle_Turret(GetTargetAngle());
    // if(IsShooterHead4Target())
    //     StopTurret();
    // else
    //     TurnToAngle_Turret(GetTargetAngle());
    // frc::SmartDashboard::PutNumber("Is Angle???", m_visionTable->GetEntry("TargetAngle").GetDouble(0.0));
    // frc::SmartDashboard::PutNumber("WE GET Angle???", target_angle);
    OutputToDashboard();

    // calc max velocity
    // double turrVel = m_turretMotor.GetSelectedSensorVelocity();
    // if (maxVel < std::abs(turrVel)) {
    //     maxVel = std::abs(turrVel);
    //     frc::SmartDashboard::PutNumber("Turret/Max Velocity-2", maxVel);
    // }

    m_currentAngle = m_turretMotor.GetSelectedSensorPosition() / TurretConstants::kTicksPerAngle;
}

double Turret::ConstrainToAcceptableRange(double angle)
{
    double goal = angle;
    while (goal < TurretConstants::kAcceptableStartRange) {
        goal += 360;
    }
    while (goal > TurretConstants::kAcceptableEndRange) {
        goal -= 360;
    }
    return goal;
}

double Turret::GetTargetAngle()
{
    return m_target;
}

bool Turret::IsInTargetAngle()
{
    return (std::abs(m_target - m_currentAngle) <= 15);
}

void Turret::FeedTalons()
{
    m_turretMotor.Feed();
}

void Turret::EnableBrakeMode()
{
    m_turretMotor.SetNeutralMode(NeutralMode::Brake);
}

void Turret::EnableCoastMode()
{
    m_turretMotor.SetNeutralMode(NeutralMode::Coast);
}

void Turret::InitClosedLoop()
{
    m_turretMotor.ConfigFactoryDefault();

    m_turretMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, TurretConstants::kTimeoutMs);
    m_turretMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, TurretConstants::kTimeoutMs);

    m_turretMotor.ConfigMotionSCurveStrength(TurretConstants::kSmoothing, 10);

    m_turretMotor.ConfigNominalOutputForward(0, TurretConstants::kTimeoutMs);
    m_turretMotor.ConfigNominalOutputReverse(0, TurretConstants::kTimeoutMs);
    m_turretMotor.ConfigPeakOutputForward(1.0, TurretConstants::kTimeoutMs);
    m_turretMotor.ConfigPeakOutputReverse(-1.0, TurretConstants::kTimeoutMs);

    m_turretMotor.SelectProfileSlot(TurretConstants::kSlotIdx, TurretConstants::kPIDLoopIdx);
    m_turretMotor.Config_kF(TurretConstants::kSlotIdx, TurretConstants::kFTurret, TurretConstants::kTimeoutMs);
    m_turretMotor.Config_kP(TurretConstants::kSlotIdx, TurretConstants::kPTurret, TurretConstants::kTimeoutMs);
    m_turretMotor.Config_kI(TurretConstants::kSlotIdx, TurretConstants::kITurret, TurretConstants::kTimeoutMs);
    m_turretMotor.Config_kD(TurretConstants::kSlotIdx, TurretConstants::kDTurret, TurretConstants::kTimeoutMs);

    m_turretMotor.ConfigMotionCruiseVelocity(TurretConstants::kMaxSensorVelocity, TurretConstants::kTimeoutMs);
    m_turretMotor.ConfigMotionAcceleration(TurretConstants::kMaxSensorAcceleration, TurretConstants::kTimeoutMs);
}

void Turret::ZeroEncoder()
{
    m_turretMotor.SetSelectedSensorPosition(0);
}

void Turret::TurnToAngle(double target)
{
    double angle_to_enc = target * TurretConstants::kTicksPerAngle;
    if (m_limelight->HasTarget()) {

        // m_turretMotor.SetSelectedSensorPosition(0);

        frc::SmartDashboard::PutNumber("Turret/Target Angle", angle_to_enc);

        m_turretMotor.Set(ControlMode::MotionMagic, ConstrainToAcceptableRange(m_currentAngle + angle_to_enc));


    } else {
        // Odometry will be accurate in auto, so we can rely on that.
        // auto pose = m_driveTrain->GetPose();
        // auto goal = WayPointConstants::kGoalLocation;
        // double turn = std::atan2((double)(goal.Y().value()) - (double)(pose.Y().value()), (double)(goal.X().value()) - (double)(pose.X().value())) - (double)m_driveTrain->GetHeading().value();
        // m_turretMotor.Set(ControlMode::MotionMagic, ConstrainToAcceptableRange(turn));
    }
}

void Turret::TurnToAngleNotConstraint(double target)
{
    double angle_to_enc = target * TurretConstants::kTicksPerAngle;
    m_turretMotor.SetSelectedSensorPosition(0);

    frc::SmartDashboard::PutNumber("Turret/Target Angle", angle_to_enc);

    m_turretMotor.Set(ControlMode::MotionMagic, angle_to_enc);
}

void Turret::RunTurretCCW()
{
    m_turretMotor.Set(ControlMode::PercentOutput, TurretConstants::kTurretSpeed);
}

void Turret::RunTurretCW()
{
    m_turretMotor.Set(ControlMode::PercentOutput, -TurretConstants::kTurretSpeed);
}

void Turret::StopTurret()
{
    m_turretMotor.Set(ControlMode::PercentOutput, 0);
}

void Turret::OutputToDashboard()
{
    // frc::SmartDashboard::PutNumber("Turret/TurretSensorVel-2", m_turretMotor.GetSelectedSensorVelocity());
    // frc::SmartDashboard::PutNumber("Turret/TurretSensorPos-2", m_turretMotor.GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("Turret MotorOutputPercent", m_turretMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Turret/ClosedLoopError", m_turretMotor.GetClosedLoopError());

    // frc::SmartDashboard::PutNumber("Turret ClosedLoopTarget",  m_turretMotor.GetClosedLoopTarget(0));
    // frc::SmartDashboard::PutNumber("Turret ActTrajVelocity",  m_turretMotor.GetActiveTrajectoryVelocity());
    // frc::SmartDashboard::PutNumber("Turret ActTrajPosition",  m_turretMotor.GetActiveTrajectoryPosition());

    // frc::SmartDashboard::PutBoolean("Is it found???", m_visionTable->GetEntry("TargetFound").GetBoolean(false));

    // add an entry listener for changed values of "Y", the lambda ("->" operator)
    // defines the code that should run when "Y" changes
    //   yEntry.addListener(event -> {
    //      System.out.println("Y changed value: " + value.getValue());
    //   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // frc::SmartDashboard::PutBoolean("Is VisionActive?", m_visionTable->GetEntry("VisionActive").GetBoolean(false));
    //  m_visionTable->AddEntryListener("Is VisionActive?", (table, key, entry, value, flags) -> {
    //      System.out.println("X changed value: " + value.getValue());
    //   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
}

// void Turret::Head4Target(){
//     if(InSafeZone() && m_visionTable->GetEntry("TargetFound").GetBoolean(false)){
//      std::cout<< "The Condition Is True" << std::endl;

//         TurnToAngle_Turret(target_angle);
//     }
//     else
//     {
//         TurnToAngle_Turret(-m_CurrentAngle);
//     }
// }

// bool Turret::IsShooterHead4Target(){
//     return (GetTargetAngle() > (-1.0) && GetTargetAngle() < 1.0);
// }