#include "subsystems/Intake.h"
#include "Constants.h"

using namespace IntakeConstants;

Intake::Intake()
    : m_IntakeMotor{kIntakemotor},
    m_IntakePusher{frc::PneumaticsModuleType::CTREPCM, kIntakePusher,kIntakePuller}
{
    m_IntakeMotor.SetNeutralMode(NeutralMode::Brake);
}

void Intake::RunIntake()
{
    m_IntakeMotor.Set(0.75);
}

void Intake::RunIntakeBack()
{
    m_IntakeMotor.Set(-0.75);
}

void Intake::StopIntake()
{
    m_IntakeMotor.Set(0.0);
}

void Intake::IntakeDown(){
    m_IntakePusher.Set(frc::DoubleSolenoid::kReverse);
}

void Intake::IntakeUp(){
    m_IntakePusher.Set(frc::DoubleSolenoid::kForward);
}

void Intake::IntakeOff(){
    m_IntakePusher.Set(frc::DoubleSolenoid::kOff);
}