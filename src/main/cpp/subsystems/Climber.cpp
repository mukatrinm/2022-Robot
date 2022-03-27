#include "subsystems/Climber.h"
#include "Constants.h"

using namespace ClimberConstants;

Climber::Climber()
: m_ClimberMotor{kClimberMotor},
m_climberSolenoid{frc::PneumaticsModuleType::CTREPCM, kClimberPusher, kClimberPuller}
{
    m_ClimberMotor.ConfigFactoryDefault();
    m_ClimberMotor.SetNeutralMode(NeutralMode::Brake);
    m_ClimberMotor.SetInverted(true);
}

void Climber::RunClimberUp() {
    m_ClimberMotor.Set(1.0);
}

void Climber::RunClimberDown() {
    m_ClimberMotor.Set(-1.0);
}

void Climber::StopClimber() {
    m_ClimberMotor.Set(0.0);
}

void Climber::PushClimber(){
    m_climberSolenoid.Set(frc::DoubleSolenoid::kReverse);
}

void Climber::PullClimber(){
    m_climberSolenoid.Set(frc::DoubleSolenoid::kForward);
}

void Climber::ClimberOff(){
    m_climberSolenoid.Set(frc::DoubleSolenoid::kOff);
}