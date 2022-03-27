#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>

class Climber : public frc2::SubsystemBase
{
public:
  Climber();
  void RunClimberUp();
  void RunClimberDown();
  void StopClimber();
  void PushClimber();
  void PullClimber();
  void ClimberOff();

private:
  WPI_TalonFX m_ClimberMotor;
  frc::DoubleSolenoid m_climberSolenoid;
};