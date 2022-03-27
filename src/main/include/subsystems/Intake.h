#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>

class Intake : public frc2::SubsystemBase
{
public:
  Intake();
  void RunIntake();
  void RunIntakeBack();
  void StopIntake();

  void IntakeDown();
  void IntakeUp();
  void IntakeOff();

private:
  WPI_TalonSRX m_IntakeMotor;
  frc::DoubleSolenoid m_IntakePusher;
};