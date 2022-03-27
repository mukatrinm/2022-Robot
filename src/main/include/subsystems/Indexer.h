#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>

class Indexer : public frc2::SubsystemBase
{
public:
  Indexer();
  void RunIndexer();
  void RunFeeder();
  void FeedCargo();
  void Throw();
  void RunIndexerBack();
  void RunFeederBack();
  void Stop();

private:
  WPI_TalonSRX m_IndexerMotor;
  WPI_TalonSRX m_FeedingMotor;
};