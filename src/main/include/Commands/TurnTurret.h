#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <iostream>

#include "../subsystems/Turret.h"

class TurnTurret
    : public frc2::CommandHelper<frc2::CommandBase, TurnTurret> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit TurnTurret(Turret* subsystem, double Target);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;


 private:
  Turret* m_Turret;
  double m_Target;
};
