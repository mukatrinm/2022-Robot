#include "Commands\TurnTurret.h"

TurnTurret::TurnTurret(Turret* subsystem, double Target)
: m_Turret(subsystem), m_Target(Target) {
  AddRequirements(subsystem);
}

void TurnTurret::Initialize(){
    std::cout << "*******************Initialize TurnTurret Started*********************************" << std::endl;
    m_Turret->TurnToAngleNotConstraint(m_Target);
}

void TurnTurret::Execute(){
    std::cout << "*******************Execute TurnTurret Started*********************************" << std::endl;
}

bool TurnTurret::IsFinished(){
    std::cout << abs(m_Turret->GetAngle() - m_Target) << std::endl;
    return abs(m_Turret->GetAngle() - m_Target)<2;
}

void TurnTurret::End(bool interrupted){
    std::cout << "******************* "<< interrupted << std::endl;
    m_Turret->StopTurret();

}