#include "subsystems/Indexer.h"
#include "Constants.h"

using namespace IndexerConstants;

Indexer::Indexer()
: m_IndexerMotor{kIndexerMotor},
m_FeedingMotor{kFeederMotor}
{
    m_IndexerMotor.SetNeutralMode(NeutralMode::Brake);
    m_FeedingMotor.SetNeutralMode(NeutralMode::Brake);
    m_IndexerMotor.SetInverted(true);
    m_FeedingMotor.SetInverted(true);
}

void Indexer::RunIndexer() {
    m_IndexerMotor.Set(0.4);
}

void Indexer::RunIndexerBack() {
    m_IndexerMotor.Set(-0.4);
}

void Indexer::RunFeeder() {
    m_FeedingMotor.Set(-1.0);
}

void Indexer::RunFeederBack() {
    m_FeedingMotor.Set(1.0);
}

void Indexer::FeedCargo() {
    RunIndexer();
    RunFeeder();
}

void Indexer::Throw() {
    m_IndexerMotor.Set(0.65);
    //RunFeederBack();
}

void Indexer::Stop() {
    m_IndexerMotor.Set(0.0);
    m_FeedingMotor.Set(0.0);
}