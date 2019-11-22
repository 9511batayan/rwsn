#include "node-energy.h"
using namespace ns3;

// public method
NodeEnergy::NodeEnergy(const double_t supplyVoltage, const double_t initialEnergyJ) : m_initialEnergyJ(initialEnergyJ)
{
	m_const_energy = supplyVoltage * m_initialEnergyJ;
	m_totaldist = 0;
	m_totalEnergyConsumptionJ = 0;
}
double
NodeEnergy::getRemainingEnergyJ()
{
	return m_remainingEnergyJ;
}
void
NodeEnergy::addBasicSourcePtr(const Ptr<BasicEnergySource> bes)
{
	m_basicSourcePtr = bes;
}
void
NodeEnergy::addRadioModelPtr(const Ptr<DeviceEnergyModel> dem)
{
	m_radioModelPtr = dem;
}
void 
NodeEnergy::sumDist(const double dist)
{
	m_totaldist +=dist;
	calcRemainingEnergyJ();
}
double_t
NodeEnergy::getRadioTotalEnergyConsumptionJ(){
	return m_radioModelPtr->GetTotalEnergyConsumption();
}

// private method
void
NodeEnergy::calcRemainingEnergyJ()
{
	m_remainingEnergyJ = m_initialEnergyJ - (m_radioModelPtr->GetTotalEnergyConsumption() + totalMobilityEnergyJ());
	if(m_remainingEnergyJ < 0) {
		m_remainingEnergyJ = 0;
	}
}
double_t
NodeEnergy::totalEnergyConsumptionJ()
{
	m_totalEnergyConsumptionJ = m_radioModelPtr->GetTotalEnergyConsumption() + totalMobilityEnergyJ();
	return  m_totalEnergyConsumptionJ;
}
double 
NodeEnergy::totalMobilityEnergyJ()
{
	return m_totaldist * m_const_energy;
}
