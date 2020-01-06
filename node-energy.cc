#include "node-energy.h"
using namespace ns3;

// public method
NodeEnergy::NodeEnergy(const double supplyVoltage, const double initialEnergyJ) 
{
//	m_const_energy = supplyVoltage * 3.32 * 1.0 / vel[m/s];		// Consumption[J] per 1[m]
	m_supplyVoltage = supplyVoltage;
	m_initialEnergyJ = initialEnergyJ;
	m_totaldist = 0;
	m_totaltime = 0.0;
	m_totalEnergyConsumptionJ = 0.0;
	m_sensorEnergyJ = 2.5 * 1;		// Energy constant of LiDER (URG-04LX)
}
double
NodeEnergy::getRemainingEnergyJ()
{
	double m_remainingEnergyJ = m_initialEnergyJ - m_totalEnergyConsumptionJ;
	if(m_remainingEnergyJ < 0) {
		m_remainingEnergyJ = 0;
	}
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
}
double_t
NodeEnergy::getRadioTotalEnergyConsumptionJ(){
	return m_radioModelPtr->GetTotalEnergyConsumption();
}
void 
NodeEnergy::updateTotalEnergyConsumptionJ(const double currentA, const double time)
{
	m_currentA = currentA;
	m_totaltime = time;
	totalEnergyConsumptionJ();
}
// private method
void
NodeEnergy::totalEnergyConsumptionJ()
{
	m_totalEnergyConsumptionJ = m_radioModelPtr->GetTotalEnergyConsumption() + totalMobilityEnergyJ() + m_sensorEnergyJ;
}
double 
NodeEnergy::totalMobilityEnergyJ()
{
	return m_supplyVoltage * m_currentA * m_totaltime;
//	return m_totaldist * m_const_energy;
}
