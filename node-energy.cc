#include "node-energy.h"
using namespace ns3;

// public method
NodeEnergy::NodeEnergy(const double supplyVoltage, const double currentA, const double initialEnergyJ, const double vel) : m_initialEnergyJ(initialEnergyJ)
{
	m_const_mobEnergy = supplyVoltage * currentA * 1.0 / vel;		// Consumption[J] per meter
	m_diff_dist = 0;
	m_diff_time = 0.0;
	m_totalEnergyConsumptionJ = 0.0;
	m_mobTotalEnergyConsumptionJ = 0.0;
	m_sensorTotalEnergyConsumptionJ = 0.0;		
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
NodeEnergy::updateTotalEnergyConsumptionJ(const double time, const double diff_dist)
{
	static double last_time = 0.0;
	m_diff_time = time - last_time;
	m_diff_dist = diff_dist;
	totalEnergyConsumptionJ();
	last_time = time;
}
double
NodeEnergy::getTotalEnergyConsumptionJ() const
{
	return m_totalEnergyConsumptionJ;
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
double
NodeEnergy::getRadioTotalEnergyConsumptionJ() const 
{
	return m_radioTotalEnergyConsumptionJ;
}

// private method
void
NodeEnergy::totalEnergyConsumptionJ()
{
	m_totalEnergyConsumptionJ = updateTotalRadioEnergyConsumptionJ() + updateTotalMobilityEnergyConsumptionJ() + updateTotalSensorEnergyConsumptionJ();
}
double
NodeEnergy::updateTotalRadioEnergyConsumptionJ()
{
	m_radioTotalEnergyConsumptionJ = m_radioModelPtr->GetTotalEnergyConsumption();
	return m_radioTotalEnergyConsumptionJ;
}
double 
NodeEnergy::updateTotalMobilityEnergyConsumptionJ()
{
	m_mobTotalEnergyConsumptionJ += m_const_mobEnergy * m_diff_dist;
	return m_mobTotalEnergyConsumptionJ;
}
double 
NodeEnergy::updateTotalSensorEnergyConsumptionJ()
{
	// Energy constant of LiDER (URG-04LX)
	// LiDER is 2.5 W
	m_sensorTotalEnergyConsumptionJ += 2.5 * m_diff_time;
	return m_sensorTotalEnergyConsumptionJ;
}
