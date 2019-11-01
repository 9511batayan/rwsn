#include "node_energy.h"
using namespace ns3;
using namespace std;

// public method
NodeEnergy::NodeEnergy(const double supplyVoltage, const double initialEnergyJ)
{
	m_supplyVoltage = supplyVoltage;
	m_initialEnergyJ = initialEnergyJ;
}
double
NodeEnergy::RemainingEnergyJ()
{
	return m_initialEnergyJ - totalEnergyConsumptionJ();
}


// private method
double_t
NodeEnergy::totalEnergyConsumptionJ()
{
	return 0;
}
double 
NodeEnergy::totalMobilityEnergyJ()
{
	return 0;
}
