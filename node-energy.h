#ifndef NODEENERGY_H
#define NODEENERGY_H
#include "ns3/energy-module.h"
using namespace ns3;
class NodeEnergy
{
	public:
		NodeEnergy(const double_t supplyVoltage, const double_t initialEnergyJ);		// constructor
		double getRemainingEnergyJ();
		void sumDist(const double dist);
		void addBasicSourcePtr(const Ptr<BasicEnergySource> bes);
		void addRadioModelPtr(const Ptr<DeviceEnergyModel> dem);
		double getRadioTotalEnergyConsumptionJ();
		
	private:
		// variable
		double m_initialEnergyJ;
	//	const double m_supplyVoltage;
		float_t m_const_energy;
		double m_totaldist;
		double m_totalEnergyConsumptionJ;
		double_t m_remainingEnergyJ;
		Ptr<BasicEnergySource> m_basicSourcePtr;
		Ptr<DeviceEnergyModel> m_radioModelPtr;
		
		// method
		void calcRemainingEnergyJ();
		double totalEnergyConsumptionJ();
		double totalMobilityEnergyJ();
};
#endif
