#ifndef NODEENERGY_H
#define NODEENERGY_H
#include "ns3/energy-module.h"
using namespace ns3;
class NodeEnergy
{
	public:
		NodeEnergy(const double supplyVoltage, const double initialEnergyJ);		// constructor
		double getRemainingEnergyJ();
		void sumDist(const double dist);
		void addBasicSourcePtr(const Ptr<BasicEnergySource> bes);
		void addRadioModelPtr(const Ptr<DeviceEnergyModel> dem);
		double getRadioTotalEnergyConsumptionJ();
		void updateTotalEnergyConsumptionJ(const double currentA, const double time);
		
	private:
		// variable
		double m_initialEnergyJ;
		double m_supplyVoltage;
		double m_currentA;
		float_t m_const_energy;
		double m_totaldist;
		double m_totaltime;
		double m_totalEnergyConsumptionJ;
		double m_sensorEnergyJ;
		Ptr<BasicEnergySource> m_basicSourcePtr;
		Ptr<DeviceEnergyModel> m_radioModelPtr;
		
		// method
		void totalEnergyConsumptionJ();
		double totalMobilityEnergyJ();
};
#endif
