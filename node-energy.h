#ifndef NODEENERGY_H
#define NODEENERGY_H
#include "ns3/energy-module.h"
using namespace ns3;
class NodeEnergy
{
	public:
		NodeEnergy(const double supplyVoltage, const double currentA, const double initialEnergyJ, const double vel);		// constructor
		double getRemainingEnergyJ();
		double getTotalEnergyConsumptionJ() const;
		double getRadioTotalEnergyConsumptionJ() const;
		void updateTotalEnergyConsumptionJ(const double time, const double diff_dist);
		void addBasicSourcePtr(const Ptr<BasicEnergySource> bes);
		void addRadioModelPtr(const Ptr<DeviceEnergyModel> dem);
		
	private:
		// variable
		const double m_initialEnergyJ;
		double m_const_mobEnergy;
		double m_diff_dist;
		double m_diff_time;
		double m_totalEnergyConsumptionJ;
		double m_radioTotalEnergyConsumptionJ;
		double m_mobTotalEnergyConsumptionJ;
		double m_sensorTotalEnergyConsumptionJ;
		Ptr<BasicEnergySource> m_basicSourcePtr;
		Ptr<DeviceEnergyModel> m_radioModelPtr;
		
		// method
		double updateTotalRadioEnergyConsumptionJ();
		double updateTotalSensorEnergyConsumptionJ();
		double updateTotalMobilityEnergyConsumptionJ();
		void totalEnergyConsumptionJ();
};
#endif
