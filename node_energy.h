#ifndef node_energy
#define node_energy
#include <iostream>
#include <vector>
#include <cmath>
#include "ns3/vector.h"
#include "ns3/energy-module.h"
using namespace ns3;
using namespace std;

class NodeEnergy
{
	public:
		NodeEnergy(double supplyVoltage, double initialEnergyJ);
		double RemainingEnergyJ();

	private:
		// variable
		double m_initialEnergyJ;
		double m_supplyVoltage;
		vector<double> m_remainingEnergyJ;
		vector<Ptr<BasicEnergySource>> m_basicSourcePtr;
		vector<Ptr<DeviceEnergyModel>> m_radioModelPtr;
		
		// method
		double totalEnergyConsumptionJ();
		double totalMobilityEnergyJ();	
};
#endif
