#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-phy.h"
#include "ns3/internet-module.h"
#include "ns3/energy-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/node-list.h"
#include "ns3/propagation-loss-model.h"
#include "node-energy.h"
#include "path-planning.h"
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <queue>
using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE("rwsn");

string rssi_file ="rssi-log.csv";
string energy_file = "remaining_energy-log.csv";
string position_file = "position_node-log.csv";
std::ofstream rssi_ofs(rssi_file);
std::ofstream energy_ofs(energy_file);
std::ofstream pos_ofs(position_file);

const int numNodes = 5;
const int source = numNodes-1;

inline double 
calcRad_phi(Vector a,Vector b)
{       
        return atan2(b.y - a.y, b.x - a.x);
}
inline double_t
calcRad_theta(Vector a, Vector b)
{
	return atan2(sqrt((b.x-a.x) * (b.x-a.x) + (b.y-a.y) * (b.y-a.y)), b.z - a.z);
}
double_t
calcDistance(Vector a, Vector b)
{
	return sqrt((b.x-a.x) * (b.x-a.x) + (b.y-a.y) * (b.y-a.y) + (b.z-a.z) * (b.z-a.z));
}
const double txPowerDbm = 16;
double_t
propagationLoss(Ptr<PropagationLossModel> model, const double txPowerDbm, const int id, const int id1)
{
	Ptr<Node> n1 = NodeList::GetNode(id);
	Ptr<Node> n2 = NodeList::GetNode(id1);
	Ptr<MobilityModel> a = n1->GetObject<MobilityModel>();
	Ptr<MobilityModel> b = n2->GetObject<MobilityModel>();
	return model->CalcRxPower(txPowerDbm, a, b);
}

vector<float_t> rssi(source,0.0);
void 
CalcSignalLevel()
{
	// create code of estimation value without noise
	rssi_ofs<<Simulator::Now().GetSeconds()<< ",";
	for(int i=0;i<source; ++i){
		Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
		Ptr<NakagamiPropagationLossModel> nak = CreateObject<NakagamiPropagationLossModel>();
		double rxPowerDbm = propagationLoss(friis, txPowerDbm, i, i+1);
		rxPowerDbm = propagationLoss(nak, rxPowerDbm, i, i+1);
		rssi[i] = rxPowerDbm;
		rssi_ofs<<rssi[i]<<",";
	}
	rssi_ofs<<"\n";
	Simulator::Schedule(Seconds(0.2),&CalcSignalLevel);
}

inline void 
SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  mob->SetPosition (position);
}
inline Vector 
GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  return mob->GetPosition ();
}

const int msn = numNodes - 2;
vector<queue<Vector>> graph_queue(2*msn);	//idx 0,1:MSN1, idx 2,3:MSN2, idx 4,5:MSN3
void 
GlobalPathPlanning()
{
	const int q_size = graph_queue.size();
	// clear graph queue
	for(int i=0; i< q_size;i++){
		while(!graph_queue[i].empty()){
			graph_queue[i].pop();
		}
	}
	int q_id = 0;
	for(int id = 1; id <= msn; id++){
		int t_id = id - 1;
		for(int cnt = 0; cnt < 2; cnt++){
			GraphSearch gs = GraphSearch();
			Vector t_pos = GetPosition(NodeList::GetNode(t_id));
			Vector cur_pos = GetPosition(NodeList::GetNode(id));
			gs.pathPlanning(cur_pos, t_pos);
			while (1) {
				Vector pos = gs.popPathPosition();
				graph_queue[q_id].push(pos);
				if (pos.x == t_pos.x && pos.y == t_pos.y && pos.z == pos.z) break;
			}
			t_id+=2;
			q_id++;
		}
	}
	Simulator::Schedule(Seconds(10.0),&GlobalPathPlanning);
}

double speed = 0.2;		// [m/s]
Vector 
UpdatePosition(Vector cur, const Vector tar)
{
	if(calcDistance(cur, tar) <= speed){
		cur = tar;
	}else{
		double theta = calcRad_theta(cur, tar);
		double phi = calcRad_phi(cur, tar);
		/*
		 * 2-d movement update formula
		 * cur.x += speed*cos(phi)*moving_cycle;
		 * cur.y += speed*sin(phi)*moving_cycle;
		 * */		
		cur.x += speed*sin(theta)*cos(phi)*moving_cycle;
		cur.y += speed*sin(theta)*sin(phi)*moving_cycle;
		cur.z += speed*cos(theta)*1;
	}
	if(cur.x < 0) cur.x = 0;
	if(cur.y < 0) cur.y = 0;
	return cur;
}
float EvalUniformEnergy(double cur_remEngy, double tar_remEngy)
{
	//評価値が大きいほど、両者の量の差も大きくなる
	/*
	 * cur_remEngy = 100, tar_remEngy = 0 => abs(100 - 0) / max(100,0) = 1
	 * cur_remEngy = 100, tar_remEngy = 100 => abs(100 - 100) / max(100,100) = 0
	 */
	return abs(cur_remEngy - tar_remEngy) / max(cur_remEngy, tar_remEngy);
}

vector<NodeEnergy> nodeEnergy;
int RetId_NodeToMove(const int id, const int tar_id)
{
	static const float_t eval_val_th = 0.040;
	if(tar_id == 0 || tar_id == source) return tar_id;
	double cur_remE = nodeEnergy[id].getRemainingEnergyJ();
	double tar_remE = nodeEnergy[tar_id].getRemainingEnergyJ();
	float_t eval_val = EvalUniformEnergy(cur_remE, tar_remE);
	if(eval_val > eval_val_th){
		cout<< "eval_val is evaluated"<<endl;
		if(cur_remE > tar_remE) return tar_id;
		else return id;
	}
	else {
		cout<<"Not evaluated"<<endl;
		return tar_id;
	}
}

const double moving_cycle = 1.0;
const int subtgt=3;		// number of subtgt = number of graph node
const Vector waypoint[subtgt] = {{0,0,0},{15,0.2,-10},{15,30,-10}};
// deployment method function
void DeploymentNode()
{
	pos_ofs << Simulator::Now().GetSeconds()<< ",";
	energy_ofs << Simulator::Now().GetSeconds()<< ",";
	for(int id = 1; id < numNodes; ++id)
	{
		Vector cur = GetPosition(NodeList::GetNode(id));
		Vector last_dist = {cur.x, cur.y, cur.z};
		if(id == source)
		{
			static short int waypoint_id = 0;
			cur = UpdatePosition(cur, waypoint[waypoint_id]);
			if(waypoint_id != subtgt-1 &&
				cur.x == waypoint[waypoint_id].x && 
				cur.y == waypoint[waypoint_id].y) { waypoint_id++; }
		}else{
	/* 
	 * MSN or sink[id-1] --- rssi[id-1] --- MSN[id] ---- rssi[id] ---- MSN or Leader[id+1]
	 */
			static const double rssi_th = -50;
			if(rssi[id] < rssi_th || rssi[id-1] < rssi_th)
			{
				/*MSNの属する通信経路で一番RSSIが低い方向に移動*/
				int t_id;
				int graph_queue_idx;	//graph_queue_idx {0,1:MSN1, 2,3:MSN2, 4,5:MSN3}				
				if(rssi[id] < rssi[id - 1]) {
					t_id = id + 1;
					graph_queue_idx = 2 * id - 1;
				}else {
					t_id = id - 1;
					if (id == 1) graph_queue_idx = 0;
					else if (id == 2) graph_queue_idx = 2;
					else if (id == 3) graph_queue_idx = 4;
				}
				t_id = RetId_NodeToMove(id, t_id);
				if(id != t_id) {
					Vector goal;
					if(graph_queue[graph_queue_idx].empty())
					{
						goal = GetPosition(NodeList::GetNode(t_id));
					}else {
						goal = graph_queue[graph_queue_idx].front();
					}
					cur = UpdatePosition(cur, goal);
					if(cur.x==goal.x && cur.y==goal.y && cur.z==goal.z) { graph_queue[graph_queue_idx].pop(); }
				}
			}
		}
		nodeEnergy[id].updateTotalEnergyConsumptionJ(Simulator::Now().GetSeconds(), calcDistance(last_dist, cur));
		pos_ofs << cur.x<<","<<cur.y<<","<<cur.z<<",";
		energy_ofs<<nodeEnergy[id].getRemainingEnergyJ()<<",";
		SetPosition(NodeList::GetNode(id), cur);
		
		printf("Time = %.2f[s] ID=%d pos : x= %.4f y=%.4f z=%.4f\n",Simulator::Now().GetSeconds(), id, cur.x, cur.y, cur.z);
	}
	pos_ofs<<"\n";
	energy_ofs<<"\n";
	Simulator::Schedule(Seconds(moving_cycle), &DeploymentNode);
}
int main (int argc, char *argv[])
{
	// Ieee802.11g phyMode 36
	std::string phyMode = "ErpOfdmRate36Mbps";
	string dateRate_kbps = "100Kib/s";
	int packetSize_byte = 512;
	bool verbose = false;
	bool animTracing = false;
	bool pcapTracing = false;
	double time_s = 1.0;
	string select_tcpip = "udp";
	
	CommandLine cmd;
	cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
	cmd.AddValue("speed","speed of moblity node",speed);
	cmd.AddValue("time","end of simulator time",time_s);
	cmd.AddValue("packetSize_byte","Packet size of OnOffApplication",packetSize_byte);
	cmd.AddValue("dateRate_kbps","Data rate of OnOffApplication",dateRate_kbps);
	cmd.AddValue("pcapTracing","Whether to generation of pcap file",pcapTracing);	
	cmd.AddValue("animTracing","Whether to allow generation of xml file for animation",animTracing);
	cmd.AddValue("tcp/udp","Is transport protocol TCP or UDP ?",select_tcpip);
	cmd.Parse (argc, argv);
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",  StringValue (phyMode));
	
	Time time = Seconds(time_s);

	NodeContainer node;
	node.Create (numNodes);
	
	WifiHelper wifi;
	if (verbose) wifi.EnableLogComponents ();
	wifi.SetStandard (WIFI_PHY_STANDARD_80211g);
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                               "ControlMode",StringValue (phyMode));
	YansWifiPhyHelper wifiPhy;
	YansWifiChannelHelper wifiChannel;
	wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
//	wifiChannel.SetPropagationDelay("ns3::RandomPropagationDelayModel","Variable",StringValue("ns3::UniformRandomVariable"));
	wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel"); //wifi;ErrorRateModel CSMA/p2p:RateErrorModel
	wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel","Frequency",DoubleValue(2.4e9));
	wifiPhy.SetChannel (wifiChannel.Create ());
	wifiPhy.Set ("TxGain", DoubleValue (0) );	//　Gain=0 <--等放射の利得
	wifiPhy.Set("RxGain",DoubleValue(0));
	wifiPhy.Set("ChannelWidth",UintegerValue(22));
	wifiPhy.Set("ChannelNumber",UintegerValue(6));
	wifiPhy.Set ("TxPowerStart", DoubleValue (txPowerDbm));
	wifiPhy.Set ("TxPowerEnd", DoubleValue (txPowerDbm));
	wifiPhy.Set("RxNoiseFigure",DoubleValue(7));
	
	WifiMacHelper wifiMac;
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                               "ControlMode",StringValue (phyMode));
	wifiMac.SetType ("ns3::AdhocWifiMac");
	NetDeviceContainer devices=wifi.Install(wifiPhy,wifiMac,node);

	InternetStackHelper internet;
	internet.Install(node);

	MobilityHelper mobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
	positionAlloc->Add (Vector (-0.8, -0.8, 0.0));
	positionAlloc->Add (Vector (-0.6, -0.6, 0.0));
	positionAlloc->Add (Vector (-0.4, -0.4, 0.0));
	positionAlloc->Add (Vector (-0.2, 0.2, 0.0));
	positionAlloc->Add (Vector (0.0, 0.0, 0.0));
	mobility.SetPositionAllocator (positionAlloc);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.InstallAll();

	Ipv4AddressHelper ipv4;
	NS_LOG_INFO ("Assign IP Addresses.");
	ipv4.SetBase ("192.168.127.0", "255.255.255.0");
	Ipv4InterfaceContainer i = ipv4.Assign (devices);

/**Application**/
/********************************************************************************************************************/
	Ipv4StaticRoutingHelper staticRoutingHelper;
	Ptr<Ipv4> ipv4_node0 = node.Get(0) -> GetObject<Ipv4>();
	Ptr<Ipv4> ipv4_node1 = node.Get(1) -> GetObject<Ipv4>();
	Ptr<Ipv4> ipv4_node2 = node.Get(2) -> GetObject<Ipv4>();
	Ptr<Ipv4> ipv4_node3 = node.Get(3) -> GetObject<Ipv4>();
	Ptr<Ipv4> ipv4_node4 = node.Get(4) -> GetObject<Ipv4>();

	Ptr<Ipv4StaticRouting> staticRouting_node0 = staticRoutingHelper.GetStaticRouting(ipv4_node0);		
	Ptr<Ipv4StaticRouting> staticRouting_node1 = staticRoutingHelper.GetStaticRouting(ipv4_node1);
	Ptr<Ipv4StaticRouting> staticRouting_node2 = staticRoutingHelper.GetStaticRouting(ipv4_node2);
	Ptr<Ipv4StaticRouting> staticRouting_node3 = staticRoutingHelper.GetStaticRouting(ipv4_node3);
	Ptr<Ipv4StaticRouting> staticRouting_node4 = staticRoutingHelper.GetStaticRouting(ipv4_node4);

	staticRouting_node0 -> AddHostRouteTo(Ipv4Address("192.168.127.1"), Ipv4Address("192.168.127.1"),0);
	staticRouting_node1 -> AddHostRouteTo(Ipv4Address("192.168.127.1"), Ipv4Address("192.168.127.1"),1);
	staticRouting_node2 -> AddHostRouteTo(Ipv4Address("192.168.127.1"), Ipv4Address("192.168.127.2"),1);
	staticRouting_node3 -> AddHostRouteTo(Ipv4Address("192.168.127.1"), Ipv4Address("192.168.127.3"),1);
	staticRouting_node4 -> AddHostRouteTo(Ipv4Address("192.168.127.1"), Ipv4Address("192.168.127.4"),1);

	uint16_t port = 9;
	string prt;
	if (select_tcpip=="udp"){prt = "ns3::UdpSocketFactory";}
	else{prt="ns3::TcpSocketFactory";}
	OnOffHelper onoff (prt,
                     Address (InetSocketAddress (i.GetAddress(0), port)));
	onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
	onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
	onoff.SetConstantRate (DataRate (dateRate_kbps),packetSize_byte);
	ApplicationContainer apps = onoff.Install (node.Get(source));
	apps.Start (Seconds (0.05));
	apps.Stop (time);

	PacketSinkHelper sink (prt,
                         Address (InetSocketAddress (Ipv4Address::GetAny(), port)));
	apps = sink.Install (node.Get(0));
	apps.Start (Seconds (0.0));
	apps.Stop (time);
/*******************************Fin Application Setting*******************************************************************/
/********************************Energy Setting***************************************************************************/
	BasicEnergySourceHelper basicSource;
	double supplyVoltage = 5.0, initialEnergyJ = 1e5;
	basicSource.Set("BasicEnergySourceInitialEnergyJ",DoubleValue(initialEnergyJ));
	basicSource.Set("BasicEnergySupplyVoltageV",DoubleValue(supplyVoltage));
	EnergySourceContainer sources = basicSource.Install (node);
	WifiRadioEnergyModelHelper radioEnergy;
	radioEnergy.Set ("IdleCurrentA", DoubleValue (0.6));
	radioEnergy.Set ("TxCurrentA", DoubleValue (0.85));
	radioEnergy.Set ("RxCurrentA", DoubleValue (0.65));
	DeviceEnergyModelContainer deviceModels = radioEnergy.Install (devices, sources);
	const double moterVoltage = 15.0; const double moterCurrentA = 3.32;
	for(int i = 0; i < numNodes; i++){
		nodeEnergy.push_back(NodeEnergy(moterVoltage, moterCurrentA, initialEnergyJ, speed));
		Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (i));
		nodeEnergy[i].addBasicSourcePtr(basicSourcePtr);
		nodeEnergy[i].addRadioModelPtr(basicSourcePtr->FindDeviceEnergyModels("ns3::WifiRadioEnergyModel").Get(0));
	}
/******************************************Fin Energy Setting*********************************************/
	Simulator::Schedule(Seconds(moving_cycle), &DeploymentNode);
	Simulator::Schedule(Seconds(0.2), &CalcSignalLevel);
	Simulator::Schedule(Seconds(10.0), &GlobalPathPlanning);
	if(animTracing) {
		AnimationInterface anim ("scratch/rwsn/rwsn.xml");
	}
	if(pcapTracing) wifiPhy.EnablePcap("scratch/rwsn/nodeinfo",devices);

//	Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx0));
//	Config::ConnectWithoutContext ("/NodeList/1/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx1));
//	Config::ConnectWithoutContext ("/NodeList/2/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx2));

	Simulator::Stop(time);
	Simulator::Run ();
	/* Mueasurement throughput End-End
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll();
	monitor->CheckForLostPackets();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());
	FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin(); iter != stats.end(); ++iter) {
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
		NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
		NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
		NS_LOG_UNCOND("Tx Bytes:" << iter->second.txBytes) ;
		NS_LOG_UNCOND("Tx Offered:" << iter->second.txBytes * 8.0 / time.GetSeconds() / 1024 / 1024  << " Mbps");
		NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);
		NS_LOG_UNCOND("Rx Bytes: " << iter->second.rxBytes);
		NS_LOG_UNCOND("Throughput: " << iter->second.rxBytes * 8.0 / time.GetSeconds() / 1024 /1024<< " Mbps");
		NS_LOG_UNCOND("Packet loss= " << ((iter->second.txPackets - iter->second.rxPackets)*1.0) / iter->second.txPackets);
	}
	*/
	cout<<"Finish time[s] : "<<Simulator::Now().GetSeconds()<<endl;
	Simulator::Destroy ();
	rssi_ofs.close();
	energy_ofs.close();
	pos_ofs.close();
	return 0;
}
