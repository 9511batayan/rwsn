#ifndef pathplanning
#define pathplanning
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <string>
#include <stack>
#include <algorithm>
#include "graph-node.h"
#include "ns3/vector.h"
using namespace ns3;
using namespace std;
class GraphSearch
{
public:
	GraphSearch();
	~GraphSearch();
	void pathPlanning(Vector pos1, Vector pos2);
	Vector popPathPosition();
	double costMinPath()const;
	void clearGraphInfo();
	static double calcRxPower(Vector sta, Vector end);
	static double calcDistance(Vector sta, Vector end);

private:
	map<int, Vector> m_nodePos;
	int m_numNodes = 6;
	const int maxsize_node;
	const double INF = 1e5;
	vector<GraphNode> m_node;
	stack<Vector> m_path;
	int m_nodePos_tail;
	Vector m_sta, m_goal;
	int m_startPos_id, m_goalPos_id;

	void addPos(Vector p);
	void connectEdge();
	void addEdge(const int v, const int u, const double weight);
	void dijkstra(int start, int end);
	void pushPathPosition();
};
#endif
