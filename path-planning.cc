/*
* グラフ構造を構築および経路計画するクラス
* 今後構築と経路計画で別モジュールに変更予定
* スタートとゴール座標だけ渡して最短経路のノードを返す
*/
#include "path-planning.h"
using namespace std;
//public
GraphSearch::GraphSearch() :maxsize_node(8)
{
	m_node.resize(maxsize_node);
	/*
	* pattern1
	*
	*	0 ---- 1 ---- 2
	*
	*	       |
	*          |
	*
	*	5 ---- 3 ---- 4
	*
	*/
	addEdge(1, 0, 61.0291);
	addEdge(1, 2, 66.5648);
	addEdge(1, 3, 73.1197);
	addEdge(3, 4, 66.5648);
	addEdge(3, 5, 61.0291);

	m_nodePos ={
	{ 0, {5,5,0} },
	{ 1, {22,5,0} },
	{ 2, {48,5,0} },
	{ 3, {22,48,0} },
	{ 4, {48,48,0} },
	{ 5, {5,48,0} }
	};
	
	// network topology is line
	/*
	addEdge(1, 0, 73.1197);
	m_nodePos =
    {
	{0, {5,5,0} },
    {1, {48,5,0}}
    };
	*/
	
	// network topology is right angle
	/*
	addEdge(1, 0, 73.1197);
	addEdge(1, 2, 73.1197);
	m_nodePos =
    {
	{0, {5,5,0} },
    {1, {48,5,0}},
    {2, {48,48,0}}
    };
	*/
	m_nodePos_tail = m_nodePos.size() - 1;
}
GraphSearch::~GraphSearch()
{

}
void 
GraphSearch::pathPlanning(Vector sta, Vector goal)
{
	m_sta = sta; m_goal = goal;
	addPos(m_sta);
	m_startPos_id = m_nodePos_tail;
	addPos(m_goal);
	m_goalPos_id = m_nodePos_tail;
	connectEdge();
	dijkstra(m_startPos_id, m_goalPos_id);
	pushPathPosition();
}
Vector
GraphSearch::popPathPosition()
{
	Vector p = m_path.top();
	m_path.pop();
	return p;
}
double
GraphSearch::costMinPath()const
{
	return m_node[m_nodePos.size() - 1].minCost;
}
void
GraphSearch::clearGraphInfo()
{
}
double
GraphSearch::calcRxPower(Vector sta, Vector end)
{
	static const double txPowerDbm = 16;
	static const double refDist = 1;
	static const double exponent = 3.0;
	static const double refLoss = 40.1156;
	double dist = calcDistance(sta, end);
	double pathLossDb = 10 * exponent*log10(dist / refDist);
	double rxc = -refLoss - pathLossDb;
	return abs(txPowerDbm + rxc);
}
inline double
GraphSearch::calcDistance(Vector sta, Vector end)
{
	return sqrt(pow(abs(sta.x - end.x), 2) + pow(abs(sta.y - end.y), 2) + pow(abs(sta.z - end.z), 2));
}

//private
void
GraphSearch::addPos(Vector p)
{
	m_nodePos_tail++;
	m_nodePos[m_nodePos_tail] = p;
}
void
GraphSearch::connectEdge() {
	vector<double> dist;
	for (int i = 0;i < (int)m_numNodes;++i)  
		dist.push_back(calcRxPower(m_nodePos[i], m_nodePos[m_startPos_id]));
	/*
	 * Connect the node with minimum distance cost and second distance cost
	 * from start node to each node
	 */
	for(int i = 0; i < 2; ++i)
	{
		auto min_itr = min_element(dist.begin(), dist.end());
		size_t min_idx = distance(dist.begin(), min_itr);
		addEdge(min_idx, m_startPos_id, dist[min_idx]);
		dist[min_idx] = INF;
	}
	
	dist.clear();
	
	for (int i = 0; i < (int)m_numNodes; ++i)
		dist.push_back(calcRxPower(m_nodePos[i], m_nodePos[m_goalPos_id]));
	/*
	 * Connect the node with minimum distance cost and second distance cost
	 * from goal node to each node
	 */
	for(int i = 0; i < 2; ++i)
	{
		auto min_itr = min_element(dist.begin(), dist.end());
		size_t min_idx = distance(dist.begin(), min_itr);
		addEdge(min_idx, m_goalPos_id, dist[min_idx]);
		dist[min_idx] = INF;
	}
}
void
GraphSearch::addEdge(const int v, const int u, const double weight)
{
	//ノードuはノードvとつながっている情報を入れる	
	m_node[u].to.push_back(v);
	//ノードuとノードvのエッジの重みを入れる
	m_node[u].cost.push_back(weight);

	//有向グラフならここから下の処理が不要

	//ノードvはノードuとつながっている情報を入れる
	m_node[v].to.push_back(u);
	//ノードvとノードuのエッジの重みを入れる
	m_node[v].cost.push_back(weight);
}
void
GraphSearch::dijkstra(int start, int end)
{
	//変数の初期化
	for (int i = 0; i < maxsize_node; i++) {
		m_node[i].done = false;
		m_node[i].minCost = -1;
	}
	m_node[start].minCost = 0;	//スタートノードまでのコストは0
	while (1) {
		int doneNode = -1;
		//最新の確定したノード番号(-1はNULLのかわり)
		for (int i = 0; i < maxsize_node; i++) {

			if (m_node[i].done == true)
			{//ノードiが確定しているときcontinue
				continue;
			}
			if (m_node[i].minCost < 0)
			{//ノードiまでの現時点での最小コストが不明のとき
				continue;
			}

			//確定したノード番号が-1かノードiの現時点の最小コストが小さいとき
			//確定ノード番号を更新する
			if (doneNode < 0 || 
				m_node[i].minCost < m_node[doneNode].minCost) {
				doneNode = i;
			}
		}

		if (doneNode == -1) break;
		//すべてのノードが確定したら終了

		m_node[doneNode].done = true;	//ノードを確定させる

		for (int i = 0; i < (int)m_node[doneNode].to.size(); i++) {
			int to = m_node[doneNode].to[i];
			double cost = m_node[doneNode].minCost + 
				m_node[doneNode].cost[i];

			//ノードtoはまだ訪れていないノード
			//またはノードtoへより小さいコストの経路だったら
			//ノードtoの最小コストを更新
			if (m_node[to].minCost < 0 || 
				 m_node[to].minCost > cost) {
				m_node[to].minCost = cost;
				m_node[to].from = doneNode;
			}
		}
	}
}
void
GraphSearch::pushPathPosition()
{
	for (int i = m_goalPos_id;i != m_startPos_id;i = m_node[i].from) {
		Vector p = m_nodePos[i];
		m_path.push(p);
	}
}
