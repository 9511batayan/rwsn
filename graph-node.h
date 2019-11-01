#pragma once
#include <vector>
using namespace std;
class GraphNode
{
public:
	vector<int> to;		//どのノードとつながっているか
	vector<double> cost;	//エッジのコスト

	//ここから下はダイクストラ法のために必要な情報
	bool done;		//確定ノードかどうか
	double minCost;	//スタートノードからのコスト
	int from;		//どのノードから来たか
};
