#pragma once
#include <vector>
using namespace std;
class GraphNode
{
public:
	vector<int> to;		//�ǂ̃m�[�h�ƂȂ����Ă��邩
	vector<double> cost;	//�G�b�W�̃R�X�g

	//�������牺�̓_�C�N�X�g���@�̂��߂ɕK�v�ȏ��
	bool done;		//�m��m�[�h���ǂ���
	double minCost;	//�X�^�[�g�m�[�h����̃R�X�g
	int from;		//�ǂ̃m�[�h���痈����
};
