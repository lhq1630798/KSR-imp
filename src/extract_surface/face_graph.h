#pragma once
#include <iostream>
#include <map>
#include <vector>
#include <queue>

/***************************Graph************************/
//const int MAX = 3000;

struct ENode      //�ڽӱ���м�ڵ�
{
	int adjvex;   //��Ӧ����
	ENode* next;
};

struct VNode //�ڽӱ���
{
	int vertex;     //ֵ
	ENode* firstarc; //ָ���һ���м�ڵ�
};

using AdjList = std::vector<VNode>;

//typedef struct VNode //�ڽӱ���
//{
//	int vertex;     //ֵ
//	ENode* firstarc; //ָ���һ���м�ڵ�
//}AdjList[MAX];

class ALGraph         //ͼ
{
private:
	AdjList adjList;          //�ڽӱ�����
	int vexNum;              //�ڵ�����
	int arcNum;              //��������
	//bool visited[MAX];        //��Ǳ�����
	std::vector<bool> visited;        //��Ǳ�����
	std::map<int, int> unionfield;  //��ǽڵ������ĸ���ͨ��

public:
	//void CreateGraph();       //����ͼ
	void CreateGraph(int vNum, int eNum, std::vector<std::pair<int, int>> edges);
	void PrintGraph();        //��ӡͼ
	void DFS(int v, int field); //�����������
	void BFS();               //�����������
	int DFS_findUnion();      //�����������Ѱ����ͨ��
	bool isConnect(int m, int n); //�ж������ڵ��Ƿ���ͨ
	std::vector<int> get_unionfield(int unionNum);//�õ���unionNum����ͨ����
	std::vector<std::pair<int, int> > erase_edges;
};


/*******************Graph end*******************/