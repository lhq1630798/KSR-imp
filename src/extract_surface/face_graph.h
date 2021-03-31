#pragma once
#include <iostream>
#include <map>
#include <vector>
#include <queue>

/***************************Graph************************/
//const int MAX = 3000;

struct ENode      //邻接表的中间节点
{
	int adjvex;   //对应索引
	ENode* next;
};

struct VNode //邻接表顶点
{
	int vertex;     //值
	ENode* firstarc; //指向第一个中间节点
};

using AdjList = std::vector<VNode>;

//typedef struct VNode //邻接表顶点
//{
//	int vertex;     //值
//	ENode* firstarc; //指向第一个中间节点
//}AdjList[MAX];

class ALGraph         //图
{
private:
	AdjList adjList;          //邻接表数组
	int vexNum;              //节点数量
	int arcNum;              //连边数量
	//bool visited[MAX];        //标记被访问
	std::vector<bool> visited;        //标记被访问
	std::map<int, int> unionfield;  //标记节点属于哪个连通域

public:
	//void CreateGraph();       //创建图
	void CreateGraph(int vNum, int eNum, std::vector<std::pair<int, int>> edges);
	void PrintGraph();        //打印图
	void DFS(int v, int field); //深度优先搜索
	void BFS();               //广度优先搜索
	int DFS_findUnion();      //深度优先搜索寻找连通域
	bool isConnect(int m, int n); //判断两个节点是否连通
	std::vector<int> get_unionfield(int unionNum);//得到第unionNum个连通分量
	std::vector<std::pair<int, int> > erase_edges;
};


/*******************Graph end*******************/