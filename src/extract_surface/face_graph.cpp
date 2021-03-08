#include "face_graph.h"

void ALGraph::CreateGraph(int vNum, int eNum, std::vector<std::pair<int, int> > edges)
{
	this->vexNum = vNum;
	this->arcNum = eNum;
	for (int i = 0; i < this->vexNum; i++)  //构建顶点数组
	{
		this->adjList[i].vertex = i;
		this->adjList[i].firstarc = nullptr;
	}

	for (int i = 0; i < this->arcNum; i++)  //构建每条邻接表
	{
		int h1 = edges[i].first;
		int h2 = edges[i].second;

		ENode* temp = new ENode();
		temp->adjvex = h2;
		temp->next = this->adjList[h1].firstarc;
		this->adjList[h1].firstarc = temp;

		temp = new ENode();
		temp->adjvex = h1;
		temp->next = this->adjList[h2].firstarc;
		this->adjList[h2].firstarc = temp;
	}
}

void ALGraph::PrintGraph()
{
	for (int i = 0; i < this->vexNum; i++)
	{
		//std::cout << this->adjList[i].vertex << "--------->";
		ENode* p = this->adjList[i].firstarc;
		while (p)
		{
			//std::cout << this->adjList[p->adjvex].vertex << " ";
			p = p->next;
		}
		//std::cout << std::endl;
	}
}
void ALGraph::BFS()
{
	for (int i = 0; i < this->vexNum; i++)
	{
		visited[i] = false;
	}
	std::queue<int> q;
	for (int i = 0; i < this->vexNum; i++)
	{
		if (!visited[i])
		{
			visited[i] = true;
			q.push(i);
			//std::cout << this->adjList[i].vertex << " ";
			while (!q.empty())
			{
				int x = q.front();
				q.pop();
				ENode* p = this->adjList[x].firstarc;
				while (p)
				{
					if (!visited[p->adjvex])
					{
						visited[p->adjvex] = true;
						//std::cout << this->adjList[p->adjvex].vertex << " ";
						q.push(p->adjvex);
					}
					p = p->next;
				}
			}
		}
	}
}
//void ALGraph::DFS(int v, int field)
//{
//	visited[v] = true;
//	this->unionfield.insert(make_pair(v, field));
//	cout << this->adjList[v].vertex << " ";
//
//	ENode* p = this->adjList[v].firstarc;
//	while (p)
//	{
//		if (!visited[p->adjvex])
//			DFS(p->adjvex, field);
//		p = p->next;
//	}
//}
void ALGraph::DFS(int v, int field)
{
	visited[v] = true;
	this->unionfield.insert(std::make_pair(v, field));
	//std::cout << this->adjList[v].vertex << " ";

	ENode* p = this->adjList[v].firstarc;
	while (p)
	{
		if (!visited[p->adjvex]) {
			this->erase_edges.push_back(std::make_pair(v, p->adjvex));
			DFS(p->adjvex, field);
		}
		p = p->next;
	}
}

int ALGraph::DFS_findUnion()
{
	this->unionfield.clear();
	int count = 0;                //连通域个数
	for (int i = 0; i < this->vexNum; i++)
	{
		visited[i] = false;
	}
	//std::cout << "connected area:" << std::endl;
	for (int i = 0; i < this->vexNum; i++)
	{
		if (!visited[i])
		{
			DFS(i, ++count);
			//std::cout << std::endl;
		}

	}
	//std::cout << std::endl;
	return count;
}

std::vector<int> ALGraph::get_unionfield(int unionNum)
{
	std::vector<int> connect;
	for (std::map<int, int>::iterator it = this->unionfield.begin(); it != this->unionfield.end(); it++)
	{
		if (it->second == unionNum)
			connect.push_back(it->first);
	}
	return connect;
}

bool ALGraph::isConnect(int m, int n)
{
	return (this->unionfield[m] == this->unionfield[n]) ? true : false;
}

