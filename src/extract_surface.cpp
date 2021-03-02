
#include "extract_surface.h"
#include "fmt/core.h"
//#include "log.h"

/***************************Graph************************/
const int MAX = 3000;

struct ENode      //邻接表的中间节点
{
	int adjvex;   //对应索引
	ENode* next;
};

typedef struct VNode //邻接表顶点
{
	int vertex;     //值
	ENode* firstarc; //指向第一个中间节点
}AdjList[MAX];

class ALGraph         //图
{
private:
	AdjList adjList;          //邻接表数组
	int vexNum;              //节点数量
	int arcNum;              //连边数量
	bool visited[MAX];        //标记被访问
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
	std::cout << this->adjList[v].vertex << " ";

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
	std::cout << "connected area:" << std::endl;
	for (int i = 0; i < this->vexNum; i++)
	{
		if (!visited[i])
		{
			DFS(i, ++count);
			std::cout << std::endl;
		}

	}
	std::cout << std::endl;
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


/*******************Graph end*******************/



bool operator<(const EdgeKey& e1, const EdgeKey& e2) {
	if (e1.edge.first != e2.edge.first) {
		return e1.edge.first < e2.edge.first;
	}
	else {
		return e1.edge.second < e2.edge.second;
	}
}

bool operator<(const NeighborKey& n1, const NeighborKey& n2) {
	if (n1.neighbors.first != n2.neighbors.first) {
		return n1.neighbors.first < n2.neighbors.first;
	}
	else {
		return n1.neighbors.second < n2.neighbors.second;
	}
}

//n vertices and area of this face
Dart_handle make_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane)
{
	//多边形的顶点
	auto id_points2 = polygon.id_polygon_2();

	std::vector<vertex_attributes> id_points;
	//Points_3 points;
	for (int i = 0; i < id_points2.size(); i++) {
		vertex_attributes v_att = { plane.to_3d(id_points2[i].pos_2), id_points2[i].ID };
		id_points.push_back(v_att);
	}
	PWN_vector inline_points = polygon.inline_points;//属于多边形内部的输入点
	FT area = polygon.area();
	Direction_3 normal = cross_product((id_points[1].pos_3 - id_points[0].pos_3), (id_points[2].pos_3 - id_points[1].pos_3)).direction();
	//bool removed = false;
	face_attributes face_att = { area,plane,inline_points };

	std::vector<Dart_handle> darts1;
	//beta1:first half-face
	for (int i = 0; i < id_points.size(); i++)
	{
		Dart_handle d = amap.create_dart();
		darts1.push_back(d);
		if (amap.attribute<0>(d) == NULL)amap.set_attribute<0>(d, amap.create_attribute<0>());
		if (amap.attribute<2>(d) == NULL)amap.set_attribute<2>(d, amap.create_attribute<2>());
		if (amap.attribute<3>(d) == NULL)amap.set_attribute<3>(d, amap.create_attribute<3>());
		amap.info<0>(d) = id_points[i];
		amap.info(d) = std::make_pair(normal,false);
	}
	for (int i = 0; i < id_points.size() - 1; i++) {
		amap.link_beta<1>(darts1[i], darts1[i + 1]);
	}
	amap.link_beta<1>(darts1[id_points.size() - 1], darts1[0]);
	amap.info<2>(darts1[0]) = face_att;

	return darts1[0];
}

Dart_handle make_twins_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane, bool is_ghost)
{
	//多边形的顶点
	auto id_points2 = polygon.id_polygon_2();

	std::vector<vertex_attributes> id_points;
	//Points_3 points;
	for (int i = 0; i < id_points2.size(); i++) {
		vertex_attributes v_att = { plane.to_3d(id_points2[i].pos_2), id_points2[i].ID };
		id_points.push_back(v_att);
	}
	PWN_vector inline_points = polygon.inline_points;//属于多边形内部的输入点
	FT area = polygon.area();
	Direction_3 normal = cross_product((id_points[1].pos_3 - id_points[2].pos_3), (id_points[0].pos_3 - id_points[1].pos_3)).direction();
	//bool removed = false;
	face_attributes face_att = { area,plane,inline_points };

	//beta1:second half-face
	std::vector<Dart_handle> darts2;
	for (int i = 0; i < id_points.size(); i++)
	{
		Dart_handle d = amap.create_dart();
		darts2.push_back(d);
		if (amap.attribute<0>(d) == NULL)amap.set_attribute<0>(d, amap.create_attribute<0>());
		if (amap.attribute<2>(d) == NULL)amap.set_attribute<2>(d, amap.create_attribute<2>());
		if (amap.attribute<3>(d) == NULL)amap.set_attribute<3>(d, amap.create_attribute<3>());
		amap.info<0>(d) = id_points[id_points.size() - 1 - i];
		amap.info(d) = std::make_pair(normal,is_ghost);
	}
	for (int i = 0; i < id_points.size() - 1; i++) {
		amap.link_beta<1>(darts2[i], darts2[i + 1]);
	}
	amap.link_beta<1>(darts2[id_points.size() - 1], darts2[0]);
	amap.info<2>(darts2[0]) = face_att;

	return darts2[0];
}


void build_edge_map(CMap_3& cm, Dart_handle it, Edge2darts& darts_per_edge, vertex_attributes p1, vertex_attributes p2) {

	EdgeKey edge1(std::make_pair(p1.ID, p2.ID));
	Edge2darts::iterator ite = darts_per_edge.find(edge1);
	if (ite != darts_per_edge.end()) {
		Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
		Point_3 normal = p1.pos_3 + cm.info(it).first.vector();
		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
		//std::cout << plane << " normal"<<normal <<" direction"<< direction2 << std::endl;

		Darts darts = ite->second;
		int n = darts.size();
		int i;
		if (n < 2) {
			darts.push_back(std::make_pair(direction2, it));
			darts_per_edge[ite->first] = darts;
		}
		else {
			for (i = 0; i < n - 1; i++) {
				if (direction2.counterclockwise_in_between(darts[i].first, darts[i + 1].first)) {
					darts.insert(darts.begin() + i + 1, std::make_pair(direction2, it));
					darts_per_edge[ite->first] = darts;
					break;
				}
			}
			if (i == n - 1) {
				darts.push_back(std::make_pair(direction2, it));
				darts_per_edge[ite->first] = darts;
			}
		}
	}
	else {
		Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
		Point_3 normal = p1.pos_3 + cm.info(it).first.vector();
		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
		//std::cout << plane << " normal" << normal << " direction" << direction2 << std::endl;

		Darts darts;
		darts.push_back(std::make_pair(direction2, it));
		darts_per_edge[edge1] = darts;
	}
}

void build_map(CMap_3& cm, KPolygons_SET& polygons_set)
{
	size_type amark;
	try
	{
		amark = cm.get_new_mark();
	}
	catch (CMap_3::Exception_no_more_available_mark)
	{
		std::cerr << "No more free mark, exit." << std::endl;
		exit(-1);
	}

	//int polynum = 0;
	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != polygons_set._kpolygons_set.end(); polygons++) {
		Plane_3 plane = polygons->plane();
		//std::cout << plane << std::endl;
		bool is_ghost = polygons->is_bbox;
		for (auto &kpoly : polygons->_kpolygons_2) {
			Dart_handle d1 = make_polygon(cm, kpoly, plane);
			Dart_handle d2 = make_twins_polygon(cm, kpoly, plane, is_ghost);
			//polynum++;
			cm.sew<3>(d1, d2);
			//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
		}
	}
	//std::cout << polynum << std::endl;
	cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;

	//build edge-darts table
	Edge2darts darts_per_edge;
	for (CMap_3::Dart_range::iterator it(cm.darts().begin()), itend(cm.darts().end()); it != itend; it++) {
		if (cm.is_marked(it, amark))
			continue;

		auto p1 = cm.info_of_attribute<0>(cm.attribute<0>(it));
		auto p2 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it, 1)));
		//std::cout << p1.ID[0] << p1.ID[1] << p1.ID[2] << " " << p2.ID[0] << p2.ID[1] << p2.ID[2] << std::endl;

		if (p1.ID < p2.ID) {
			build_edge_map(cm, it, darts_per_edge, p1, p2);
		}
		else {
			build_edge_map(cm, cm.beta(it, 3), darts_per_edge, p2, p1);
		}

		cm.mark(it, amark);
		cm.mark(cm.beta(it, 3), amark);
	}
	cm.unmark_all(amark);

	/*Edge2darts::iterator it = darts_per_edge.begin();
	Edge2darts::iterator itEnd = darts_per_edge.end();
	while (it != itEnd) {
		std::cout << "edge:"<< it->first.edge.first[0] << it->first.edge.first[1] << it->first.edge.first[2] << " " << it->first.edge.second[0] << it->first.edge.second[1] << it->first.edge.second[2] << std::endl;
		std::cout << "direction:" << std::endl;
		if (it->second.size() == 1) {
			std::cout << cm.info_of_attribute<0>(cm.attribute<0>(it->second[0].second)).pos_3 << std::endl;
		}
		for (int i = 0; i < it->second.size(); i++) {
			std::cout << it->second[i].first << " " << cm.info(it->second[i].second) << std::endl;
		}
		std::cout<<std::endl;
		it++;
	}*/

	//beta2
	Edge2darts::iterator it = darts_per_edge.begin();
	Edge2darts::iterator itEnd = darts_per_edge.end();
	//it = darts_per_edge.begin();
	//itEnd = darts_per_edge.end();
	while (it != itEnd) {
		Darts darts = it->second;
		for (int i = 0; i < darts.size() - 1; i++) {
			cm.sew<2>(darts[i].second, cm.beta(darts[i + 1].second, 3));
		}
		cm.sew<2>(darts[darts.size() - 1].second, cm.beta(darts[0].second, 3));
		it++;
	}

	//polyhedra_attribute
	int num = 0;
	for (CMap_3::One_dart_per_cell_range<3>::iterator it(cm.one_dart_per_cell<3>().begin()), itend(cm.one_dart_per_cell<3>().end()); it != itend; it++) {

		Point_3 center;
		Vector_3 center_V = CGAL::NULL_VECTOR;
		int size = 0;
		//找到it所在的polyhedra的所有点
		for (CMap_3::One_dart_per_incident_cell_range<0, 3>::iterator it2(cm.one_dart_per_incident_cell<0, 3>(it).begin()), itend2(cm.one_dart_per_incident_cell<0, 3>(it).end()); it2 != itend2; it2++) {
			center_V += cm.info_of_attribute<0>(cm.attribute<0>(it2)).pos_3 - CGAL::ORIGIN;
			size++;
		}

		center_V = center_V / size;
		center = CGAL::ORIGIN + center_V;
		polyhedra_attributes poly_att = { num, center };
		cm.info<3>(it) = poly_att;
		num++;
	}
	cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
}

//0:out  1:in
int D(CMap_3& cm, PWN_vector polyhedra_points, Point_3 center, int status) {

	int sum_d = 0;
	if (status == 0) {
		for (int i = 0; i < polyhedra_points.size(); i++) {
			Vector_3 u = center - polyhedra_points[i].first;
			Vector_3 n = polyhedra_points[i].second;
			if (n*u < 0) {
				sum_d++;
			}
		}
		return sum_d;
	}
	if (status == 1) {
		for (int i = 0; i < polyhedra_points.size(); i++) {
			Vector_3 u = center - polyhedra_points[i].first;
			Vector_3 n = polyhedra_points[i].second;
			if (n*u > 0) {
				sum_d++;
			}
		}
		return sum_d;
	}
}

struct myComp {
	bool operator()(const Plane_3& p1, const Plane_3& p2) const{
		if (to_double(p1.a()) != to_double(p2.a())) {
			return to_double(p1.a()) < to_double(p2.a());
		}
		else if (to_double(p1.b()) != to_double(p2.b())) {
			return to_double(p1.b()) < to_double(p2.b());
		}
		else if (to_double(p1.c()) != to_double(p2.c())) {
			return to_double(p1.c()) < to_double(p2.c());
		}
		else{
			return to_double(p1.d()) < to_double(p2.d());
		}
	}
};

/**************Merge functions**************/
//得到图的每条边连接的两个节点索引
std::vector<std::pair<int, int>> get_edges(std::map<int, std::vector<vertex_descriptor>> faces) {

	std::map<std::pair<vertex_descriptor, vertex_descriptor>, int> edges;
	std::vector<std::pair<int, int>> graph_edges;
	auto it = faces.begin();
	auto itEnd = faces.end();
	while (it != itEnd) {
		std::vector<vertex_descriptor> verts = it->second;
		for (int i = 0; i < verts.size() - 1; i++) {
			//auto itb = find(edges.begin(),edges.end(),std::make_pair(verts[i], verts[i + 1]));
			auto itb = edges.find(std::make_pair(verts[i + 1], verts[i]));
			if (itb != edges.end()) {
				graph_edges.push_back(std::make_pair(it->first, itb->second));
				edges.erase(itb);
			}
			/*else if (itb2 != edges.end()) {
				common_edges.push_back(*itb);
				common_edges.push_back(*itb2);
				edges.erase(itb2);

			}*/
			else {
				//graph_edges.push_back(*itb);
				edges[std::make_pair(verts[i], verts[i + 1])] = it->first;
			}
		}
		auto itb = edges.find(std::make_pair(verts[0], verts[verts.size() - 1]));
		if (itb != edges.end()) {
			graph_edges.push_back(std::make_pair(it->first, itb->second));
			edges.erase(itb);
		}
		else {
			edges[std::make_pair(verts[verts.size() - 1], verts[0])] = it->first;
		}

		it++;
	}
	return graph_edges;
}

void merge_item(std::list<std::vector<vertex_descriptor>>& item) {
	/*for (auto it = item.begin(); it != item.end(); it++) {
		for (int j = 0; j < (*it).size(); j++) {
			std::cout << (*it)[j] << " ";
		}
		std::cout << std::endl;
	}*/
	auto it = item.begin();
	//std::vector<Vertex_index>::iterator it_erase;
	while (it != item.end()) {
		auto it2 = item.begin();
		while (it2 != item.end()) {

			if (*(it->end() - 1) == *(it2->begin())) {
				it->insert(it->end(), it2->begin() + 1, it2->end());
				it2 = item.erase(it2);
				it2 = item.begin();
				break;
			}

			it2++;
		}
		if (item.size() == 1) {
			break;
		}
		if (it2 == item.end()) {
			it++;
		}
	}
}

std::vector<std::vector<vertex_descriptor>> get_boundary_edge(std::vector<int> unionField, std::map<int, std::vector<vertex_descriptor>> faces, std::set<std::pair<vertex_descriptor, vertex_descriptor> > erased_edges) {

	std::vector<std::vector<vertex_descriptor>> boundary_edges;
	for (int i = 0; i < unionField.size(); i++) {
		std::vector<vertex_descriptor> verts = faces[unionField[i]];
		std::list<std::vector<vertex_descriptor>> item;
		for (int j = 0; j < verts.size() - 1; j++) {
			auto itb = erased_edges.find(std::make_pair(verts[j], verts[j + 1]));
			auto itb2 = erased_edges.find(std::make_pair(verts[j + 1], verts[j]));

			if (itb == erased_edges.end() && itb2 == erased_edges.end()) {
				std::vector<vertex_descriptor> temp{ verts[j], verts[j + 1] };
				item.push_back(temp);
			}
		}
		auto itb = find(erased_edges.begin(), erased_edges.end(), std::make_pair(verts[verts.size() - 1], verts[0]));
		auto itb2 = find(erased_edges.begin(), erased_edges.end(), std::make_pair(verts[0], verts[verts.size() - 1]));

		if (itb == erased_edges.end() && itb2 == erased_edges.end()) {
			std::vector<vertex_descriptor> temp{ verts[verts.size() - 1], verts[0] };
			item.push_back(temp);
		}
		merge_item(item);
		for (auto it = item.begin(); it != item.end(); it++) {
			boundary_edges.push_back(*it);
		}
	}
	return boundary_edges;
}

void merge_face(std::vector<std::vector<vertex_descriptor>>& boundary_edges) {

	/*for (auto it = boundary_edges.begin(); it != boundary_edges.end(); it++) {
		for (int j = 0; j < (*it).size(); j++) {
			std::cout << (*it)[j] << " ";
		}
		std::cout << std::endl;
	}*/

	auto it = boundary_edges.begin();
	//std::vector<Vertex_index>::iterator it_erase;
	while (it != boundary_edges.end()) {
		auto it2 = it + 1;
		while (it2 != boundary_edges.end()) {

			if (*(it->end() - 1) == *(it2->begin())) {
				it->insert(it->end(), it2->begin() + 1, it2->end());
				boundary_edges.erase(it2);
				break;
			}

			it2++;
		}
		if ((*it)[0] == (*it)[(*it).size() - 1]) {
			it++;
		}
	}

}

std::optional<std::vector<Vec3>> merge(Surface_Mesh& m, std::map<int, std::vector<vertex_descriptor>>& faces, std::map<vertex_descriptor, in_Point> index_point) {
	std::vector<std::pair<int, int>> edges = get_edges(faces);
	/*for (int i = 0; i < edges.size(); i++) {
		std::cout << edges[i].first << " " << edges[i].second << std::endl;
	}*/

	ALGraph* graph = new ALGraph();
	graph->CreateGraph(faces.size(), edges.size(), edges);
	//cout << graph->DFS_findUnion() << "个连通域" << endl;
	//cout << "连通情况为：" << graph->isConnect(6, 7) << endl;

	int unionSize = graph->DFS_findUnion();

	std::set<std::pair<vertex_descriptor, vertex_descriptor> > erased_edges;
	for (int i = 0; i < graph->erase_edges.size(); i++) {

		std::vector<vertex_descriptor> v1 = faces[graph->erase_edges[i].first];
		std::vector<vertex_descriptor> v2 = faces[graph->erase_edges[i].second];
		std::sort(v1.begin(), v1.end());
		std::sort(v2.begin(), v2.end());
		std::vector<vertex_descriptor> v_intersection;
		std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v_intersection));
		erased_edges.insert(std::make_pair(v_intersection[0], v_intersection[1]));
	}

	/*for (auto it = erased_edges.begin(); it != erased_edges.end(); it++) {
		std::cout << it->first << " " << it->second << std::endl;
	}*/

	for (int i = 0; i < unionSize; i++) {
		std::vector<int> unionField = graph->get_unionfield(i + 1);
		std::vector<std::vector<vertex_descriptor>> boundary_edges = get_boundary_edge(unionField, faces, erased_edges);
		auto bk_boundary_edges = boundary_edges;
		merge_face(boundary_edges);

		std::list<vertex_descriptor> vertList;
		for (int i = 0; i < boundary_edges[0].size() - 1; i++) {
			vertList.push_back(boundary_edges[0][i]);
		}

		if (vertList.empty()) fmt::print("empty vertlist\n");

		for (auto it = vertList.begin(); it != vertList.end();) {
			auto it2 = std::next(it);
			if (it2 == vertList.end())
				it2 = vertList.begin();
			auto it3 = std::next(it2);
			if (it3 == vertList.end())
				it3 = vertList.begin();

			if (*it != *it3) {
				it++;
			}
			else {
				vertList.erase(it2);
				vertList.erase(it3);
				if (it != vertList.begin()) it = std::prev(it);
			}
		}


		std::vector<vertex_descriptor> results;
		for (auto it = vertList.begin(); it != vertList.end(); it++) {
			//std::cout << *it << " ";
			results.push_back(*it);
		}
		//std::cout << std::endl;
		if (results.empty())
		{
			fmt::print("empty results list\n");
		}

		
		//将空洞情况的共同点用不同的点索引表示
		std::vector<vertex_descriptor>::iterator it, it2;
		for (it = results.begin(); it != results.end(); it++) {
			it2 = find(results.begin(), it, *it);
			if (it2 != it) {
				vertex_descriptor new_v = m.add_vertex(index_point[*it]);
				*it = new_v;
			}
		}

		/*判断最后结果是否还存在ABA形式*/
		/*for (auto it = results.begin(); it != results.end();) {
			auto it2 = std::next(it);
			if (it2 == results.end())
				it2 = results.begin();
			auto it3 = std::next(it2);
			if (it3 == results.end())
				it3 = results.begin();

			if (*it != *it3) {
				it++;
			}
			else {
				std::cout << "ABA happens" << std::endl;
			}
		}*/


		face_descriptor f = m.add_face(results);
		//R_assert(f != Surface_Mesh::null_face());
		if (f == Surface_Mesh::null_face()) {
			std::cout << "face invalid" << std::endl;
			std::vector<Vec3> end_points;
			for (const auto &boundary : bk_boundary_edges) {
				auto point_iter = boundary.begin();
				point_iter++;
				while (point_iter != boundary.end())
				{
					auto point = index_point[*(point_iter-1)];
					end_points.push_back(Vec3{ point.x() ,point.y() ,point.z() });
					point = index_point[*point_iter];
					end_points.push_back(Vec3{ point.x() ,point.y() ,point.z() });
					point_iter++;
				}
			}
			return { end_points };
			/*std::vector<vertex_descriptor> results2;
			for (auto it = results.end(); it != results.begin(); it--) {
				results2.push_back(*it);
			}
			f = m.add_face(results2);*/
		}
		std::cout << f << std::endl;
		//m.add_face(boundary_edges[0]);
	}
	return {};
}

void merge_without_holes(Surface_Mesh& m, std::map<int, std::vector<vertex_descriptor>>& faces, std::map<vertex_descriptor, in_Point> index_point) {
	std::vector<std::pair<int, int>> edges = get_edges(faces);
	for (int i = 0; i < edges.size(); i++) {
		std::cout << edges[i].first << " " << edges[i].second << std::endl;
	}

	ALGraph* graph = new ALGraph();
	graph->CreateGraph(faces.size(), edges.size(), edges);
	//cout << graph->DFS_findUnion() << "个连通域" << endl;
	//cout << "连通情况为：" << graph->isConnect(6, 7) << endl;

	int unionSize = graph->DFS_findUnion();

	std::set<std::pair<vertex_descriptor, vertex_descriptor> > erased_edges;
	for (int i = 0; i < edges.size(); i++) {

		std::vector<vertex_descriptor> v1 = faces[edges[i].first];
		std::vector<vertex_descriptor> v2 = faces[edges[i].second];
		std::sort(v1.begin(), v1.end());
		std::sort(v2.begin(), v2.end());
		std::vector<vertex_descriptor> v_intersection;
		std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v_intersection));
		erased_edges.insert(std::make_pair(v_intersection[0], v_intersection[1]));
	}

	for (auto it = erased_edges.begin(); it != erased_edges.end(); it++) {
		std::cout << it->first << " " << it->second << std::endl;
	}

	for (int i = 0; i < unionSize; i++) {
		std::vector<int> unionField = graph->get_unionfield(i + 1);
		std::vector<std::vector<vertex_descriptor>> boundary_edges = get_boundary_edge(unionField, faces, erased_edges);
		merge_face(boundary_edges);

		for (int p = 0; p < boundary_edges.size(); p++) {
			std::list<vertex_descriptor> vertList;
			//去掉最后一个元素
			for (int i = 0; i < boundary_edges[p].size() - 1; i++) {
				vertList.push_back(boundary_edges[p][i]);
			}

			/*for (auto it = vertList.begin(); it != vertList.end();) {
				auto it2 = std::next(it);
				if (it2 == vertList.end())
					it2 = vertList.begin();
				auto it3 = std::next(it2);
				if (it3 == vertList.end())
					it3 = vertList.begin();


				if (*it != *it3) {
					it++;
				}
				else {
					vertList.erase(it2);
					vertList.erase(it3);
					if (it != vertList.begin())it = std::prev(it);
				}
			}*/


			std::vector<vertex_descriptor> results;
			//std::vector<vertex_descriptor>
			for (auto it = vertList.begin(); it != vertList.end(); it++) {
				std::cout << *it << " ";
				results.push_back(*it);
			}
			std::cout << std::endl;

			//std::vector<vertex_descriptor> results2;
			std::vector<vertex_descriptor>::iterator it, it2;
			for (it = results.begin(); it != results.end(); it++) {
				it2 = find(results.begin(), it, *it);
				if (it2 != it) {
					vertex_descriptor new_v = m.add_vertex(index_point[*it]);
					*it = new_v;
				}
			}

			for (auto it = results.begin(); it != results.end(); it++) {
				std::cout << *it << " ";
			}
			std::cout << std::endl;

			std::cout << m.add_face(results) << std::endl;
			//m.add_face(boundary_edges[0]);
		}

	}
}
/********************************Merge end*****************/



std::optional<std::vector<Vec3>> extract_surface(KPolygons_SET& polygons_set, std::string filename)
{

	CMap_3 cm;
	build_map(cm, polygons_set);

	int ghost_num;
	//C:保存每个polyhedra的一个dart
	std::vector<Dart_handle> C;
	for (CMap_3::One_dart_per_cell_range<3>::iterator it(cm.one_dart_per_cell<3>().begin()), itend(cm.one_dart_per_cell<3>().end()); it != itend; it++) {
		if (cm.info(it).second) {
			ghost_num = cm.info<3>(it).number;
			continue;
		}
		C.push_back(it);
	}
	int C_num = C.size();
	std::cout << "polyhedra number:" << C_num << std::endl;

	//F:保存每个face的一个dart
	std::vector<Dart_handle> F;
	for (CMap_3::One_dart_per_cell_range<2>::iterator it(cm.one_dart_per_cell<2>().begin()), itend(cm.one_dart_per_cell<2>().end()); it != itend; it++) {
		F.push_back(it);
	}
	std::cout << "F number" << F.size() << std::endl;

	//N:保存相邻的两个polyhedra的dart,和两个polyhedra之间的共面总面积
	Neighbor N;
	//std::vector<std::pair<Dart_handle, Dart_handle> > N;
	for (int i = 0; i < F.size(); i++) {
		int poly_number1 = cm.info_of_attribute<3>(cm.attribute<3>(F[i])).number;
		int poly_number2 = cm.info_of_attribute<3>(cm.attribute<3>(cm.beta(F[i], 3))).number;
		if (poly_number1 == ghost_num || poly_number2 == ghost_num) {
			continue;
		}
		if (poly_number1 < poly_number2) {
			NeighborKey key(std::make_pair(poly_number1, poly_number2));
			Neighbor::iterator ite = N.find(key);
			if (ite != N.end()) {
				NeighborDarts darts = ite->second;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
			else {
				NeighborDarts darts;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
		}
		else {
			NeighborKey key(std::make_pair(poly_number2, poly_number1));
			Neighbor::iterator ite = N.find(key);
			if (ite != N.end()) {
				NeighborDarts darts = ite->second;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
			else {
				NeighborDarts darts;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
		}
	}
	int N_num = N.size();
	std::cout << "N number" << N_num << std::endl;

	typedef Graph<int, int, int> GraphType;
	GraphType *g = new GraphType(/*estimated # of nodes*/ C_num, /*estimated # of edges*/ N_num);

	for (int i = 0; i < C_num; i++) {
		g->add_node();
	}

	//|I|
	int points_count = 0;
	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != polygons_set._kpolygons_set.end(); polygons++) {
		for (auto &kpoly : polygons->_kpolygons_2) {
			points_count += kpoly.inline_points.size();
		}
	}
	points_count = points_count * 2;
	std::cout << "points_count:" << points_count << std::endl;

	//A
	double sum_area = 0;
	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != std::prev(polygons_set._kpolygons_set.end(), 6); polygons++) {
		Plane_3 plane = polygons->plane();
		for (auto &kpoly : polygons->_kpolygons_2) {
			sum_area += CGAL::to_double(kpoly.area());
		}
	}
	std::cout << "sum_area:" << sum_area << std::endl;


	//{i,S,T}
	for (int i = 0; i < C_num; i++) {
		//g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, D(cm, C[i], 0) / points_count, D(cm, C[i], 1) / points_count);
		Point_3 center = cm.info_of_attribute<3>(cm.attribute<3>(C[i])).center;
		std::vector<Dart_handle> face_darts;
		PWN_vector polyhedra_points;
		//找到d所在的polyhedra的所有face上的dart
		for (CMap_3::One_dart_per_incident_cell_range<2, 3>::iterator it(cm.one_dart_per_incident_cell<2, 3>(C[i]).begin()), itend(cm.one_dart_per_incident_cell<2, 3>(C[i]).end()); it != itend; it++) {
			face_darts.push_back(it);
		}
		for (auto dart : face_darts) {
			for (auto p : cm.info_of_attribute<2>(cm.attribute<2>(dart)).inline_points) {
				polyhedra_points.push_back(p);
			}
		}
		std::cout << "Polyhedra " << i << " points num:" << polyhedra_points.size() << std::endl;

		int d_out = D(cm, polyhedra_points, center, 0);
		int d_in = D(cm, polyhedra_points, center, 1);
		g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, d_out * sum_area, d_in * sum_area);
		//std::cout << "node :"<<cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number << " i,S:"<< d_out * sum_area << " i,T:"<< d_in * sum_area << std::endl;
	}

	//{i,j}
	double lamda = 0.5;
	Neighbor::iterator ite = N.begin();
	Neighbor::iterator iteEnd = N.end();
	while (ite != iteEnd) {
		NeighborDarts darts = ite->second;
		double area = 0;
		for (auto pair_dart : darts) {
			area += CGAL::to_double(cm.info_of_attribute<2>(cm.attribute<2>(pair_dart.first)).area);
		}
		g->add_edge(ite->first.neighbors.first, ite->first.neighbors.second, lamda * area * points_count, lamda * area * points_count);
		ite++;
	}

	int flow = g->maxflow();

	printf("Flow = %d\n", flow);
	printf("Minimum cut:\n");
	for (int i = 0; i < C_num; i++) {
		if (g->what_segment(i) == GraphType::SOURCE)
			printf("node%d is in the SOURCE set\n", i);
		else
			printf("node%d is in the SINK set\n", i);
	}


	//-----get surface mesh---------
	Surface_Mesh m;

	//找到source和sink之间的面的dart d
	std::vector<Dart_handle> surface_darts;
	ite = N.begin();
	iteEnd = N.end();
	while (ite != iteEnd) {
		int d1 = ite->first.neighbors.first;
		int d2 = ite->first.neighbors.second;
		if (g->what_segment(d1) != g->what_segment(d2)) {
			NeighborDarts darts = ite->second;
			for (int i = 0; i < darts.size(); i++) {
				if (g->what_segment(cm.info<3>(darts[i].first).number) == GraphType::SINK) {
					surface_darts.push_back(darts[i].first);
				}
				else {
					surface_darts.push_back(cm.beta(darts[i].first,3));
				}
			}
		}
		ite++;
	}

	//每个平面上的所有面
	std::map < Plane_3, std::vector<Dart_handle>, myComp > merge_face;
	for (int i = 0; i < surface_darts.size(); i++) {
		Plane_3 plane = cm.info<2>(surface_darts[i]).plane;
		Direction_3 p_normal = plane.orthogonal_direction();
		Direction_3 normal = cm.info(surface_darts[i]).first;
		if (normal == p_normal) {
			auto it = merge_face.find(plane);
			if (it != merge_face.end()) {
				std::vector<Dart_handle> darts = it->second;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
			else {
				std::vector<Dart_handle> darts;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
		}
		else {
			plane = plane.opposite();
			auto it = merge_face.find(plane);
			if (it != merge_face.end()) {
				std::vector<Dart_handle> darts = it->second;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
			else {
				std::vector<Dart_handle> darts;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
		}
	}
	
	/*auto faceBegin = merge_face.begin();
	auto faceEnd = merge_face.end();
	while (faceBegin != faceEnd) {
		std::cout<<faceBegin->second.size()<<std::endl;
		faceBegin++;
	}*/

	auto itMerge = merge_face.begin();
	auto itMEnd = merge_face.end();

	//遍历每个平面
	//int num = 0;
	while (itMerge != itMEnd) {

		std::vector<Dart_handle> plane_darts = itMerge->second;
		
		//Surface_Mesh m;
		
		//get faces
		std::map<in_Point, vertex_descriptor> point_index;
		std::map<int, std::vector<vertex_descriptor>> faces;
		for (int i = 0; i < plane_darts.size(); i++) {
			Dart_handle d = plane_darts[i];

			std::vector<Vertex_index> v;

			for (CMap_3::One_dart_per_incident_cell_range<0, 2>::iterator it(cm.one_dart_per_incident_cell<0, 2>(d).begin()), itend(cm.one_dart_per_incident_cell<0, 2>(d).end()); it != itend; ++it)
			{
				Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it)).pos_3;
				in_Point p = in_Point(
					CGAL::to_double(p3.x()),
					CGAL::to_double(p3.y()),
					CGAL::to_double(p3.z()));

				auto itp = point_index.find(p);
				if (itp != point_index.end()) {
					v.push_back(itp->second);
				}
				else {
					vertex_descriptor u = m.add_vertex(p);
					point_index[p] = u;
					v.push_back(u);
				}
			}
			faces[i] = v;
			
		}
		std::map<vertex_descriptor, in_Point> index_point;
		for (auto it = point_index.begin(); it != point_index.end(); it++) {
			index_point[it->second] = it->first;
		}

		// auto maybe_lines = merge(m, faces,index_point);
		// if (maybe_lines){
		// 	fmt::print("merge face wrong\n");
		// 	return maybe_lines;
		// }

		// //输出plane
		// std::string file = "src/output/" + filename + std::to_string(num) + ".off";
		// std::ofstream f(file);
		// if (!CGAL::write_off(f, m)) {
		// 	std::cout << "write wrong" << std::endl;
		// }
		//num++;

		
		merge_without_holes(m, faces, index_point);

		itMerge++;
	}

	//输出convex mesh
	std::string file = "src/output/" + filename + ".off";
	std::ofstream f(file);
	if (!CGAL::write_off(f, m)) {
		std::cout << "write wrong" << std::endl;
	}
	delete g;
	return {};
}