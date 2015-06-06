#include "Graph.h"
#include <sstream>
#include <fstream>
#include <limits>
#include <queue>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

vector<string> getValues(const string &data);

Graph::Graph()
{
}

Graph::~Graph()
{
}

void Graph::buildNode(const string &data)
{
	vector<string> &values = getValues(data);
	Node p;

	p.name = stoi(values[0]);
	p.x = stod(values[1]);
	p.y = stod(values[2]);

	p.isAnchor = values[3] == "1";
	p.isPseudoAnchor = false;

	if (!p.isAnchor) {
		p.cx = numeric_limits<double>::max();
		p.cy = p.cx;
	}
	else {
		p.cx = p.x;
		p.cy = p.y;
	}

	points.push_back(p);
	// id is it's index
	points.back().id = points.size() - 1;
}

void Graph::buildEdge(const string &data)
{
	vector<string> &values = getValues(data);
	Node &p1 = points[stoi(values[0]) - 1];
	Node &p2 = points[stoi(values[1]) - 1];
	double d = stod(values[2]);

	p1.addNeighbour(p2, d);
	p2.addNeighbour(p1, d);
}

vector<string> getValues(const string &data) {
	istringstream ss(data);
	string value;
	vector<string> values;
	int i = 0;
	while (ss >> value) {
		values.push_back(value);
		++i;
	}
	return values;
}

GRAPHDLL_API void Graph::buildFromFile(ifstream &pointData, ifstream &edgeData) {
	for (std::string line; std::getline(pointData, line); ) {
		buildNode(line);
	}

	for (std::string line; std::getline(edgeData, line);) {
		buildEdge(line);
	}
}

GRAPHDLL_API void Graph::shortestPath_Floyd()
{
	if (minimumPathLength.size() == points.size()) {
		return;
	}

	minimumPathLength.resize(points.size());
	for (auto &p1 : points) {
		minimumPathLength[p1.getId()] = vector<double>(points.size(), numeric_limits<double>::max());;
		// self distance is 0
		minimumPathLength[p1.getId()][p1.getId()] = 0;
	}

	for (auto &p : points)
	{
		for (auto &pair : p.neighbourDistance) {
			int neighbour = pair.first,
				pid = p.getId();
			// the adjacent point has their immediate distance
			minimumPathLength[pid][neighbour] = 1;
			minimumPathLength[neighbour][pid] = 1;
		}
	}

	for (size_t k = 0; k < points.size(); k++)
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			for (size_t j = 0; j < points.size(); j++)
			{
				if (minimumPathLength[i][j] > minimumPathLength[i][k] + minimumPathLength[k][j])
				{
					minimumPathLength[i][j] = minimumPathLength[i][k] + minimumPathLength[k][j];
				}
			}
		}
	}
}

GRAPHDLL_API void Graph::shortestPath_Dijest()
{
	for (auto &p : points) {
		vector<double> dist(points.size(), numeric_limits<double>::max());
		set<int> seen;
		deque<Node*> neighbours;

		neighbours.push_back(&p);

		while (neighbours.size() > 0) {
			Node *current = neighbours.front();
			neighbours.pop_front();

			for (auto &nextNeighbour : current->neighbourDistance) {
				if (nextNeighbour.second + dist[current->getId()] <
					dist[nextNeighbour.first]) {
					dist[nextNeighbour.first] = nextNeighbour.second + dist[current->getId()];
				}
				if (seen.find(nextNeighbour.first) == seen.end()) {
					seen.insert(nextNeighbour.first);
					neighbours.push_back(&points[nextNeighbour.first]);
				}
			}
		}
	}
}

GRAPHDLL_API vector<Node>::size_type Graph::size()
{
	return points.size();
}

GRAPHDLL_API Node& Graph::getPoint(int id)
{
	try {
		return points[id];
	}
	catch (exception e) {
		printf(e.what());
	}
}

// ���㷽�� Ax = b
GRAPHDLL_API Vector2d trilateration(Matrix<double, Dynamic, 2> &a, VectorXd &b) {
	return a.colPivHouseholderQr().solve(b);
}

// ����׼����� Ax = b ���󷽳��е� A ����� b ����������ĳ���㵽������ľ�������λ��
// ԭ���ķ�����Ӧ���� (cx - ax(i))^2 + (cy - ay(i))^2 = d(i)^2; i = 1,2,3,4...n�� n����֪�ڵ������
// ���� cx,cy ��δ֪������꣬ ax(i),ay(i) ����֪������꣬ d(i) ������֮��ľ���
// ʵ�ʾ��� ����֮���������빫ʽ
// ͨ���任���Եõ�����		cx^2 + cy^2 - 2*cx*ax(i) - 2*cy*ay(i) = d(i)^2 - ax(i)^2 - ay(i)^2 �� n�� ���� ��ɵķ�����
// ͨ�������黯�򣬿��Եõ�	2*cx*(ax(1) - ax(i)) + 2*cy*(ay(1) - ay(i)) = d(i)^2 - ax(i)^2 - ay(i)^2 - (d(1)^2 - ax(1)^2 - ay(1)^2) 
// ��д�ɾ�����ʽ��		A*x = b. ���� A(i,0) = 2*(ax(1) - ax(i), A(i,1) = 2*(ay(1) - ay(i), x ��(cx,cy) ����, b�����淽���ұ���ɵ�����
GRAPHDLL_API Vector2d calculatePoint(Node &node, vector<Node*> anchors, int pivot = 0) {
	Matrix<double, Dynamic, 2> a(anchors.size() - 1, 2);

	double x1, y1, d1;
	vector<Node*>::value_type p1;
	if (pivot == -1 || pivot >= anchors.size ()) {
		p1 = *(--anchors.end ());
	}
	else {
		p1 = anchors[pivot];
	}
	x1 = p1->cx;
	y1 = p1->cy;
	d1 = node.distanceToLandmarks.at(p1->getId());

	// (d(1)^2 - ax(1)^2 - ay(1)^2)  �����ұ��ⲿ���ǲ���ģ����Ի���������
	double fixed = d1 * d1 - x1 * x1 - y1* y1;

	int i = 0;

	// ���ɾ��� A
	for (auto it = anchors.begin(); it != anchors.end(); it++) {
		if (*it == p1) {
			continue;
		}

		// �󷽳���ߵ� A(i, 0) = 2*(ax(1) - ax(i)) �� A(i, 1) = 2*(ay(1) - ay(i))
		// it �� ��֪�ڵ㣨ê�ڵ㣩��ָ��
		a(i, 0) = 2 * (x1 - (*it)->cx);
		a(i, 1) = 2 * (y1 - (*it)->cy);

		++i;
	}

	VectorXd b(anchors.size() - 1);
	i = 0;

	// ��������b
	for (auto it = anchors.begin(); it != anchors.end(); it++) {
		if (*it == p1) {
			continue;
		}

		double d = node.distanceToLandmarks.at((*it)->getId());
		auto node = (*it);

		// b(i) = d(i)^2 - ax(i)^2 - ay(i)^2 - (d(1)^2 - ax(1)^2 - ay(1)^2)
		b(i) = d * d - node->cx * node->cx -
			node->cy * node->cy - fixed;

		++i;
	}

	// ��� x. A*x = b; ���� x = pinv(A'*A)*A'*b; pinv() ������α�����
	return trilateration(a, b);
}

/***
* ��������ζ�λ��
* ˼·��ʹ�����Ƕ�λ�㷨��ͨ����֪ê�ڵ�����δ֪�ڵ��λ�ã�����δ֪�ڵ�����Ϊαê�ڵ㹩����
* δ֪�ڵ���㣬ֱ�����нڵ���֪λ�û��ߵ�������������ޡ�
*/
GRAPHDLL_API void recursiveTri(Graph &graph) {
	// ����������Ҫ����λ�õĽڵ�Ķ���
	queue<Node *> tobeCal;

	for (auto iter = graph.points.begin(); iter != graph.points.end(); ++iter) {
		if (!iter->isAnchor) {
			// ��ʼ��ʱ�����з�ê�ڵ㶼��Ҫ����
			tobeCal.push(&(*iter));
		}
	}

	// ���õ����������ޣ������нڵ�������10�������Ϊ10������ͼ�ĵ���
	// ����������Ϊ�˱��⵱����������һ������ͨͼ��ʱ������������ѭ��
	int maxIteration = 320 * 10;

	// ���Ѿ���������Ҫ����ĵ� ���� ���������������ޣ�ֹͣ����
	while (tobeCal.size() != 0 && maxIteration-- != 0) {
		// ȡ�����ڵ���еĵ�һ���ڵ� node
		Node *node = tobeCal.front();
		tobeCal.pop();

		vector<Node *> anchors;

		// ȡ node ���ڽ��ڵ������� ê�ڵ� �� αê�ڵ� �������Ƕ�λ
		for (auto it = node->neighbourDistance.cbegin();
			it != node->neighbourDistance.cend(); it++) {
			if (graph.points[it->first].isAnchor || graph.points[it->first].isPseudoAnchor) {
				node->distanceToLandmarks[it->first] = it->second;
				anchors.push_back(&graph.points[it->first]);
			}
		}

		// ��� node �ڽڵ��е�ê�ڵ�������3�����޷��������Ƕ�λ
		// �� node ���·��뵽 �����ڵ���еĶ���β�����ȴ���һ�ֵ���
		if (node->distanceToLandmarks.size() < 3) {
			tobeCal.push(node);
			continue;
		}
		
		// ������λ��
		auto &ret = calculatePoint(*node, anchors, -1);

		// ������
		node->cx = ret(0);
		node->cy = ret(1);

		// ����Ϊαê�ڵ�
		node->isPseudoAnchor = true;
	}
}

GRAPHDLL_API void dvhop(Graph &graph) {
	vector<Node *> anchors;

	// find out all the landmarks
	for (auto &p : graph.points) {
		if (p.isAnchor) {
			anchors.push_back(&p);
		}
	}

	// calculate the correction of landmarks
	for (auto &anchor : anchors) {
		double distSum = 0,
			hopSum = 0;

		for (auto &landmark : anchors) {
			if (anchor != landmark) {
				double dx = (anchor->getX() - landmark->getX()),
					dy = (anchor->getY() - landmark->getY());

				distSum += sqrt(dx*dx + dy*dy);
				hopSum += graph.minimumPathLength[anchor->getId()][landmark->getId()];
			}
		}

		anchor->correction = distSum / hopSum;
	}

	// determine the correction of regular point
	// to be its nearest landmark's correction
	for (auto &p : graph.points) {
		if (p.isAnchor) {
			continue;
		}

		decltype(anchors[0]) min = anchors[0];

		// find the nearest anchor
		for (auto &anchor : anchors) {
			if (graph.minimumPathLength[p.getId()][anchor->getId()] <
				graph.minimumPathLength[p.getId()][min->getId()]) {
				min = anchor;
			}
		}

		p.correction = min->correction;

		for (auto &anchor : anchors) {
			p.distanceToLandmarks[anchor->getId()] = graph.minimumPathLength[p.getId()][anchor->getId()] * p.correction;
		}

		auto ret = calculatePoint(p, anchors);

		p.cx = ret(0);
		p.cy = ret(1);
	}
}

MatrixXd pinv(MatrixXd &mat) {
	
	auto svd = mat.jacobiSvd(ComputeFullU | ComputeFullV);
	VectorXd singularValue = svd.singularValues(), singularValue_inv = singularValue;

	double pinvtoler = 0.98, sum = 0;
	int truncateIndex = 0;

	for (int i = 0; i < singularValue.rows(); i++) {
		if ((sum += singularValue[i]) / singularValue.sum() > pinvtoler) {
			truncateIndex = i;
			break;
		}
	}

	for (long i = 0; i < mat.cols(); i++) {
		if (i < truncateIndex) {
			singularValue_inv(i) = 1.0 / singularValue(i);
		}
		else
			singularValue_inv(i) = 0;
	}

	return (svd.matrixV() * singularValue_inv.asDiagonal() * svd.matrixU().transpose());
}

GRAPHDLL_API void pdm(Graph &graph)
{
	vector<Node *> anchors;

	// find out all the landmarks
	for (auto &p : graph.points) {
		if (p.isAnchor) {
			anchors.push_back(&p);
		}
	}

	// P is the proximity matrix and
	// T is the geographic matrix
	MatrixXd P(anchors.size(), anchors.size());
	MatrixXd L(anchors.size(), anchors.size());

	// calculate the correction of landmarks
	for (int i = 0; i < anchors.size(); i++) {
		auto &a1 = anchors[i];
		for (int j = 0; j < anchors.size(); j++) {
			auto &a2 = anchors[j];

			if (a1 != a2) {
				double dx = (a1->getX() - a2->getX()),
					dy = (a1->getY() - a2->getY());

				// geographic distance
				L(i, j) = sqrtl(dx*dx + dy*dy);
				// proximity matrix here assumed to be mini-hop
				P(i, j) = graph.minimumPathLength[a1->getId()][a2->getId()];
			}
			else {
				L(i, j) = 0;
				P(i, j) = 0;
			}
		}
	}

	MatrixXd T = L * pinv(P);

	for (auto &p : graph.points) {
		if (p.isAnchor) {
			continue;
		}

		VectorXd proximity(anchors.size());

		for (int i = 0; i < anchors.size(); i++) {
			proximity(i) = graph.minimumPathLength[p.getId()][anchors[i]->getId()];
		}

		VectorXd geography = T * proximity;

		for (int i = 0; i < anchors.size(); i++) {
			//p.addNeighbour(*anchors[i], geography(i));
			p.distanceToLandmarks[anchors[i]->getId()] = geography(i);
		}

		auto ret = calculatePoint(p, anchors);

		p.cx = ret(0);
		p.cy = ret(1);
	}
}

/*calculate the x(i, j)^2 / (n* m) where x is a nxm matrix*/
double qudraticMean(const MatrixXd &mat) {
	double ret = 0;
	int cols = mat.cols(), rows = mat.rows();
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			ret += mat(i, j) * mat(i, j);
		}
	}
	return ret / cols / rows;
}

MatrixXd doubleCenter(MatrixXd &mat) {
	if (mat.rows() != mat.cols()) {
		throw runtime_error("Can't double center non-square matrix");
	}

	size_t n = mat.rows();
	MatrixXd ret(n, n);

	double means = qudraticMean(mat);

	for (size_t i = 0; i < n; ++i) {
		for (size_t j = 0; j < n; ++j) {
			ret(i, j) = -(mat(i, j)*mat(i, j) - qudraticMean(mat.row(i)) - qudraticMean(mat.col(j)) + means) / 2;
		}
	}

	return ret;
}

/*****
* MDS algorithm
* 
*/
GRAPHDLL_API void mds(Graph &graph)
{
	MatrixXd P(graph.minimumPathLength.size(), graph.minimumPathLength.size());

	for (size_t i = 0, n = P.rows(); i < n; ++i) {
		for (size_t j = 0; j < n; ++j) {
			P(i, j) = graph.minimumPathLength[i][j];
		}
	}

	vector<int> anchors;

	// find out all the landmarks
	for (auto &p : graph.points) {
		if (p.isAnchor) {
			anchors.push_back(p.getId());
		}
	}
	cout << "doublecenter start" << endl;
	// double centered matrix
	MatrixXd B = doubleCenter(P);

	cout << "doublecenter end" << endl;

	// singular value decomposition
	cout << "svd start" << endl;
	JacobiSVD<MatrixXd> svd(B, ComputeFullU | ComputeThinV);

	// picking r=2 largest singular value because we are
	// computing in a 2D space
	auto singulars = svd.singularValues();
	MatrixXd U(P.rows(), 2), V(2, 2);
	U.col(0) = svd.matrixU().col(0);
	U.col(1) = svd.matrixU().col(1);

	V << singulars(0), 0, 0, singulars(1);

	cout << "svd end" << endl;

	// compute back to the coordinates matrix
	MatrixXd X = U * V.sqrt();

	// convert relative coordinate to absolute coordinate
	// using transformation computing through anchors
	MatrixXd anchorRel(anchors.size(), 3),
		anchorAbs(anchors.size(), 2);

	for (size_t i = 0; i < anchors.size(); ++i) {
		anchorRel(i, 0) = 1;
		anchorRel(i, 1) = X(anchors[i], 0);
		anchorRel(i, 2) = X(anchors[i], 1);
		anchorAbs(i, 0) = graph.getPoint(anchors[i]).getX();
		anchorAbs(i, 1) = graph.getPoint(anchors[i]).getY();
	}
	cout << "anchor svd start" << endl;
	MatrixXd T = anchorRel.jacobiSvd(ComputeThinU | ComputeThinV).solve(anchorAbs);
	cout << "anchor svd end" << endl;

	VectorXd temp(3);
	for (int i = 0; i < graph.size(); i++) {
		temp << 1,X(i, 0),X(i, 1);
		const MatrixXd &ret = temp.transpose() *T;
		graph.getPoint(i).cx = ret(0);
		graph.getPoint(i).cy = ret(1);
	}
}

GRAPHDLL_API void mytest(int size) {
	MatrixXd test = MatrixXd::Random(size, size);
	MatrixXd ret = test.inverse();
}