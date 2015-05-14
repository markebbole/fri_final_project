#ifndef NODE_H_FRI2015YOLO
#define NODE_H_FRI2015YOLO
#include <vector>
using namespace std;

class Node {
	
public:
	int reward;
	int color; //00000000rrrrrrrrggggggggbbbbbbbb
	int index; //used to 
	vector<Node*> neighbors;
	Node(int r, int c, int i) : reward(r), color(c), index(i) {}
	void addNeighbor(Node* n) {
		neighbors.push_back(n);
	}
};

#endif