#ifndef NODE_H_FRI2015YOLO
#define NODE_H_FRI2015YOLO

class Node {
	int reward;
	int color; //00000000rrrrrrrrggggggggbbbbbbbb
	vector<Node*> neighbors;

	public Node(int r, int c): reward(r), color(c){}
	public void addNeighbor(Node* n) {
		neighbors.push_back(n);
	}
};

#endif