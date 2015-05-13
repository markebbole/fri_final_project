#ifndef NODE_H_FRI2015YOLO
#define NODE_H_FRI2015YOLO

class Node {
	int reward;
	int color; //00000000rrrrrrrrggggggggbbbbbbbb
	int index;
	vector<Node*> neighbors;

	public Node(int r, int c, int i): reward(r), color(c), index(i) {}
	public void addNeighbor(Node* n) {
		neighbors.push_back(n);
	}
};

#endif