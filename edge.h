#include "common.h"

class Edge {
	Node _node1, _node2;
	int _cost;
	Location _location;
public:
	int getCost(void) { return _cost; }
	Node getNextNode(Node fromNode) { return fromNode == _node1 ? _node2 : _node1; }
	Edge(Node node1, Node node2, int cost, Location location);
	~Edge(void);
};

