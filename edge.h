#ifndef EDGE_H
#define EDGE_H

#include "common.h"

class Edge {
	Location _node1, _node2;
	int _cost;
	Location _location;
public:
	typedef std::vector<Edge>::iterator edge_itr;
	int getCost(void) { return _cost; }
	Location getLocation(void) { return _location; }
	Location getNextNode(Location fromNode) { return fromNode == _node1 ? _node2 : _node1; }
	Edge(Node node1, Node node2, int cost, Location location);
	~Edge(void);
};

#endif
