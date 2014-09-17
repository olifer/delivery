#include "edge.h"

Edge::Edge(Node node1, Node node2, int cost, Location location) {
	_node1 = node1;
	_node2 = node2;
	_cost = cost;
	_location = location;
}

Edge::~Edge(void){
}