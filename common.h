#ifndef COMMON_H
#define COMMON_H

#include "stdafx.h"

#include "DeliveryManClient.h"
#pragma comment (lib,"DeliveryManClient")

#define G_MATRIX_WIDTH 40	// width of the game matrix
#define G_MATRIX_LENGTH 40	// length of the game matrix
// apprx. update interval of the traffic conditions
#define G_UPDATE_INTERVAL 100	

// Game node represented by its location <Y,X>
typedef Location Node;	
// 2D array representing types of game nodes
typedef std::vector<std::vector<std::wstring>> TypesOfNodes;
// 2D array representing costs of game edges 
typedef std::vector<std::vector<int>> GameEdgesCosts;
// Array of vans entries
typedef std::vector<VanInfo> VanList;
// Array of deliveries entries
typedef std::vector<DeliveryInfo> DeliveryList;
// Set of instructions maped to each van
typedef std::map<int,std::vector<Location>> InstructionsSet;
//typedef InstructionsSet::iterator InstructionsSetIterator;



struct GameInfo{
	int time;
	GameEdgesCosts edges;
	VanList vans;
	DeliveryList waitingDeliveries;
	DeliveryList activeDeliveries;
	std::vector<std::pair<int,int>> completedDeliveries;
	std::wstring output; // a response.
}; 

/*	
*	The structure keeps extended information for the node, when its parent
*	becomes expanded. The node entry is placed in the open/close list
*	and intended to algorithm processing and path reconstruction. 
*/

class Edge;	// forward declaration

struct NodeEntry{
	Location* _node;	// considered node
	Edge* _edge;	// the edge that led to the node
	int _computedCost;	// g(n)
	int _estimatedTotalCost; // f(n) = g(n) + h(n)
};

// List of node entries (intended for open/closed sets)
typedef std::vector<NodeEntry> NodeEntryList;

#endif