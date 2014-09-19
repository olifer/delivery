#ifndef COMMON_H
#define COMMON_H

#include "stdafx.h"

#include "DeliveryManClient.h"
#pragma comment (lib,"DeliveryManClient")

#include <queue>

#define G_MATRIX_WIDTH 40	// width of the game matrix
#define G_MATRIX_LENGTH 40	// length of the game matrix
// apprx. update interval of the traffic conditions
#define G_UPDATE_INTERVAL 100	
#define N_VANS 5	

// Game node represented by its location <Y,X>
typedef Location Node;
// 2D array representing types of game nodes
typedef std::vector<std::vector<std::wstring>> GameNodesTypes;
// 2D array representing costs of game edges 
typedef std::vector<std::vector<int>> GameEdgesCosts;
// Array of vans entries
typedef std::vector<VanInfo> VanList;
// Array of deliveries entries
typedef std::vector<DeliveryInfo> DeliveryList;
typedef std::vector<Location> Path;
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

struct NodeEntry{
	Node node;	// considered node
	Location edge;	// the edge that led to the node Edge*
	int computedCost;	// g(n)
	int expectedTotalCost; // f(n) = g(n) + h(n)

	// NodeEntries are equal if their nodes have equal <Y,X>
    bool operator==(const NodeEntry& entry) const
    {
        return (node == entry.node);
    }
};
/* Overload < of NodeEntry for usage in priority queue*/
inline bool operator> (const NodeEntry& node1, const NodeEntry& node2){
	return node1.expectedTotalCost > node2.expectedTotalCost;
}
// List of node entries (intended for open/closed sets)
typedef std::vector<NodeEntry> NodeEntryList;
//typedef priority_queue<NodeEntry, NodeEntryList, less<NodeEntry>> JobQueue;
//typedef priority_queue<NodeEntry> NodeEntryList2;

struct hash_pair {
    template <typename T, typename U>
    std::size_t operator ()(std::pair<T, U> const& p) const {
        using std::hash;
        return hash<T>()(p.first) ^ hash<T>()(p.second);
    }
};

#endif