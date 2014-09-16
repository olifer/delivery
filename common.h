#include "stdafx.h"

#include "DeliveryManClient.h"
#pragma comment (lib,"DeliveryManClient")


typedef Location Node;
typedef std::vector<std::vector<std::wstring>> GameNodes;
typedef std::vector<std::vector<int>> GameEdgesCosts;
typedef std::vector<VanInfo> VanList;
typedef std::vector<DeliveryInfo> DeliveryList;
// set of instructions maped to each van
typedef std::map<int,std::vector<Location>> InstructionsSet;
//typedef InstructionsSet::iterator InstructionsSetIterator;

struct GameInfo{
	int time;
	GameEdgesCosts edges;
	VanList vans;
	DeliveryList waitingDeliveries;
	DeliveryList activeDeliveries;
	std::vector<std::pair<int,int>> completedDeliveries;
	std::wstring output;
}; 