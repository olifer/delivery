#ifndef GAME_ENV_H
#define GAME_ENV_H

// warning C4482: nonstandard extension used: enum 'ALGO' used in qualified name 
#pragma warning(disable : 4482) 

#include <unordered_map>
#include "common.h"
#include "edge.h"

/* Types of heuristic function. */
enum ALGO { 
	DIJKSTRA=1, 
	EUCLIDEAN_DISTANCE, 
	MANHATTAN_DISTANCE, 
	REPULSIVE_CENTER
};

struct NodeRecord{
	NodeRecord(){};
	NodeRecord(Node node, Location fromEdge, int cost):node(node),expectedTotalCost(cost),fromEdge(fromEdge){};
	Node node;
	Location fromEdge; // edge that led to the node
	int expectedTotalCost; // f(n) = g(n) + h(n)
};
/* Overload < of NodeEntry for usage in priority queue*/
inline bool operator> (const NodeRecord& node1, const NodeRecord& node2){
	return node1.expectedTotalCost > node2.expectedTotalCost;
}
inline bool operator>= (const NodeRecord& node1, const NodeRecord& node2){
	return node1.expectedTotalCost >= node2.expectedTotalCost;
}
/*---------------------------------------------------------------------------*/
// Schedule information
struct ActiveTasks {
	// Relevant for pick-up deliveries only
	std::unordered_map<int, int> vans; // <van number, delivery number>
	std::unordered_map<int, int> deliveries; // <delivery number, van number>
	// <location, delivery number>
	std::unordered_map<Location, int, hash_pair> pickup2del; 
	// <delivery number location>
	std::unordered_map<int, Location> del2pickup;

	// Relevant for drop-off deliveries only
	// <delivery number van number>
	std::unordered_map<int, int> deferredDeliveries;
	std::unordered_map<int, int> deferredVans;
};

// For DEBUG purporses
struct DebugInfo{
	double quality;
	double elapsed_secs;
	double avg;
	double avg_game_units;
	double avg_time;
};		

class GameEnv {
	// ponter to heuristic function
	typedef uint8_t (GameEnv::*h_func)(Node node1, Node node2);
    static const int _spread_out_distance = 10;
	std::vector<Node> _anchors;
	DM_Client *_client;
	GameNodesTypes _gameNodesTypes;
	GameEdgesCosts _edgesTypes;
	GameInfo _gameInfo;
	// active tasks assign to vans with specification
	// of the current goal type (pick-up:false, drop-off:true)
	ActiveTasks _activeTasks;
	// for DEBUG
	int cost;
	int maxCost;
public:
	GameEnv(DM_Client *client){ 
		_client = client; 
		_activeTasks = ActiveTasks(); 
		Node arr[] = {std::make_pair(20,10),std::make_pair(30,20),
			std::make_pair(20,30),std::make_pair(10,20),std::make_pair(20,20)};
		_anchors.assign(arr,arr+5);
		maxCost = 0;
	}
	~GameEnv(void);

	int getGameTime(void){ return _gameInfo.time; }

	bool isTimeElapsed () { 
		return (_gameInfo.time >= G_END_TIME 
			|| _gameInfo.completedDeliveries.size()==TOTAL_DELIVERIES); 
	}

	void startGame(void);

	// VAN CONTROL
	void spreadOut(void);

	// GAME ROUTINE
	void updateGameInfo(void);
	
	// DELIVERY MANAGEMENT
	void assignDeliveriesParallel(void);
	// Drop-off routines
	void manageDeliveries(void);
	void manageDefferedDeliveries(void);

	// HEURISTICS
	uint8_t euclideanDistance(Node node1, Node node2);
	uint8_t roadBase(Node node1, Node node2);
	uint8_t manhattanDistance(Node node1, Node node2);
	uint8_t manhattanDistanceWeighted(Node node1, Node node2);
	uint8_t repulsiveCenter(Node node1, Node node2);
	uint8_t dijkstra(Node node1, Node node2);

	// DEBUG FUNCTIONALITY
	bool checkPath(Node start, Node end, h_func heuristic);
	
	// PATH FINDING
	void precomputeRoadTypes(void);

	// DEBUG FUNCTIONALITY
	DebugInfo TestAlgorithm(int h_func);
private:
	// OPERATIONAL INFORMATION
	DeliveryInfo* getDeliveryInfo(int& deliveryNum);
	DeliveryInfo* getAvailableDelivery(void);
	VanInfo* getFreeNearestVan(Node& pickup);
	bool isDeliveryAssignedForPickUp(int& deliveryNum);
	bool isFreeVan(int& vanNum);
	bool hasVanPickedUpDelivery(int& vanNum);

	// VAN CONTROL
	void sendInstructions(InstructionsSet& instuctions);
	void sendInstructionsToVan(int& van, Path& instructions);
	void sendVanTo(int& vanNumber, Location& location);

	// TASKS MANAGEMENT
	bool schedulePickUpTask(DeliveryInfo& delivery, int& vanNum);
	void removePickUpTask(int& deliveryNum, int& vanNum);
	void addDefferedDropOffTask(int& deliveryNum, int& vanNum);
	void removeDefferedDropOffTask(int& deliveryNum, int& vanNum);
	
	// PATH FINDING
	Path findPath(Node& start, Node& end);
	Path findRoadOptimized(Node& start, Node& goal, h_func heuristic);
	std::vector<Edge> getOutgoingEdges(Node fromNode);

	// DELIVERY MANAGEMENT
	void SolveConflict(int& schdDeliveryNum, int& vanNum, int& actualDeliveryNum);
	
	// GAME ROUTINE
	void clearGameInfo(void); 
	
	// DEBUG FUNCTIONALITY
	bool isAdjacent(Location edge1, Location edge2);
	Path findPathDebug(Node start, Node end, int h_func);

	// TEMP
	// old version of path finding algorithm
	Path findRoad(Node start, Node end, h_func heuristic);
};

#endif