#ifndef GAME_ENV_H
#define GAME_ENV_H

#include <unordered_map>
#include "common.h"
#include "edge.h"

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
/*---------------------------------------------------------------------------*/
struct ActiveTasks {
	std::unordered_map<int, int> vans; // <van number, delivery number>
	std::unordered_map<int, int> deliveries; // <delivery number, van number>
	// <location, delivery number>
	std::unordered_map<Location, int, hash_pair> pickup2del; 
	// <delivery number location>
	std::unordered_map<int, Location> del2pickup;
	// <delivery number van number>
	std::unordered_map<int, int> deferredDeliveries;
	std::unordered_map<int, int> deferredVans;
};


enum algo {dijkstra=1, euclideanDistance, manhattanDistance, repulsiveCenter};
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
    static const int _spread_out_distance = 15;

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
	int getGameTime(void){return _gameInfo.time;}
	bool isTimeElapsed () { 
		return (_gameInfo.time >= G_END_TIME 
			|| _gameInfo.completedDeliveries.size()==TOTAL_DELIVERIES); 
	}
	void spreadOut(void);
	void updateGameInfo(void);
	void startGame(void);
	void sendInstructions(InstructionsSet instuctions);
	// assign a delivery to vans.
	void assignDeliveries(void /*probably something here*/); 
	void computeInstructions(void /*same thing here*/); // A* algorithm 
	void checkForAccidents(void);
	uint8_t euclideanDistance(Node node1, Node node2);
	uint8_t roadBase(Node node1, Node node2);
	uint8_t manhattanDistance(Node node1, Node node2);
	uint8_t manhattanDistanceWeighted(Node node1, Node node2);
	uint8_t dijkstra(Node node1, Node node2);
	uint8_t repulsiveCenter(Node node1, Node node2);
	uint8_t recursiveCost(Node start, Node goal);
	uint8_t GameEnv::inspect(Node start, Node goal, uint8_t limit);

	bool checkPath(Node start, Node end, h_func heuristic);
	void manageDeliveries(void);
	void spreadOutFreeVans(void);
	void manageDefferedDeliveries(void);
	void precomputeRoadTypes(void);
	void assignDeliveriesParallel(void);
	DebugInfo TestAlgorithm(int h_func);
private:
	void clearGameInfo(void); 
	VanInfo* getFreeVanNumber(Node goal); 
	DeliveryInfo* getAvailableDelivery(void);
	bool scheduleDeliveryTask(__int8 deliveryNum, __int8 vanNum);
	void removeTaskByDelivery(__int8 deliveryNum);
	void removeTaskByVan(__int8 vanNum);
	void removeTask(__int8 deliveryNum, __int8 vanNum);
	bool isAdjacent(Location edge1, Location edge2);
	std::vector<Edge> getOutgoingEdges(Node fromNode);
	Path findRoad(Node start, Node end, h_func heuristic);
	Path findRoadOptimal(Node start, Node end, h_func heuristic);
	Path findPath(Node start, Node end);
	Path findPathDebug(Node start, Node end, int h_func);
	void sendInstructionsToVan(uint8_t van, Path instructions);
	Node getDropOff(uint8_t deliveryNum);
	Node findNearestAnchor(Node node);
	bool isAnchor(Node node);
	void addDefferedDelivery(int deliveryNum, int vanNum);
	Path findRoadOptimized(Node start, Node goal, h_func heuristic);
};
//Node arr[] = {std::make_pair(20,10),std::make_pair(30,20),std::make_pair(20,30),std::make_pair(10,20)};
//std::vector<int> TestVector(arr, arr+5);
//const std::vector<Node> GameEnv::_anchors(arr,arr+5);

#endif