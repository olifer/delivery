#ifndef GAME_ENV_H
#define GAME_ENV_H

#include <unordered_map>
#include "common.h"
#include "edge.h"

/*---------------------------------------------------------------------------*/
struct ActiveTasks {
	std::unordered_map<int, int> vans; // <van number, delivery number>
	std::unordered_map<int, int> deliveries; // <delivery number, van number>
	// <location, delivery number>
	std::unordered_map<Location, int, hash_pair> pickup2del; 
	// <delivery number location>
	std::unordered_map<int, Location> del2pickup;
};

class GameEnv {
	// ponter to heuristic function
	typedef uint8_t (GameEnv::*h_func)(Node node1, Node node2);
    static const int _spread_out_distance = 10;
	DM_Client *_client;
	GameNodesTypes _gameNodesTypes;
	GameInfo _gameInfo;
	// active tasks assign to vans with specification
	// of the current goal type (pick-up:false, drop-off:true)
	ActiveTasks _activeTasks;
public:
	GameEnv(DM_Client *client){ 
		_client = client; _activeTasks = ActiveTasks(); 
	}
	~GameEnv(void);
	bool isTimeElapsed () { return (_gameInfo.time > G_END_TIME); }
	void spreadOut(void);
	void updateGameInfo(void);
	void startGame(void);
	void sendInstructions(InstructionsSet instuctions);
	// assign a delivery to vans.
	void assignDeliveries(void /*probably something here*/); 
	void computeInstructions(void /*same thing here*/); // A* algorithm 
	void checkForAccidents(void);
	uint8_t euclideanDistance(Node node1, Node node2);
	uint8_t euclideanDistanceFast(Node node1, Node node2);
	uint8_t manhattanDistance(Node node1, Node node2);
	bool checkPath(Node start, Node end, h_func heuristic);
	void manageDeliveries(void);
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
	void sendInstructionsToVan(uint8_t van, Path instructions);
	Node getDropOff(uint8_t deliveryNum);
};

#endif