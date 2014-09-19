#ifndef GAME_ENV_H
#define GAME_ENV_H

#include <unordered_map>
#include <functional>
#include "common.h"
#include "edge.h"

// <van number, delivery number>

struct ActiveTasks {
	std::unordered_map<int, int> vans;
	std::unordered_map<int, int> deliveries;
};

class GameEnv {
    static const int _spread_out_distance = 10;
	DM_Client *_client;
	GameNodesTypes _gameNodesTypes;
	GameInfo _gameInfo;
	// active tasks assign to vans with specification
	// of the current goal type (pick-up:false, drop-off:true)
	ActiveTasks _activeTasks;
public:
	GameEnv(DM_Client *client){ _client = client; _activeTasks = ActiveTasks(); }
	~GameEnv(void);
	bool isTimeElapsed () { return (_gameInfo.time > 1440); }
	void spreadOut(void);
	void updateGameInfo(void);
	void startGame(void);
	void sendInstructions(InstructionsSet instuctions);
	void assignDeliveries(void /*probably something here*/); // assign a delivery to vans.
	void computeInstructions(void /*same thing here*/); // A* algorithm 
	void checkForAccidents(void);
	unsigned  __int8 euclideanDistance(Node node1, Node node2);
	unsigned  __int8 euclideanDistanceFast(Node node1, Node node2);
	unsigned  __int8 manhattanDistance(Node node1, Node node2);
	bool checkPath(Node start, Node end,
		unsigned  __int8 (GameEnv::*heuristic)(Node node1, Node node2));
private:
	void clearGameInfo(void); 
	int getFreeVanNumber(void); 
	__int8 getAvailableDelivery(void);
	void GameEnv::scheduleDeliveryTask(__int8 deliveryNum, __int8 vanNum);
	void GameEnv::removeTaskByDelivery(__int8 deliveryNum);
	void GameEnv::removeTaskByVan(__int8 vanNum);
	void GameEnv::removeTask(__int8 deliveryNum, __int8 vanNum);
	std::vector<Edge> getOutgoingEdges(Node fromNode);
	Path findRoad(Node start, Node end,
		unsigned  __int8 (GameEnv::*heuristic)(Node node1, Node node2));
};

#endif