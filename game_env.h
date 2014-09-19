#ifndef GAME_ENV_H
#define GAME_ENV_H

#include "common.h"
#include "edge.h"

class GameEnv {
    static const int _spread_out_distance = 10;
	DM_Client *_client;
	GameNodesTypes _gameNodesTypes;
	GameInfo _gameInfo;
public:
	GameEnv(DM_Client *client){ _client = client; }
	~GameEnv(void);
	bool isTimeElapsed () { return (_gameInfo.time > 1440); }
	void spreadOut(void);
	void updateGameInfo(void);
	void startGame(void);
	void sendInstructions(InstructionsSet instuctions);
	void assignDeliveries(void /*probably something here*/); // assign a delivery to vans.
	void computeInstructions(void /*same thing here*/); // A* algorithm 
	void checkForAccidents(void);
private:
	void clearGameInfo(void); 
	int getFreeVanNumber(void); 
	std::vector<Edge> getOutgoingEdges(Node fromNode);
	NodeEntry findRoad(Node start, Node end, GameNodesTypes* nodes,
		unsigned  __int8 (GameEnv::*heuristic)(Node node1, Node node2));
	unsigned  __int8 euclideanDistance(Node node1, Node node2);
	unsigned  __int8 euclideanDistanceFast(Node node1, Node node2);
	unsigned  __int8 manhattanDistance(Node node1, Node node2);
};

#endif