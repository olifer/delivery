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
	GameEnv(DM_Client *client);
	~GameEnv(void);
	std::vector<Edge> getOutgoingEdges(Node fromNode);
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
};

#endif