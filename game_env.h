#ifndef COMMON_H
#define COMMON_H
#include "common.h"
#endif

#ifndef EDGE_H
#define EDGE_H
#include "edge.h"
#endif

class GameEnv {
    static const int _spread_out_distance = 10;
	DM_Client *_client;
	GameNodes _gameNodes;
	GameInfo _gameInfo;
public:
	GameEnv(DM_Client *client);
	~GameEnv(void);
	std::vector<Edge> getEdges(Node fromNode);
	void spreadOut(void);
	void updateGameInfo(void);
	void startGame(void);
	void sendInstructions(InstructionsSet instuctions);
	
}; 
