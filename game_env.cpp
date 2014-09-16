#define GAME_ENV_H
#include "game_env.h"

using namespace std;

/*----------------------------Constructors-----------------------------------*/
GameEnv::GameEnv(DM_Client *client){
	_client = client;
}

/*----------------------------Interfaces-------------------------------------*/
/* 
*	Should be run first.
*/
void GameEnv::startGame(void){
	wstring output;
	_client->startGame(_gameNodes, output);
}
/* 
*	Send instructions to the vans.
*/
void GameEnv::sendInstructions(InstructionsSet sendInstructions){
	wstring output;
	_client->sendInstructions(sendInstructions, output);
}
/* 
*	Update game information. //ASK: How frequently?
*/
void GameEnv::updateGameInfo(void){
	_gameInfo.vans.clear();
	_client->getInformation(_gameInfo.time, _gameInfo.edges, _gameInfo.vans, _gameInfo.waitingDeliveries,
			_gameInfo.activeDeliveries, _gameInfo.completedDeliveries, _gameInfo.output);
}

/* 
	Initial spread out for the vans. Should be called only once, 
	immediately after the game start 
*/
void GameEnv::spreadOut(void){
	InstructionsSet instructionsSet;

	updateGameInfo();

	// the first van stays in the center
	for(int i=0; i<_spread_out_distance; i++){
		instructionsSet[1].push_back(make_pair(40-2*i-1,20));
		instructionsSet[2].push_back(make_pair(40,20+i));
		instructionsSet[3].push_back(make_pair(42+2*i-1,20));
		instructionsSet[4].push_back(make_pair(40,19-i)); 
	}

	sendInstructions(instructionsSet);
}



/*
vector<Edge> GameEnv::getEdges(Node fromNode){
	vector<Edge> Edges;
	Node nextNode;
	Location loc;
	int cost;

	if(fromNode.second > 0){ // check node above fromNode
		nextNode = make_pair(fromNode.first+1,fromNode.second);
		loc = make_pair(2*fromNode.first-1, fromNode.second);
		cost = (*_gameEdgesCosts)[loc.second][loc.first];
		Edges.push_back(Edge (fromNode, nextNode, cost, loc));
	}
	// TODO below node, right node, and left node.
	//Edges.push_back();
}*/

/*----------------------------Clean-up---------------------------------------*/

GameEnv::~GameEnv(){
}
