#include "game_env.h"
#include <algorithm>    // std::find
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
	_client->startGame(_gameNodesTypes, output);
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
	clearGameInfo();
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
	Get outgoing edges from the node
*/
vector<Edge> GameEnv::getOutgoingEdges(Node fromNode){
	vector<Edge> edges;
	Node nextNode;
	Location loc;
	int cost;

	edges.reserve(4);    
	// Let <y,x> be the node coordinate:
	// Edge below: <2 y + 1,x>
	//		above: <2 y - 1,x>
	//		left:  <2 y, x - 1>
	//		right: <2 y, x>
	if(fromNode.first > 0){ // check edge above fromNode
		nextNode = make_pair(fromNode.first-1, fromNode.second);
		loc = make_pair(2*fromNode.first-1, fromNode.second);
		cost = _gameInfo.edges[loc.first][loc.second];
		edges.push_back(Edge (fromNode, nextNode, cost, loc));
	}

	if(fromNode.first < G_MATRIX_WIDTH ) { // check edge below fromNode
		nextNode = make_pair(fromNode.first+1,fromNode.second);
		loc = make_pair(2*fromNode.first+1, fromNode.second);
		cost = _gameInfo.edges[loc.first][loc.second];
		edges.push_back(Edge (fromNode, nextNode, cost, loc));
	}

	if(fromNode.second < G_MATRIX_LENGTH) { // check edge to the right fromNode
		nextNode = make_pair(fromNode.first,fromNode.second+1);
		loc = make_pair(2*fromNode.first, fromNode.second);
		cost = _gameInfo.edges[loc.first][loc.second];
		edges.push_back(Edge (fromNode, nextNode, cost, loc));
	}

	if(fromNode.second > 0) { // check edge to the right fromNode
		nextNode = make_pair(fromNode.first,fromNode.second-1);
		loc = make_pair(2*fromNode.first, fromNode.second-1);
		cost = _gameInfo.edges[loc.first][loc.second];
		edges.push_back(Edge (fromNode, nextNode, cost, loc));
	}
	return edges;
}

void GameEnv::assignDeliveries(void){
	updateGameInfo();
	if(!_gameInfo.waitingDeliveries.empty()){
		int van_num ; // we have delivery!
		if( van_num = getFreeVanNumber() != -1 ){
			// we have a free van, time to pick-up delivery!
		}
	}
}

/* Return Free Van*/
int GameEnv::getFreeVanNumber(void) {
	
	/*auto pred = [](const VanInfo &van) {
		return van.instructions.empty() && van.cargo == -1;
	};

	VanList::iterator free_vans = 
		find_if(_gameInfo.vans.begin(), _gameInfo.vans.end(), pred);

	return *free_vans;*/
	/* temporal */
	if(_gameInfo.vans[0].instructions.empty() && _gameInfo.vans[0].cargo == -1){
		return 0;
	} else {
		return -1;
	}
}
/*-------------------------Private Interfaces--------------------------------*/
void GameEnv::clearGameInfo(void){
	_gameInfo.vans.clear();
	_gameInfo.edges.clear();
	_gameInfo.waitingDeliveries.clear();
	_gameInfo.activeDeliveries.clear();
	_gameInfo.completedDeliveries.clear();
}
/*----------------------------Clean-up---------------------------------------*/

GameEnv::~GameEnv(){
	delete _client;
}
