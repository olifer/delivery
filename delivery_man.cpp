// delivery_man.cpp : Defines the entry point for the console application.
//

#include "game_env.h"
 #include <Windows.h>
using namespace std;


int main(int argc, _TCHAR* argv[])
{
	wstring group(L"3");
	bool status;
	// DM_Client from DLL (closed source).
	DM_Client client = DM_Client(group, status);

	GameEnv gameEnv(&client);
	gameEnv.startGame();
	// start timer
	time_t start = time(NULL);

	gameEnv.spreadOut();
	gameEnv.updateGameInfo();
	gameEnv.precomputeRoadTypes();
	/*vector<Edge> edges;
	edges.reserve(4);    
	edges = gameEnv.getOutgoingEdges(make_pair(20,20));
	edges = gameEnv.getOutgoingEdges(make_pair(0,0));
	edges = gameEnv.getOutgoingEdges(make_pair(40,40));*/
	//for(int i=0;i<100;i++){
	//bool isCorrect = gameEnv.checkPath(make_pair(0,0),make_pair(20,20), &GameEnv::euclideanDistance);
	
	// ALGO DEBUG FACILITIES
	/*while(gameEnv.getGameTime()<800){gameEnv.updateGameInfo();};
	DebugInfo estimates = gameEnv.TestAlgorithm(algo::dijkstra);
	estimates = gameEnv.TestAlgorithm(algo::euclideanDistance);
	estimates = gameEnv.TestAlgorithm(algo::manhattanDistance);
	estimates = gameEnv.TestAlgorithm(algo::repulsiveCenter);*/


	while(!gameEnv.isTimeElapsed()) {
		gameEnv.updateGameInfo();
		gameEnv.manageDefferedDeliveries();
		gameEnv.manageDeliveries();
		//gameEnv.assignDeliveries();
		gameEnv.assignDeliveriesParallel();
		//gameEnv.spreadOutFreeVans();
		

		// dummy loop
		// 1) Assign a delivery to each van.
		// 2) Compute shortest path for each van to it's assigned target point.
		// 3) Send instructions with path
		// 4) Update time. (eventually sleep, because the game state might not advance). 
		// 5) Check for accidents and do stuff.
		//gameEnv.assignDeliveries(); // Check for new deliveries and assign van.
		//gameEnv.computeInstructions(); // Compute shortest paths to either 
		// gameEnv.sendInstructions(Instructions);
		
		//gameEnv.checkForAccidents();
	}
	gameEnv.~GameEnv();

	return 0;
}

