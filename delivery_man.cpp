// delivery_man.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "common.h"

using namespace std;

struct GameInfo{
	int time;
	GameEdges edges;
	VanList vans;
	DeliveryList waitingDeliveries;
	DeliveryList activeDeliveries;
	vector<pair<int,int>> completedDeliveries;
	wstring output;
}; 

int main(int argc, _TCHAR* argv[])
{
	wstring output(L"");
	wstring group(L"3");
	bool status;

	GameNodes nodes;

	// kee current state
	GameInfo gameInfo;

	
	DM_Client client = DM_Client(group, status);
	client.startGame(nodes, output);

	client.getInformation(gameInfo.time, gameInfo.edges, gameInfo.vans, gameInfo.waitingDeliveries,
		gameInfo.activeDeliveries, gameInfo.completedDeliveries, gameInfo.output);
	
	return 0;
}

