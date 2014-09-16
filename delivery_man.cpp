// delivery_man.cpp : Defines the entry point for the console application.
//

#include "game_env.h"

using namespace std;


int main(int argc, _TCHAR* argv[])
{
	wstring group(L"3");
	bool status;
	DM_Client client = DM_Client(group, status);

	GameEnv gameEnv(&client);
	gameEnv.startGame();

	gameEnv.spreadOut();

	for(int i=0;i<100;i++){
		// dummy loop
		gameEnv.updateGameInfo();
	}

	return 0;
}

