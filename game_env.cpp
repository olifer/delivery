#include "game_env.h"
//#include "pqueue.h"
#include <algorithm>    // std::find
#include <functional>   // passing functions as parameters.
using namespace std;

/*----------------------------Constructors-----------------------------------*/
/*GameEnv::GameEnv(DM_Client *client){
	_client = client;
}*/
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

void GameEnv::assignDeliveries(void){
	updateGameInfo();
	//findRoad(make_pair(20,20),make_pair(20,20),&_gameNodesTypes, &GameEnv::euclideanDistance);
	/*Path road = findRoad(make_pair(10,20), 
				make_pair(30,40),&_gameNodesTypes, &GameEnv::euclideanDistanceFast); */
	if(!_gameInfo.waitingDeliveries.empty()){
		int i=0;
		//while(!_activeTasks.deliveries[i].count()){
			DeliveryInfo delivery = _gameInfo.waitingDeliveries[0];
			//if(_activeCargoFreeVans[]
			int van_num ; // we have delivery!
			if( van_num = getFreeVanNumber() != -1 ){
				// we have a free van, time to pick-up delivery!
			
				Path road = findRoad(_gameInfo.vans[van_num].location, 
					delivery.pickUp,&_gameNodesTypes, &GameEnv::euclideanDistanceFast); 
			}
		//}
	}
}

__int8 GameEnv::getAvailableDelivery(void){
	for(size_t i=0; i<_gameInfo.waitingDeliveries.size(); i++){
		//if(_activeTasks.deliveries[_gameInfo.waitingDeliveries[i].Number].count()){
		//}
	}

	return -1;
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

/* 
	Get outgoing edges from the node
*/
vector<Edge> GameEnv::getOutgoingEdges(Node fromNode){
	vector<Edge> edges;
	Node nextNode;
	Node loc;
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

Path GameEnv::findRoad(Node start, Node goal, GameNodesTypes* nodes, 
	unsigned  __int8 (GameEnv::*heuristic)(Node, Node)){
		vector<Edge> edges;
		edges.reserve(4);

		// road is formed by child->parent relation between edges
		unordered_map<Node, Node, hash_pair> road;
		// visited set = closed set + open set
		unordered_map<Node, NodeEntry, hash_pair> visited;
		// frontier
		priority_queue<NodeEntry, NodeEntryList, greater<NodeEntry>> open; 
		//priority_queue<NodeEntry, NodeEntryList, greater<NodeEntry>> openSet; 


		// Initialization
		Node current, next;
		NodeEntry entry;
		
		entry.computedCost = 0;
		entry.expectedTotalCost = (this->*heuristic)(start, goal);
		entry.node = start;
		entry.edge = Location(-1,-1);

		road[entry.edge] = Location(-1,-1);
		visited[entry.node] = entry;

		open.push(entry);

		// Heart of A*
		while(!open.empty()){
			// Pick the cheapest node in open set
			entry = open.top();
			open.pop(); 

			// Goal check
			if(entry.node == goal) { 
				break;
			}

			// Get possible edges
			edges.empty();
			edges = getOutgoingEdges(entry.node);

			for(vector<Edge>::iterator e = edges.begin(); e != edges.end(); e++){
				next = e->getNextNode(entry.node);

				if(!visited.count(next)){ // unvisited node
					NodeEntry newEntry = NodeEntry();
					newEntry.edge = e->getLocation();
					newEntry.node = next;
					newEntry.computedCost = entry.computedCost + e->getCost();
					newEntry.expectedTotalCost = newEntry.computedCost + (this->*heuristic)(newEntry.node, goal);
					// place to visited set
					visited[newEntry.node] = newEntry; //newEntry.edge.first == entry.edge.first && newEntry.edge.second == entry.edge.second
					// place to frontier
					open.push(newEntry);
					// keep track of road
					road[newEntry.edge] = entry.edge;
				} else { // visited node, does not matter if it either closed or open set
					int g = entry.computedCost + e->getCost();
					// check if we can improve the cost
					if(visited[next].computedCost > g){
						// no need to calculate the heuristic function again, since it 
						// remains the same for the specific node
						visited[next].expectedTotalCost = g + 
							(visited[next].expectedTotalCost - visited[next].computedCost);
						visited[next].computedCost = g;
						
						// update road
						visited[next].edge = e->getLocation();

						// place again to the open set. 
						// note: the same node could be located twice in the open set, but
						// still the smallest one will be chose. thereby, we avoid 
						// a searching operation across the open set.
						open.push(visited[next]);
					}
				}
			}
		}

		Path pathToGoal;
		pathToGoal.reserve(120);
		Location currentEdge = entry.edge; 
		// Make path
		while(currentEdge.first>=0){
			pathToGoal.push_back(currentEdge);
			currentEdge = road[currentEdge];
		}

		return pathToGoal;
}

bool GameEnv::checkPath(Node start, Node end, GameNodesTypes* nodes, 
	unsigned  __int8 (GameEnv::*heuristic)(Node node1, Node node2)) {
		bool isCorrect = true;
		// Get the path.
		Path path = GameEnv::findRoad(start, end, nodes, heuristic);

		for(Path::iterator edge = path.begin(); edge != path.end(); ++edge) {
			Path::iterator next = edge + 1; // get location of next edge in the matrix.
			if(next == path.end()) {//If it is the last edge of the path, need to check it reaches the end.
				vector<Edge> edgesFromEnd = getOutgoingEdges(end); // get edges from the end.
				bool reachesEnd = false;
				for(vector<Edge>::iterator e = edgesFromEnd.begin(); e != edgesFromEnd.end(); ++e) {
					// Check that one of those edges have same location as the last edge of the path
					// (i.e. that the path actually reaches the goal)
					reachesEnd |= ((e->getLocation().first == edge->first) & (e->getLocation().second == edge->second));
				}
				return isCorrect & reachesEnd;
			} 
			// Check location of edges: it should only move by 1 unit in X OR in Y (not both).
			int Y, nextY, X, nextX;
			Y = edge->first;
			nextY = next->first;
			X = edge->second;
			nextX = next->second;
			isCorrect &= (abs(Y - nextY) + (abs(X - nextX))) == 1; // 
		}
		// if I'm here something went wrong.
		return false;
}

/* Calculate idean distance between two nodes */
unsigned  __int8 GameEnv::euclideanDistance(Node node1, Node node2) {
	__int8 deltaY = node1.first - node2.first;
	__int8 deltaX = node1.second - node2.second;
	return (int) (sqrt(pow(deltaY,2.0)+pow(deltaX,2.0)) + 0.5 );
}

unsigned  __int8 GameEnv::euclideanDistanceFast(Node node1, Node node2) {
	__int8 deltaY = abs(node1.first - node2.first);
	__int8 deltaX = abs(node1.second - node2.second);
	return deltaY+deltaX; 
}

/* Calculate Manhattan distance between two nodes */
unsigned  __int8 GameEnv::manhattanDistance(Node node1, Node node2) {
	__int8 deltaY = abs(node1.first - node2.first);
	__int8 deltaX = abs(node1.second - node2.second);
	return deltaY+deltaX;
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
}
