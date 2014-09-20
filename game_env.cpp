#include "game_env.h"
//#include <algorithm>    // std::find
using namespace std;

/*----------------------------Interfaces-------------------------------------*/
/* 
*	Should be run first.
*/
void GameEnv::startGame(void){
	wstring output;
	_client->startGame(_gameNodesTypes, output);
}
/* 
*	Send instructions to the van.
*/

void GameEnv::sendInstructionsToVan(uint8_t van, Path instructions){
	wstring output;
	InstructionsSet instructionsSet;
	instructionsSet[van].reserve(instructions.size());
	instructionsSet[van] = instructions;
	_client->sendInstructions(instructionsSet, output);
}
/* 
*	Send instructions to the vans.
*/
void GameEnv::sendInstructions(InstructionsSet sendInstructions){
	wstring output;
	_client->sendInstructions(sendInstructions, output);
}
/* 
*	Update game information.
*/
void GameEnv::updateGameInfo(void){
	clearGameInfo();
	_client->getInformation(_gameInfo.time, _gameInfo.edges, _gameInfo.vans,
		_gameInfo.waitingDeliveries, _gameInfo.activeDeliveries, 
		_gameInfo.completedDeliveries, _gameInfo.output);
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

void GameEnv::manageDeliveries(void){
	Location vanLoc, pickupLoc, dropoffLoc, goal;
	uint8_t deliveryNum, vanNumber;

	for(uint8_t i=0; i<N_VANS; i++){
		if(_gameInfo.vans[i].cargo != -1 && _activeTasks.vans.count(i)) {
			vanNumber = _gameInfo.vans[i].Number;
			vanLoc = _gameInfo.vans[i].location;
			deliveryNum = _activeTasks.vans[vanNumber];
			//deliveryNum = _gameInfo.vans[i].cargo;//_activeTasks.vans[vanNumber];
			pickupLoc = _activeTasks.del2pickup[deliveryNum];
			if(deliveryNum == _gameInfo.vans[i].cargo){ // the delivery is picked up!
				removeTask(deliveryNum, vanNumber);
				//dropoffLoc = 
			} else { // accidental pickup
				// conflict delivery
				deliveryNum = _gameInfo.vans[i].cargo;//_activeTasks.pickup2del[vanLoc];
				uint8_t conflictVan = 
					_activeTasks.deliveries[deliveryNum]; 
				// clean instructions of the conflict van
				sendInstructionsToVan(conflictVan, vector<Location>());
				removeTask(deliveryNum, conflictVan);
			}

			dropoffLoc = getDropOff(deliveryNum);
			if(dropoffLoc.first != -1){ 
				// find and send drop off instructions to the van
				Path road = findRoad(vanLoc, 
					dropoffLoc, &GameEnv::euclideanDistanceFast); 
				sendInstructionsToVan(vanNumber, road);
			} else {  /* otherwise something went wrong */ }
		}
	}
}

Node GameEnv::getDropOff(uint8_t deliveryNum){
	auto pred = [deliveryNum](const DeliveryInfo &delivery) {
		return delivery.Number == deliveryNum;
	};

	DeliveryList::iterator deliveryInfo = 
		find_if(_gameInfo.activeDeliveries.begin(), 
			_gameInfo.activeDeliveries.end(), pred);

	if(deliveryInfo == _gameInfo.activeDeliveries.end()){
		return make_pair(-1,-1);
	} else {
		return deliveryInfo[0].dropOff;
	}
}

void GameEnv::assignDeliveries(void){
	//updateGameInfo();
	if(!_gameInfo.waitingDeliveries.empty()){
		DeliveryInfo *deliveryInfo;
		VanInfo* vanInfo;
		//uint8_t deliveryNum, vanNum;
		if((deliveryInfo=getAvailableDelivery()) != NULL &&
				(vanInfo = getFreeVanNumber(deliveryInfo->pickUp)) != NULL){
			Location pickUp	= deliveryInfo->pickUp;
			Location vanLoc	= vanInfo->location;
			uint8_t deliveryNum	= deliveryInfo->Number;			
			uint8_t vanNum	= vanInfo->Number;
			if(scheduleDeliveryTask(deliveryNum, vanNum)){
				//Location pickUp = _activeTasks.del2pickup[deliveryNum];
				// we have a free van, time to pick-up delivery!
				Path road = findRoad(vanLoc, pickUp, 
						&GameEnv::euclideanDistanceFast); 
				sendInstructionsToVan(vanNum, road);
			}
		}
	}
}


bool GameEnv::scheduleDeliveryTask(__int8 deliveryNum, __int8 vanNum){
	Location pickUp = make_pair(-1,-1);
	for(uint8_t i=0; i<_gameInfo.waitingDeliveries.size(); i++){
		if(_gameInfo.waitingDeliveries[i].Number == deliveryNum) {
			pickUp = _gameInfo.waitingDeliveries[i].pickUp;
			break;
		}
	}
	if(pickUp.first == -1){ // no delivery found
		return false;
	}

	_activeTasks.deliveries[deliveryNum] = vanNum;
	_activeTasks.vans[vanNum] = deliveryNum;
	_activeTasks.pickup2del[pickUp] = deliveryNum;
	_activeTasks.del2pickup[deliveryNum] = pickUp;

	return true;
}

void GameEnv::removeTaskByDelivery(__int8 deliveryNum){
	__int8 vanNum = _activeTasks.deliveries[deliveryNum];
	removeTask(deliveryNum, vanNum);
}

void GameEnv::removeTask(__int8 deliveryNum, __int8 vanNum){
	_activeTasks.vans.erase(vanNum);
	_activeTasks.deliveries.erase(deliveryNum);
	Location location = _activeTasks.del2pickup[deliveryNum];
	_activeTasks.pickup2del.erase(location);
	_activeTasks.del2pickup.erase(deliveryNum);
}

/* Retrieve the oldest dilivery that is not a pick-up goal for any van */
DeliveryInfo* GameEnv::getAvailableDelivery(void){
	int maxTime = G_END_TIME;
	DeliveryInfo* deliveryInfo = NULL;
	for(size_t i=0; i<_gameInfo.waitingDeliveries.size(); i++){
		if(!_activeTasks.deliveries.count(_gameInfo.waitingDeliveries[i].Number)){
			if(_gameInfo.waitingDeliveries[i].Time < maxTime){
				maxTime = _gameInfo.waitingDeliveries[i].Time;
				deliveryInfo = &_gameInfo.waitingDeliveries[i];
			}
		}
	}	

	return deliveryInfo;
}
/* Retrieve the nearest free van to the goal loaction */
VanInfo* GameEnv::getFreeVanNumber(Node goal) {
	/*auto pred = [](const VanInfo &van) {
		return van.instructions.empty() && van.cargo == -1;
	};

	VanList::iterator free_vans = 
		find_if(_gameInfo.vans.begin(), _gameInfo.vans.end(), pred);
	*/
	VanInfo* nearestVan = NULL;
	uint8_t nearestDist, dist;
	nearestDist = 100;
	for(uint8_t i=0; i<N_VANS; i++){
		if(_gameInfo.vans[i].instructions.empty() 
				&& _gameInfo.vans[i].cargo == -1){
				dist = euclideanDistanceFast(_gameInfo.vans[i].location, goal);
				if(dist<nearestDist){
					nearestDist = dist;
					nearestVan = &_gameInfo.vans[i];
				}
		}
	}

	return nearestVan;

	/*if(free_vans == _gameInfo.vans.end()){
		return -1;
	}

	for(Edge::edge_itr e = edgesFromEnd.begin(); 
					e != edgesFromEnd.end(); ++e)

	return *free_vans;*/
	/* temporal */
	/*if(_gameInfo.vans[0].instructions.empty() && _gameInfo.vans[0].cargo == -1){
		return 0;
	} else {
		return -1;
	}*/
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

Path GameEnv::findRoad(Node start, Node goal, h_func heuristic){
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

			for(Edge::edge_itr e = edges.begin(); e != edges.end(); e++){
				next = e->getNextNode(entry.node);

				if(!visited.count(next)){ // unvisited node
					NodeEntry newEntry = NodeEntry();
					newEntry.edge = e->getLocation();
					newEntry.node = next;
					newEntry.computedCost = entry.computedCost + e->getCost();
					newEntry.expectedTotalCost = newEntry.computedCost + 
						(this->*heuristic)(newEntry.node, goal);
					// place to visited set
					visited[newEntry.node] = newEntry; 
					//newEntry.edge.first == entry.edge.first 
					// && newEntry.edge.second == entry.edge.second
					// place to frontier
					open.push(newEntry);
					// keep track of road
					road[newEntry.edge] = entry.edge;
				} else if(0){ 
					// visited node, does not matter if it either in 
					// the closed or open set
					int g = entry.computedCost + e->getCost();
					// check if we can improve the cost
					if(visited[next].computedCost > g){
						// no need to calculate the heuristic function again,
						// since it remains the same for the specific node
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
		pathToGoal.reserve(G_MATRIX_LENGTH*3);
		Location currentEdge = entry.edge; 
		// Make path
		while(currentEdge.first>=0){
			pathToGoal.push_back(currentEdge);
			currentEdge = road[currentEdge];
		}

		reverse(pathToGoal.begin(),pathToGoal.end());

		return pathToGoal;
}

bool GameEnv::checkPath(Node start, Node end, h_func heuristic) {
		bool isCorrect = true;
		// Get the path.
		Path path = findRoad(start, end, heuristic);

		for(Path::iterator edge = path.begin(); edge != path.end(); ++edge) {
			// get location of next edge in the matrix.
			Path::iterator next = edge + 1; 
			// If it is the last edge of the path, need to check it reaches 
			// the end.
			if(next == path.end()) {
				// get edges from the end.
				vector<Edge> edgesFromEnd = getOutgoingEdges(end); 
				bool reachesEnd = false;
				for(Edge::edge_itr e = edgesFromEnd.begin(); 
					e != edgesFromEnd.end(); ++e) {
					// Check that one of those edges have same location as the 
				    // last edge of the path
					// (i.e. that the path actually reaches the goal)
					reachesEnd |= ((e->getLocation().first == edge->first) 
						& (e->getLocation().second == edge->second));
				}
				return isCorrect & reachesEnd;
			} 
			// Check location of edges: it should only move by 1 unit
			// in X OR in Y (not both). - wrong assumption
			//isCorrect &= (abs(Y - nextY) + (abs(X - nextX))) == 1; // 
			isCorrect &= isAdjacent(*edge, *next); 
		}
		// if I'm here something went wrong.
		return false;
}
bool GameEnv::isAdjacent(Location edge1, Location edge2){
	bool result = true;
	int Y,X,nextY,nextX,dX, dY;

	Y = edge1.first;
	nextY = edge2.first;
	X = edge1.second;
	nextX = edge2.second;
	dX = abs(edge1.second - edge2.second);
	dY = abs(edge1.first - edge2.first);

	if( Y%2==0 && nextY%2==0){// -- horizontal plane
		result = (dY == 0 && dX == 1);
	} 
	else if(Y%2==0 || nextY%2==0){// |_ perpendicular
		result = (dY == 1 && dX == 0) || (dY == 1 && dX == 1);
	} else {
		// edges are in vertical plane
		result = (dY == 2 && dX == 0);
	}
	return result;
}
/* Calculate idean distance between two nodes */
uint8_t GameEnv::euclideanDistance(Node node1, Node node2) {
	__int8 deltaY = node1.first - node2.first;
	__int8 deltaX = node1.second - node2.second;
	return (int) (sqrt(pow(deltaY,2.0)+pow(deltaX,2.0)) + 0.5 );
}

uint8_t GameEnv::euclideanDistanceFast(Node node1, Node node2) {
	__int8 deltaY = abs(node1.first - node2.first);
	__int8 deltaX = abs(node1.second - node2.second);
	return deltaY+deltaX; 
}

/* Calculate Manhattan distance between two nodes */
uint8_t GameEnv::manhattanDistance(Node node1, Node node2) {
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
