#include "game_env.h"
#include <ppl.h>
//#include <algorithm>    // std::find
using namespace std;
using namespace Concurrency;

/* Calculate idean distance between two nodes */
uint8_t GameEnv::euclideanDistance(Node node1, Node node2) {
	__int8 deltaY = node1.first - node2.first;
	__int8 deltaX = node1.second - node2.second;
	return (int) (sqrt(pow(deltaY,2.0)+pow(deltaX,2.0)) + 0.5 );
}

uint8_t GameEnv::repulsiveCenter(Node node1, Node node2) {
	uint8_t centerCost = std::max((uint8_t) 100, manhattanDistance(node1, make_pair(20,20)));
	return (manhattanDistance(node1, node2) + (1 / centerCost));
}
/* Calculate idean distance between two nodes */
uint8_t GameEnv::roadBase(Node node1, Node node2) {
	return (int) (euclideanDistance(node1, node2) + 0.5 );
}

/* Calculate Manhattan distance between two nodes */
uint8_t GameEnv::manhattanDistance(Node node1, Node node2) {
	__int8 deltaY = abs(node1.first - node2.first);
	__int8 deltaX = abs(node1.second - node2.second);
	return deltaY+deltaX;
}

/* Calculate Manhattan distance between two nodes */
uint8_t GameEnv::dijkstra(Node node1, Node node2) {
	return 0;
}

// USE HEURISTIC HERE
Path GameEnv::findPath(Node start, Node end){
	// manhattanDistance repulsiveCenter euclideanDistance dijkstra
	return findRoad(start, end, &GameEnv::repulsiveCenter); //
}

Path GameEnv::findPathDebug(Node start, Node end, int h_func){
	// manhattanDistance repulsiveCenter euclideanDistance dijkstra
	
	switch(h_func){
		case algo::dijkstra: 
			return findRoad(start, end, &GameEnv::dijkstra);
		case algo::euclideanDistance: 
			return findRoad(start, end, &GameEnv::euclideanDistance);
		case algo::manhattanDistance: 
			return findRoad(start, end, &GameEnv::manhattanDistance);
		case algo::repulsiveCenter: 
			return findRoad(start, end, &GameEnv::repulsiveCenter);
	}
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
	for(int i=0; i<this->_spread_out_distance; i++){
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
				if(_activeTasks.deliveries.count(deliveryNum)) {
					uint8_t conflictVan = 
						_activeTasks.deliveries[deliveryNum]; 
					// clean instructions of the conflict van
					sendInstructionsToVan(conflictVan, vector<Location>());
					sendInstructionsToVan(vanNumber, vector<Location>());
					removeTask(deliveryNum, conflictVan);
				}
				addDefferedDelivery(deliveryNum, vanNumber);
				return;
			}

			dropoffLoc = getDropOff(deliveryNum);
			if(dropoffLoc.first != -1){ 
				// find and send drop off instructions to the van
				Path road = findPath(vanLoc, dropoffLoc); 
				sendInstructionsToVan(vanNumber, road);
			} else {  /* otherwise something went wrong */ }
		}
	}
}

void GameEnv::addDefferedDelivery(int deliveryNum, int vanNum){
	_activeTasks.deferredDeliveries[deliveryNum] = vanNum;
	_activeTasks.deferredVans[vanNum] = deliveryNum;
}

Node GameEnv::getDropOff(uint8_t deliveryNum){
	auto pred = [deliveryNum](const DeliveryInfo &delivery) {
		return delivery.Number == deliveryNum;
	};

	DeliveryList::iterator deliveryInfo = 
		find_if(_gameInfo.activeDeliveries.begin(), 
			_gameInfo.activeDeliveries.end(), pred);
	// remove else
	if(deliveryInfo == _gameInfo.activeDeliveries.end()){
		return make_pair(-1,-1);
	} 
	return deliveryInfo[0].dropOff;
}

void GameEnv::assignDeliveriesParallel(void){
	vector<VanInfo> assignments_v;
	vector<DeliveryInfo> assignments_d;
	if(!_gameInfo.waitingDeliveries.empty()){
		DeliveryInfo *deliveryInfo;
		VanInfo* vanInfo;

		while((deliveryInfo=getAvailableDelivery()) != NULL &&
				(vanInfo = getFreeVanNumber(deliveryInfo->pickUp)) != NULL){
			if(scheduleDeliveryTask(deliveryInfo->Number, vanInfo->Number)){
				assignments_v.push_back(*vanInfo);
				assignments_d.push_back(*deliveryInfo);
			}
		}
	}

	if(assignments_v.size()>1){
		parallel_for(size_t(0), assignments_v.size(), [&](size_t i) {
			Path road = findPath(assignments_v[i].location, 
				assignments_d[i].pickUp); 
			sendInstructionsToVan(assignments_v[i].Number, road);
		});
	} else if(!assignments_v.empty()){
		Path road = findPath(assignments_v[0].location, 
			assignments_d[0].pickUp); 
		sendInstructionsToVan(assignments_v[0].Number, road);
	}
}
void GameEnv::assignDeliveries(void){
	//updateGameInfo();
	if(!_gameInfo.waitingDeliveries.empty()){
		DeliveryInfo *deliveryInfo;
		VanInfo* vanInfo;
		//uint8_t deliveryNum, vanNum;
		while((deliveryInfo=getAvailableDelivery()) != NULL &&
				(vanInfo = getFreeVanNumber(deliveryInfo->pickUp)) != NULL){
			uint8_t vanNum	= vanInfo->Number;
			Location pickUp	= deliveryInfo->pickUp;
			uint8_t deliveryNum	= deliveryInfo->Number;		

			/*if(!vanInfo->instructions.empty()){
				if(scheduleDeliveryTask(deliveryNum, vanNum)){
					_activeTasks.deferred[vanNum] = pickUp;
					continue;
				}
			}*/
			
			Location vanLoc	= vanInfo->location;
				
			/*if(!vanInfo->instructions.empty()){ // deffered task
				_activeTasks.deferred[vanNum] = pickUp;
				sendInstructionsToVan(vanNum, vector<Location>());
				continue;
			}*/
			if(scheduleDeliveryTask(deliveryNum, vanNum)){
				//Location pickUp = _activeTasks.del2pickup[deliveryNum];
				// we have a free van, time to pick-up delivery!
				Path road = findPath(vanLoc, pickUp); 
				sendInstructionsToVan(vanNum, road);
			}
		}
	}
}

void GameEnv::manageDefferedDeliveries(void){
	int vanNum, deliveryNum, i = 0;
	vector<int> numList;
	
	if(_activeTasks.deferredVans.empty()){
		return;
	}

	for (auto it = _activeTasks.deferredVans.begin(); it != _activeTasks.deferredVans.end(); ++it) {
		if(_gameInfo.vans[it->first].instructions.empty()){
			vanNum = it->first;
			deliveryNum = it->second;
			numList.push_back(vanNum);
			Location dropoffLoc = getDropOff(deliveryNum);
			Path road = findPath(_gameInfo.vans[vanNum].location, 
				dropoffLoc); 
			sendInstructionsToVan(vanNum, road);
			
			i++;
		}
	}
	for(uint8_t j=0; j<i; j++){
		deliveryNum = _activeTasks.deferredVans[numList[j]];
		_activeTasks.deferredDeliveries.erase(deliveryNum);
		_activeTasks.deferredVans.erase(numList[j]);
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
	_activeTasks.deliveries.size();

	if(_activeTasks.deliveries.size() !=_activeTasks.vans.size()){
		_activeTasks.deliveries[deliveryNum] = vanNum;
		return false;
	}
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

	/*
	if(_activeTasks.deliveries.size() !=_activeTasks.vans.size()){
		_activeTasks.deliveries[deliveryNum] = vanNum;
	}*/
}

/* Retrieve the oldest dilivery that is not a pick-up goal for any van */
DeliveryInfo* GameEnv::getAvailableDelivery(void){
	int deliveryNum, maxTime = G_END_TIME;
	DeliveryInfo* deliveryInfo = NULL;
	for(size_t i=0; i<_gameInfo.waitingDeliveries.size(); i++){
		deliveryNum = _gameInfo.waitingDeliveries[i].Number;
		if(!_activeTasks.deliveries.count(deliveryNum)
			&& !_activeTasks.deferredDeliveries.count(deliveryNum)){
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
		if(/*_gameInfo.vans[i].instructions.empty() */
			!_activeTasks.vans.count(_gameInfo.vans[i].Number) 
				&& _gameInfo.vans[i].cargo == -1
				&& !_activeTasks.deferredVans.count(_gameInfo.vans[i].Number)){
				dist = manhattanDistance(_gameInfo.vans[i].location, goal);
				if(dist<nearestDist){
					nearestDist = dist;
					nearestVan = &_gameInfo.vans[i];
				}
		}
	}

	return nearestVan;
}

void GameEnv::spreadOutFreeVans(void){
	// Vans coming back home after deliveries.
	Node anchor, vanLoc;
	int vanNum;
	if(_gameInfo.time<420){
		return;
	}

	for(uint8_t i=0; i<N_VANS; i++){
		vanNum = _gameInfo.vans[i].Number;
		if(!_activeTasks.vans.count(vanNum) 
			&& _gameInfo.vans[i].instructions.empty() 
			&& _gameInfo.vans[i].cargo == -1){
				vanLoc = _gameInfo.vans[i].location;
				if(isAnchor(vanLoc)){// if its in the base, stop
					continue;
				}
				anchor = findNearestAnchor(vanLoc);
				Path road = findRoad(vanLoc, anchor, 
						&GameEnv::euclideanDistance); 
				sendInstructionsToVan(vanNum, road);
		}
	}
}

bool GameEnv::isAnchor(Node node){
	for(uint8_t i=0; i<_anchors.size(); i++){
		if(node == _anchors[i]){
			return true;
		}
	}
	return false;
}

Node GameEnv::findNearestAnchor(Node node){
	Node nearest;
	int dist, min = 100;
	for(uint8_t i=0; i<_anchors.size(); i++){
		dist = manhattanDistance(node, _anchors[i]);
		if(dist<min){
			min = dist;
			nearest = _anchors[i];
		}
	}
	return nearest;
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
		// tree of visited nodes
		unordered_map<Node, Node, hash_pair> road;
		// visited set = closed set + open set
		unordered_map<Node, NodeEntry, hash_pair> visited;
		// dob't look at that now
		unordered_map<Node, bool, hash_pair> openTracker;
		// frontier
		priority_queue<NodeEntry, NodeEntryList, greater<NodeEntry>> open; 

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

		openTracker[entry.edge] = false;

		// Heart of A*
		while(!open.empty()){
			// Pick the cheapest node in open set
			entry = open.top();
			open.pop(); 


			// Goal check
			if(entry.node == goal) { 
				break;
			}

			if(openTracker.count(entry.edge)){
				openTracker.erase(entry.edge);
			} else {
				continue;
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
						/*/ _edgesTypes[newEntry.edge.first][newEntry.edge.second]+0.5);*/
					// place to visited set
					visited[newEntry.node] = newEntry; 
					//newEntry.edge.first == entry.edge.first 
					// && newEntry.edge.second == entry.edge.second
					// place to frontier
					open.push(newEntry);
					openTracker[newEntry.edge] = false;
					// keep track of road
					road[newEntry.edge] = entry.edge;
				} else { //if(0)
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
						road[visited[next].edge] = entry.edge;

						// place again to the open set. 
						// note: the same node could be located twice in the open set, but
						// still the smallest one will be chose. thereby, we avoid 
						// a searching operation across the open set.
						open.push(visited[next]);
						openTracker[visited[next].edge] = true;
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

		this->cost = entry.computedCost;
		return pathToGoal;
}

/* return <total time, avg time>*/
DebugInfo GameEnv::TestAlgorithm(int h_func) {
	DebugInfo info= DebugInfo();
	// constant distribution
	Location arr[] = {std::make_pair(5, 40), std::make_pair(30, 4),
		std::make_pair(7, 39),std::make_pair(7, 38),std::make_pair(37, 10)
		,std::make_pair(8, 29),std::make_pair(14, 0),std::make_pair(17, 36),
		std::make_pair(0, 0), std::make_pair(7, 39),std::make_pair(7, 38),
		std::make_pair(37, 10),std::make_pair(20, 31),std::make_pair(31, 19),
		std::make_pair(21, 28),std::make_pair(0, 0),std::make_pair(40, 13)};
	
	int sumOfCosts = 0, dummy[] = {73,72,3,68,70,57,69,75,74,3,68,50,38,32,77};
	static int sumOfOptimalCosts = 1;
	vector<int> optimal_costs(dummy,dummy+15);//sizeof(arr)/sizeof(make_pair<int,int>)
	vector<int> costs;
	Path path;

	updateGameInfo();
	int elapsed_game_units=_gameInfo.time;
	clock_t begin = clock();
	int size = 17;
	for(int i=1; i < size; i++){
		path = findPathDebug(arr[i-1],arr[i],h_func);
		optimal_costs.push_back(cost);
		sumOfCosts += cost;
	}
	clock_t end = clock();
	updateGameInfo();
	if(h_func == algo::dijkstra){
		sumOfOptimalCosts = sumOfCosts;
	}
	info.avg_game_units=double (_gameInfo.time - elapsed_game_units)/(size-1);
	info.elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	info.avg = info.elapsed_secs/(size-1);
	info.quality = 2.0-(double)sumOfCosts/sumOfOptimalCosts;
	info.avg_time = (double)sumOfCosts/((size-1)/2);
	return info;
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
void GameEnv::precomputeRoadTypes(void){
	_edgesTypes = (_gameInfo.edges);
	for(uint8_t i=0; i<_edgesTypes.size(); i++ ){
		vector<int> row;
		for(uint8_t j=0; j<_edgesTypes[i].size(); j++ ){
			switch(_edgesTypes[i][j]){
				case 1: _edgesTypes[i][j]=3; break; //highway
				case 3: _edgesTypes[i][j]=2; break; //suburban
				case 4:	_edgesTypes[i][j]=1; break; //urban
			}
		}
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
