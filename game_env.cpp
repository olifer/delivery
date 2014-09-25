#include "game_env.h"
#include <ppl.h>
//#include <algorithm>    // std::find
using namespace std;
using namespace Concurrency;

/*-----------------HEURISTICS------------------------------------------------*/
/* Calculate idean distance between two nodes */
uint8_t GameEnv::euclideanDistance(Node node1, Node node2) {
	__int8 deltaY = node1.first - node2.first;
	__int8 deltaX = node1.second - node2.second;
	return (int) (sqrt(pow(deltaY,2.0)+pow(deltaX,2.0)) + 0.5 );
}

uint8_t GameEnv::repulsiveCenter(Node node1, Node node2) {
	// Cost of going away from center.
	uint8_t centerCost = manhattanDistance(node1, make_pair(20,20));
	return (manhattanDistance(node1, node2) + (1 / (std::max(centerCost, (uint8_t) 1))) );
}
/* Calculate ideal distance between two nodes */
uint8_t GameEnv::roadBase(Node node1, Node node2) {
	return (int) (euclideanDistance(node1, node2) + 0.5 );
}

/* Calculate Manhattan distance between two nodes */
uint8_t GameEnv::manhattanDistanceWeighted(Node node1, Node node2) {
	__int8 deltaY = abs(node1.first - node2.first);
	__int8 deltaX = abs(node1.second - node2.second);
	return (deltaY+(2*deltaX));
}

/* Calculate Manhattan distance between two nodes */
uint8_t GameEnv::manhattanDistance(Node node1, Node node2) {
	__int8 deltaY = abs(node1.first - node2.first);
	__int8 deltaX = abs(node1.second - node2.second);
	return (deltaY+deltaX);
}

/* Calculate Manhattan distance between two nodes */
uint8_t GameEnv::dijkstra(Node node1, Node node2) {
	return 0;
}

// USE HEURISTIC HERE
Path GameEnv::findPath(Node& start, Node& end){
	// manhattanDistance repulsiveCenter euclideanDistance dijkstra
	//return findRoad(start, end, &GameEnv::repulsiveCenter); 
	return findRoadOptimized(start, end, &GameEnv::dijkstra);
}

/*----------------------------Interfaces-------------------------------------*/
/* 
*	Should be run first. 
*	Throws a runtime_error exception if the game is not started.
*/
void GameEnv::startGame(void){
	wstring output, success(L"201"),temp;
	_client->startGame(_gameNodesTypes, output);

	if(output.substr(0,3)!=success){
		throw std::runtime_error( "The game could not be started!" );
	}
}
/*-----------------GAME ROUTINE----------------------------------------------*/
/* Update game information */
void GameEnv::updateGameInfo(void){
	clearGameInfo();
	_client->getInformation(_gameInfo.time, _gameInfo.edges, _gameInfo.vans,
		_gameInfo.waitingDeliveries, _gameInfo.activeDeliveries, 
		_gameInfo.completedDeliveries, _gameInfo.output);
}

/* Clear game information before updating a game state */
void GameEnv::clearGameInfo(void){
	_gameInfo.vans.clear();
	_gameInfo.edges.clear();
	_gameInfo.waitingDeliveries.clear();
	_gameInfo.activeDeliveries.clear();
	_gameInfo.completedDeliveries.clear();
}

/*-----------------DELIVERY MANAGEMENT---------------------------------------*/
/*
*	Assign new deliveries to the free vans.
*	If there's more than one delivery to assign, computation of paths occurs in
*	parallel.
*/
void GameEnv::assignDeliveriesParallel(void){
	size_t count = 0; // number of scheduled assignments
	vector<pair<int,DeliveryInfo>> assignments; 

	// retrieve van to delivery assignments
	if(!_gameInfo.waitingDeliveries.empty()){
		DeliveryInfo *deliveryInfo;
		VanInfo* vanInfo;

		while((deliveryInfo=getAvailableDelivery()) != NULL &&
				(vanInfo = getFreeNearestVan(deliveryInfo->pickUp)) != NULL){
			if(schedulePickUpTask(*deliveryInfo, vanInfo->Number)){
				assignments.push_back(make_pair(vanInfo->Number,*deliveryInfo));
				++count;
			}
		}
	}

	// compute path for each assignment and send instructions to van
	if(count > 1){
		parallel_for(size_t(0), count, [&](size_t i) {
			int vanNumber = assignments[i].first;
			sendVanTo(assignments[i].first, assignments[i].second.pickUp);
		});
	} else if(count == 0){
		sendVanTo(assignments[0].first, assignments[0].second.pickUp);
	}
}

/* Assign drop-off instruction for every van who has picked-up the delivery.
*  Handle accidental pick-ups.
*/
void GameEnv::manageDeliveries(void){
	Location vanLoc, pickupLoc, goal;
	DeliveryInfo* delivery;
	int schdDeliveryNum; // scheduled delivery number
	int vanNum, actualDeliveryNum;

	for(int i=0; i<N_VANS; i++){
		if(hasVanPickedUpDelivery(i)) {
			// gather van and delivery info
			vanNum = _gameInfo.vans[i].Number;
			vanLoc = _gameInfo.vans[i].location;
			schdDeliveryNum = _activeTasks.vans[vanNum];
			pickupLoc = _activeTasks.del2pickup[schdDeliveryNum];

			actualDeliveryNum = _gameInfo.vans[i].cargo;

			if(schdDeliveryNum == actualDeliveryNum){ // the delivery is picked up!
				removePickUpTask(schdDeliveryNum, vanNum);
				// send drop off instructions to the van
				if( (delivery=getDeliveryInfo(schdDeliveryNum)) != NULL ){ 
					sendVanTo(vanNum, delivery->dropOff);
				} else {  /* otherwise something went wrong */ }

			} else { // accidental pickup
				SolveConflict(schdDeliveryNum, vanNum, actualDeliveryNum);
				return;
			}
		}
	}
}

/* Manage deffered drop-off tasks */
void GameEnv::manageDefferedDeliveries(void){

	if(_activeTasks.deferredVans.empty()){
		return;
	}

	int vanNum, deliveryNum;
	uint8_t count = 0;
	vector<pair<int,int>> defferedDeliveries;

	for (auto it = _activeTasks.deferredVans.begin(); it != _activeTasks.deferredVans.end(); ++it) {
		if(_gameInfo.vans[it->first].instructions.empty()){ // van is stopped
			vanNum = it->first;
			deliveryNum = it->second;

			DeliveryInfo* delivery;
			// send drop-off instructions to the van
			if((delivery = getDeliveryInfo(deliveryNum)) != NULL ){
				sendVanTo(vanNum, delivery->dropOff);
				defferedDeliveries.push_back(make_pair(vanNum,delivery->Number));
				count++;
			}
		}
	}
	// remove deffered drop-off tasks
	for(uint8_t j=0; j<count; j++){
		removeDefferedDropOffTask(defferedDeliveries[j].second,defferedDeliveries[j].first);
	}
}

/* Solve conflict arises due to accidental pick-up */
void GameEnv::SolveConflict(int& schdDeliveryNum, int& vanNum, int& actualDeliveryNum){
	// send stop instructions to the current van
	sendInstructionsToVan(vanNum, vector<Location>());
	// remove no longer relevant delivery task for the current van
	removePickUpTask(schdDeliveryNum, vanNum);
	// add deffered pick-up task for the current van
	addDefferedDropOffTask(actualDeliveryNum, vanNum);

	// check for conflicts with another van				
	if(_activeTasks.deliveries.count(actualDeliveryNum)) {
		int conflictVan = _activeTasks.deliveries[actualDeliveryNum]; 
		// send stop instructions to the conflict van
		sendInstructionsToVan(conflictVan, vector<Location>());
		removePickUpTask(actualDeliveryNum, conflictVan);
	}
}
/*-----------------PATH FINDING----------------------------------------------*/
/* Optimized version of the searching A* algorithm */
Path GameEnv::findRoadOptimized(Node& start, Node& goal, h_func heuristic){
	// the tree of visited edges
	unordered_map<Location, Location, hash_pair> road;
	// g(n) - computed cost
	unordered_map<Node, int, hash_pair> g_func, h_func;
	// frontier
	priority_queue<NodeRecord, vector<NodeRecord>, greater_equal<NodeRecord>> open; 
	// store actual data
	NodeRecord currentNodeRecord;
	Location currentNode, nextNode;

	int currentCost, f_func;
	bool isVisited;

	road[Location(-1,-1)] = Location(-1,-1);
	g_func[start] = 0;
	open.push(NodeRecord(start,Location(-1,-1),g_func[start]));

	while(!open.empty()){
		// Pick the cheapest node in open set
		currentNodeRecord = open.top();
		open.pop(); 
		
		currentNode = currentNodeRecord.node;

		// Goal check
		if(currentNode == goal) { 
			break;
		}

		// Get possible edges
		vector<Edge> edges = getOutgoingEdges(currentNode);
		for(Edge::edge_itr e = edges.begin(); e != edges.end(); e++){
			nextNode = e->getNextNode(currentNode);
			currentCost = g_func[currentNode] + e->getCost();
			if(currentCost>185){maxCost =currentCost; continue;}
			// 1. cost for next node is not computed(not visited node) OR
			// 2. currentCost is better than already computed
			
			if((isVisited=!g_func.count(nextNode)) || g_func[nextNode] > currentCost){
				if(isVisited){ // we don't want to recalculate the cost
					f_func = currentCost + h_func[nextNode]; 
				} else {
					h_func[nextNode] = (this->*heuristic)(nextNode, goal); 
					f_func = currentCost + h_func[nextNode]; 
				}	
				g_func[nextNode] = currentCost;
				road[e->getLocation()] = currentNodeRecord.fromEdge;
				open.push(NodeRecord(nextNode,e->getLocation(),f_func));
			}
		}
	}

	Path pathToGoal;
	pathToGoal.reserve(G_MATRIX_LENGTH*3);
	Location currentEdge = currentNodeRecord.fromEdge; 
	// Make path
	while(currentEdge.first>=0){
		pathToGoal.push_back(currentEdge);
		currentEdge = road[currentEdge];
	}

	reverse(pathToGoal.begin(),pathToGoal.end());

	this->cost = g_func[currentNode];
	if(cost>maxCost){
		maxCost = cost;
	}
	return pathToGoal;
}

/* Get outgoing edges from the node */
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
/* Precompute gain coefficients for heuristic function */
void GameEnv::precomputeRoadTypes(void){
	_edgesTypes = _gameInfo.edges;
	/*for(uint8_t i=0; i<_edgesTypes.size(); i++ ){
		vector<int> row;
		for(uint8_t j=0; j<_edgesTypes[i].size(); j++ ){
			switch(_edgesTypes[i][j]){
				case 1: _edgesTypes[i][j]=3; break; //highway
				case 3: _edgesTypes[i][j]=2; break; //suburban
				case 4:	_edgesTypes[i][j]=1; break; //urban
			}
		}
	}*/
}
/*-----------------VAN CONTROL-----------------------------------------------*/
/* 
*	Send bulk of instructions to all vans. 
*	Used only by SpreadOut at the start of the game.
*/
void GameEnv::sendInstructions(InstructionsSet& sendInstructions){
	wstring output, success(L"200");
	_client->sendInstructions(sendInstructions, output);

	if(output.substr(0,3)!=success){
		throw std::runtime_error( "The instructions could not be sent!" );
	}
}

/* 
*	Send instructions to the van.
*/
void GameEnv::sendInstructionsToVan(int& van, Path& instructions){
	wstring output, success(L"200");
	InstructionsSet instructionsSet;
	instructionsSet[van].reserve(instructions.size());
	instructionsSet[van] = instructions;
	_client->sendInstructions(instructionsSet, output);

	if(output.substr(0,3)!=success){
		throw std::runtime_error( "The instructions could not be sent!" );
	}
}

/* Send a van to the specified location */
void GameEnv::sendVanTo(int& vanNumber, Location& location){
	Path road = findPath(_gameInfo.vans[vanNumber].location, location); 
	sendInstructionsToVan(vanNumber, road);
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
/*-----------------OPERATIONAL INFORMATION-----------------------------------*/
/* Retrieve the oldest delivery that is not a pick-up goal for any van */
DeliveryInfo* GameEnv::getAvailableDelivery(void){
	int deliveryNum, maxTime = G_END_TIME;
	DeliveryInfo* deliveryInfo = NULL;

	for(size_t i=0; i<_gameInfo.waitingDeliveries.size(); i++){
		deliveryNum = _gameInfo.waitingDeliveries[i].Number;

		if(!isDeliveryAssignedForPickUp(deliveryNum)){
			if(_gameInfo.waitingDeliveries[i].Time < maxTime){
				maxTime = _gameInfo.waitingDeliveries[i].Time;
				deliveryInfo = &_gameInfo.waitingDeliveries[i];
			}
		}
	}	

	return deliveryInfo;
}

/* Retrieve the nearest free van from the pickup node */
VanInfo* GameEnv::getFreeNearestVan(Node& pickup) {
	VanInfo* nearestVan = NULL;
	uint8_t dist, nearestDist = 100;

	for(uint8_t i=0; i<N_VANS; i++){
		if(isFreeVan(_gameInfo.vans[i].Number)){
			dist = manhattanDistance(_gameInfo.vans[i].location, pickup);
			if(dist < nearestDist){
				nearestDist = dist;
				nearestVan = &_gameInfo.vans[i];
			}
		}
	}

	return nearestVan;
}

/* Return a deliveryInfo by delivery number */
DeliveryInfo* GameEnv::getDeliveryInfo(int& deliveryNum){
	auto pred = [deliveryNum](const DeliveryInfo &delivery) {
		return delivery.Number == deliveryNum;
	};

	DeliveryList::iterator deliveryInfo = 
		find_if(_gameInfo.activeDeliveries.begin(), 
			_gameInfo.activeDeliveries.end(), pred);
	
	if(deliveryInfo == _gameInfo.activeDeliveries.end()){
		return NULL;
	} 
	return &deliveryInfo[0];
}
/* Check if the delivery has been already assigned to any van */
bool GameEnv::isDeliveryAssignedForPickUp(int& deliveryNum){
	return _activeTasks.deliveries.count(deliveryNum)
			|| _activeTasks.deferredDeliveries.count(deliveryNum);
}

/* Check if the van is free */
bool GameEnv::isFreeVan(int& vanNum){
	VanInfo van = _gameInfo.vans[vanNum];

	return !_activeTasks.vans.count(vanNum) // not in active pick-up tasks
			&& !_activeTasks.deferredVans.count(vanNum) //not in deffered tasks
			&& van.cargo == -1 // doesn't have cargo		
			&& van.instructions.empty(); // doesn't have instructions
}

/* Check if the van has picked-up a delivery */
bool GameEnv::hasVanPickedUpDelivery(int& vanNum){
	return _gameInfo.vans[vanNum].cargo != -1 // has cargo
			&& _activeTasks.vans.count(vanNum); // in active pick-up tasks
}
/*-----------------TASKS MANAGEMENT------------------------------------------*/
/* Place pickup information to the set of unodered_map for the fast access*/
bool GameEnv::schedulePickUpTask(DeliveryInfo& delivery, int& vanNum){
 	_activeTasks.deliveries[delivery.Number] = vanNum;
	_activeTasks.vans[vanNum] = delivery.Number;
	_activeTasks.pickup2del[delivery.pickUp] = delivery.Number;
	_activeTasks.del2pickup[delivery.Number] = delivery.pickUp;

	return true;
}
/* Remove scheduled pick-up task (the van has picked-up the delivery) */
void GameEnv::removePickUpTask(int& deliveryNum, int& vanNum){
	_activeTasks.vans.erase(vanNum);
	_activeTasks.deliveries.erase(deliveryNum);

	Location location = _activeTasks.del2pickup[deliveryNum];

	_activeTasks.pickup2del.erase(location);
	_activeTasks.del2pickup.erase(deliveryNum);
}

/* Add deffered drop-off task for the van */
void GameEnv::addDefferedDropOffTask(int& deliveryNum, int& vanNum){
	_activeTasks.deferredDeliveries[deliveryNum] = vanNum;
	_activeTasks.deferredVans[vanNum] = deliveryNum;
}

/* Add deffered drop-off task for the van */
void GameEnv::removeDefferedDropOffTask(int& deliveryNum, int& vanNum){
	_activeTasks.deferredDeliveries.erase(deliveryNum);
	_activeTasks.deferredVans.erase(vanNum);
}
/*---------------------------------------------------------------------------*/


/*-----------------DEBUG FUNCTIONALITY---------------------------------------*/
Path GameEnv::findPathDebug(Node start, Node end, int h_func){
	switch(h_func){
		case ALGO::DIJKSTRA: 
			return findRoadOptimized(start, end, &GameEnv::dijkstra);
		case ALGO::EUCLIDEAN_DISTANCE: 
			return findRoadOptimized(start, end, &GameEnv::euclideanDistance);
		case ALGO::MANHATTAN_DISTANCE: 
			return findRoadOptimized(start, end, &GameEnv::manhattanDistance);
		case ALGO::REPULSIVE_CENTER: 
			return findRoadOptimized(start, end, &GameEnv::repulsiveCenter);
		default: 
			return findRoadOptimized(start, end, &GameEnv::dijkstra);
	}
}
bool GameEnv::checkPath(Node start, Node end, h_func heuristic) {
		bool isCorrect = true;
		// Get the path.
		Path path = findRoadOptimized(start, end, heuristic);

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
	if(h_func == ALGO::DIJKSTRA){
		sumOfOptimalCosts = sumOfCosts;
	}
	info.avg_game_units=double (_gameInfo.time - elapsed_game_units)/(size-1);
	info.elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	info.avg = info.elapsed_secs/(size-1);
	info.quality = 2.0-(double)sumOfCosts/sumOfOptimalCosts;
	info.avg_time = (double)sumOfCosts/((size-1)/2);
	return info;
}

/*-----------------TEMP------------------------------------------------------*/

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

/*----------------------------Clean-up---------------------------------------*/

GameEnv::~GameEnv(){
	delete _client;
}
