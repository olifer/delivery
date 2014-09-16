#include "edge.h"

class GameGraph {
	GameEdgesCosts *_gameEdgesCosts;
	GameNodes *_gameNodes;
public:
	void setEdges(GameEdgesCosts *gameEdgesCosts);
	std::vector<Edge> getEdges(Node fromNode);
	GameGraph(GameNodes *nodes);
	GameGraph(GameNodes *gameNodes, GameEdgesCosts *gameEdgesCosts);
	~GameGraph(void);
private:
	void setNodes(GameNodes *gameNodes);
}; 


void GameGraph::setEdges(GameEdgesCosts *gameEdgesCosts){
	_gameEdgesCosts = gameEdgesCosts;
}

void GameGraph::setNodes(GameNodes *gameNodes){
	_gameNodes = gameNodes;
}

std::vector<Edge> GameGraph::getEdges(Node fromNode){
	std::vector<Edge> Edges;
	Node nextNode;
	Location loc;
	int cost;

	if(fromNode.second > 0){ // check node above fromNode
		nextNode = std::make_pair(fromNode.first+1,fromNode.second);
		loc = std::make_pair(2*fromNode.first-1, fromNode.second);
		cost = (*_gameEdgesCosts)[loc.second][loc.first];
		Edges.push_back(Edge (fromNode, nextNode, cost, loc));
	}
	// TODO below node, right node, and left node.
	//Edges.push_back();
}

GameGraph::GameGraph(GameNodes *gameNodes){
	GameGraph::setNodes(gameNodes);
}

GameGraph::GameGraph(GameNodes *gameNodes, GameEdgesCosts *GameEdgesCosts){
	GameGraph::setNodes(gameNodes);
	GameGraph::setEdges(GameEdgesCosts);
}

GameGraph::~GameGraph(){
}
