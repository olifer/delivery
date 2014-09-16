Delivery
========
C++ implementation of the A* algorithm for the AI course (Autumn 2014) at Uppsala University.

Setup
-----
1. Clone repository.
2. Follow instructions given in "Step by Step Instructions for DM.pdf" in the repo directory. 
And then it *should* be okay.

Structure
---------
* *Node*: Represents a node of the graph. 
* *Edge*: Represents an edge of the graph. 
* *GameNodes*: List of nodes.
* *GameEdgesCosts*: Costs of all edges. (integers)
* *VanList*: List of vans.
* *DeliveryList*: List of deliveries (and their info).
* *GameGraph*: List of nodes and costs. 

Tips
----
Use `#include <algorithm>` . 
This library contains several useful functions such as `find`, `foreach`,`mix/max_element` and more.
See [here](http://www.cplusplus.com/reference/algorithm/).

Side note: It might be possible to use `functional` (from stdlib) to pass the heuristic
function as a parameter of the algorithm. 
See [here](http://www.cplusplus.com/reference/functional/).
