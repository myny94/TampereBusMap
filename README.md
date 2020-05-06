This program is made as a part of the course "Data Structures and Algorithms" at Tampere University.
The part "mainprogram.cc, mainprogram.hh, mainwindow.cc, mainwindow.hh, mainwindow.ui" is made by the course personnel, and the part "datastructures.hh, datastructures.cc" is fully implemented by me.


# Datastructures and Algorithms
## Datastructures
### Route 
```
struct Route {
           RouteID id;
           std::vector<StopID> stops;
           std::unordered_map<StopID, int> stops_index;
           std::vector<std::vector<Time>> timetables;
       };

std::unordered_map<RouteID, std::shared_ptr<Route>> routes_map;
```
Store information of a route id and find the Route struct fast by route id using unordered_map. Since the order of stops and the trip times are same, many times there is a need to get the time corresponding a given stop, so for this, stops_index unordered_map was added as extra datastructure to keep track of index fast. 


### Routes with a given stop
 ```
std::unordered_map<StopID, std::vector<RouteID>> routes_with_stop;
 ```
 In several methods, routes that contains a given stop should be returned, and for this, I added unordered_map that saves the routes that contains a given stop. This datastructure is updated in add_route function. Without this datastructure, all routes should be looked up to return the routes that contain the certain stop and this is inefficient.

### Distance between stops
```
std::unordered_map<StopID, std::unordered_map<StopID, int>> stop_distance;
 ```
 Sqrt is an expensive function so it is if the euclidean distance should be calculated every time we need to. So instead, I saved the starting point as the key of the unordered_map, and arriving point as the key of the nested unordered_map. This way, same distance does not need to be calculated multiple times, but rather can be looked up with constant time.

### Priority queue
Priority queue was used to choose the best nodes to visit next among all the neighbors. Empty priority queue to which n items to be added takes time O(n*log(n)) whereas putting the all the elements at once takes O(n) time. And removing n items from the priority queue takes O(n*log(n)) time. In this project, neighbor nodes are added one by one to priority queue so it is slower than adding them at once, but since the time /distance should be calculated for each of the neighbor node, this method was used. And priority queue enables us to apply Dijkstra's algorithm and find the best journey with a given criteria.


## Algorithms

### Dijkstra's algorithm
Dijkstra's algorithm finds a shortest path tree from a single source node by building a set of nodes that have minimum criteria set from the source. In my program, I used this algorithm for finding the shortest route and the fastest journey. Djkstra is slower than A start search algorithm which is an informed variastion of Dijkstra. However, A start does not always return the best joureny, but "good enough" journey. In my code, I tested with both algorithms and find out there is not so much difference in performance, so I chose the exact algorithm for accuracy.


### Breadth First Search
For both Depth first search and Breadth first search, the worst case is same (it needs to go through all stops). However, if the search can be aborted when a matching element is found, BFS is slightly faster if the searched element is higher up in the search tree since BFS goes level by level. That is how BFS can be used to find the route that has the least nodes. In this program, BFS was used for finding the journey with the least nodes and returning any journey.

