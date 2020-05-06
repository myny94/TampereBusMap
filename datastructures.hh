// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>
#include <unordered_map>
#include <memory>
#include <unordered_set>

// Types for IDs
using StopID = long int;
using RegionID = std::string;
using RouteID = std::string;
using Name = std::string;

// Return values for cases where required thing was not found
RouteID const NO_ROUTE = "!!NO_ROUTE!!";
StopID const NO_STOP = -1;
RegionID const NO_REGION = "!!NO_REGION!!";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

struct Node {
    RouteID route_id;
    StopID stop_id;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary
inline bool operator==(Node n1, Node n2) { return n1.stop_id == n2.stop_id && n1.route_id == n2.route_id; }
inline bool operator!=(Node n1, Node n2) { return !(n1==n2); } // Not strictly necessary

struct NodeHash
{
    std::size_t operator()(Node node) const
    {
        auto str_hasher = std::hash<std::string>();
        auto int_hasher = std::hash<long int>();
        auto xhash = int_hasher(node.stop_id);
        auto yhash = str_hasher(node.route_id);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for time of day in minutes from midnight (i.e., 60*hours + minutes)
using Time = int;

// Return value for cases where color was not found
Time const NO_TIME = std::numeric_limits<Time>::min();

// Type for a duration of time (in minutes)
using Duration = int;

// Return value for cases where Duration is unknown
Duration const NO_DURATION = NO_VALUE;

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Duration is unknown
Distance const NO_DISTANCE = NO_VALUE;

struct distance_comparison { bool operator()(std::pair<Node, Distance> const& d1, std::pair<Node, Distance> const& d2){return d1.second > d2.second; } };
struct time_comparison { bool operator()(std::pair<Node, Time> const& t1, std::pair<Node, Time> const& t2){return t1.second > t2.second; } };

// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: O(1)
    // Short rationale for estimate: for unordered_map, the function .size() takes constant time.
    int stop_count();

    // Estimate of performance: O(1)
    // Short rationale for estimate: Clear operation used is linear on size (destructors) for
    // vectors and unorederd map according to cpp references.
    void clear_all();

    // Estimate of performance: Theta(n), O(n^2)
    // Short rationale for estimate: Accessing keys from unordered_map takes constant time, so obtaining elements
    // from the unordered_map takes linear time. However, worst case rarely happens if the hash function is good.
    std::vector<StopID> all_stops();

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: In average, adding elements in unordered_map takes constant time,
    // but when hash function produces collisions for every insertion in the map(if the hash function is not good) it might cause
    // linear time in the worst case.
    bool add_stop(StopID id, Name const& name, Coord xy);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: In average, finding elements in unordered_map takes constant time.
    // But rarely, in worst case, time complexity might be linear.
    Name get_stop_name(StopID id);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: In average, finding elements in unordered_map takes constant time.
    // But rarely, in worst case, time complexity might be linear.
    Coord get_stop_coord(StopID id);

    // Estimate of performance: O(nlogn)
    // Short rationale for estimate: std::sort takes O(nlogn).
    std::vector<StopID> stops_alphabetically();

    // Estimate of performance: O(nlogn)
    // Short rationale for estimate: std::sort takes O(nlogn)
    std::vector<StopID> stops_coord_order();

    // Estimate of performance: O(n)
    // Short rationale for estimate: In my program, I am using unordered_map as a main data structure and when sorting, keys are copied
    // and sorted, so there is no way to keep track if the data structure is sorted or not. So for getting minimum and maximum, all elements
    // should be touched once (linear).
    StopID min_coord();

    // Estimate of performance: O(n)
    // Short rationale for estimate: To get a maximum value, all elements should be checked once (linear).
    StopID max_coord();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Searching elements in unordered_map by key has constant time in average, and has linear time in
    // container size in worst case. However, worst case rarely happens if the hash function is good.
    std::vector<StopID> find_stops(Name const& name);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Searching elements in unordered_map by key has constant time in average, and has linear time in
    // container size in worst case. However, worst case rarely happens if the hash function is good.
    bool change_stop_name(StopID id, Name const& newname);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Searching elements in unordered_map by key has constant time in average, and has linear in
    // container size in worst case.
    bool change_stop_coord(StopID id, Coord newcoord);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Searching elements in unordered_map by key has constant time in average, and has linear in
    // container size in worst case.
    bool add_region(RegionID id, Name const& name);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Searching elements in unordered_map by key has constant time in average, and has linear in
    // container size in worst case.
    Name get_region_name(RegionID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Going through all elements and pushing back to the new vector takes linear time.
    std::vector<RegionID> all_regions();

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Finding element in unordered_map by key takes contant time in avearge, and has linear in
    // container size in worst case. So for both stops_map and regions_map, it takes constant time
    bool add_stop_to_region(StopID id, RegionID parentid);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Finding element in unordered_map by key takes contant time in avearge, and has linear in
    // container size in worst case. So for both stops_map and regions_map, it takes constant time
    bool add_subregion_to_region(RegionID id, RegionID parentid);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Finding element in unordered_map by key takes contant time in avearge, and has linear in
    // container size in worst case. But the worst case rarely happens if the hash function is good.
    std::vector<RegionID> stop_regions(StopID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Both for loop through the unordered_map and std::min_element and std::max_element takes linear time.
    std::pair<Coord, Coord> region_bounding_box(RegionID id);

    // Estimate of performance: O(nlogm) where m = (middle - first of the array length),
    // Short rationale for estimate: std::partial_sort is gives slightly better performance compared to the entire sort. In our case, since
    // only need to return the closest 5 stops, partial sort was used.
    std::vector<StopID> stops_closest_to(StopID id);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Finding an element by key in unordered_map takes constant time in average, and has linear in
    // container size in worst case. However, worst case rarely happens if the hash function is good.
    bool remove_stop(StopID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: searching element in unordered_set takes constant time in average, and has linear in container size
    // in worst case. Also, looping through the keys in unordered_map takes linear time. I copied vector into unordered_set in order to
    // find the common elements more quickly.
    RegionID stops_common_region(StopID id1, StopID id2);

    // Phase 2 operations

    // Estimate of performance: O(n)
    // Short rationale for estimate: Every key in unordered_map should be gone through to return the keys of the unordered_map.
    std::vector<RouteID> all_routes();

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Finding element in unordered_map by key takes contant time in avearge, and has linear in
    // container size in worst case. But the worst case rarely happens if the hash function is good.
    bool add_route(RouteID id, std::vector<StopID>& stops);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: Finding elements in unordered_map takes constant time. The worst case is when the stop belongs to
    // all the possible routes, and the loop needs to go through all the existing routes.
    std::vector<std::pair<RouteID, StopID>> routes_from(StopID stopid);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: In average, finding elements in unordered_map takes constant time.
    // But rarely, in worst case, time complexity might be linear.
    std::vector<StopID> route_stops(RouteID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Clear operation used is linear on size (destructors) for
    // vectors and unorederd map according to cpp references.
    void clear_routes();

    // Estimate of performance: O(V+E) where V is the number of vertices and E is the number of edges.
    // The Djikstra algorithm was used with adjacency List and Priority queue where v is the total number of vertices and E is total number of edges
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_any(StopID fromstop, StopID tostop);

    // Estimate of performance: O(V+E) where V is the number of vertices and E is the number of edges.
    // Short rationale for estimate: The Breadth-first search algorithm was implemented using
    // a queue where v is the total number of vertices and E is total number of edges
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_least_stops(StopID fromstop, StopID tostop);

    // Estimate of performance: O(V+E) where V is the number of vertices and E is the number of edges.
    // Short rationale for estimate: Journey_with_cycle uses "journey_any" method, so the performace is same.
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_with_cycle(StopID fromstop);

    // Estimate of performance: O((v+e) log v)
    // Short rationale for estimate: The Djikstra algorithm was used with adjacency List and Priority queue where v is the total number of vertices
    // and E is total number of edges
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_shortest_distance(StopID fromstop, StopID tostop);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: In average, finding elements in unordered_map takes constant time.
    // But rarely, in worst case, time complexity might be linear. And push_back to std::vector takes constant time as well.
    bool add_trip(RouteID routeid, const std::vector<Time> &stop_times);

    // Estimate of performance: Theta(1), O(n)
    // Short rationale for estimate: In average, finding elements in unordered_map takes constant time.
    // But rarely, in worst case, time complexity might be linear. And the loop in this function takes O(M) times where M is the number
    // of timetables with a given route.
    std::vector<std::pair<Time, Duration> > route_times_from(RouteID routeid, StopID stopid);

    // Estimate of performance: O((v+e) log v)
    // Short rationale for estimate: The Djikstra algorithm was used with adjacency List and Priority queue where v is the total number of vertices
    // and E is total number of edges
    std::vector<std::tuple<StopID, RouteID, Time>> journey_earliest_arrival(StopID fromstop, StopID tostop, Time starttime);

private:
    // Add stuff needed for your class implementation here
    struct Stop {
           StopID id;
           Name name;
           Coord coord;
           RegionID regionID;
       };

       struct Region {
           RegionID id;
           Name name;
           std::weak_ptr<Region> parent;
           std::vector<std::weak_ptr<Region>> children;
       };

       struct Route {
           RouteID id;
           std::vector<StopID> stops;
           std::unordered_map<StopID, int> stops_index;
           std::vector<std::vector<Time>> timetables;
       };

       std::unordered_map<StopID, std::shared_ptr<Stop>> stops_map;
       std::unordered_map<RegionID, std::shared_ptr<Region>> regions_map;
       std::unordered_map<RouteID, std::shared_ptr<Route>> routes_map;
       std::unordered_map<StopID, std::vector<RouteID>> routes_with_stop;

       std::unordered_map<StopID, std::unordered_map<StopID, int>> stop_distance;

       //functions
       std::vector<std::shared_ptr<Region>> descendants(std::shared_ptr<Region> region);
       // Breadth first search algorithm
       std::vector<std::pair<Node,int>> BFS(StopID fromstop, StopID tostop);
       // find neighbors of a given node
       std::vector<Node> find_neighbors(Node node);
       std::vector<std::pair<Node,int>> path(const std::unordered_map<Node, std::pair<Node, int>, NodeHash> & parentMap, StopID fromstop, std::pair<Node,int> endNode);

       Distance calculate_distance(StopID from, StopID to);
       Time calculate_time(Node from, Node to, Time time);
       std::pair<std::vector<Time>, int> get_earliest_departure(RouteID routeid, StopID stopid, Time time);
};

#endif // DATASTRUCTURES_HH
