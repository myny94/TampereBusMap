// Datastructures.cc

#include "datastructures.hh"

#include <random>
#include <cmath>
#include <stdexcept>

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <map>
#include <set>
#include <math.h>
#include <iostream>
#include <cmath>
#include <queue>
#include <ostream>
#include <bits/stdc++.h>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

double distance(Coord c1, Coord c2)
{
   return sqrt(pow(c2.x-c1.x, 2) + pow(c2.y-c1.y, 2));
}

Datastructures::Datastructures()
{

}

Datastructures::~Datastructures()
{
    clear_all();
}

int Datastructures::stop_count()
{
    std::size_t size = stops_map.size();
    return size;
}

void Datastructures::clear_all()
{
    stops_map.clear();
}

std::vector<StopID> Datastructures::all_stops()
{
    if (stops_map.size() == 0) {
        return {NO_STOP};
    }
    std::vector<StopID> stops_vector;
    for (const auto &stopPair : stops_map) {
        stops_vector.push_back(stopPair.first);
    }
    return stops_vector;
}


bool Datastructures::add_stop(StopID id, const Name& name, Coord xy)
{
    // if the stop is not found in data structure
    if (stops_map.find(id) == stops_map.end()) {
        auto new_stop = std::make_shared<Stop>(Stop{id, name, xy,""});
        stops_map.insert({id, new_stop});
        return true;
    }
    else {
        return false;
    }
}

Name Datastructures::get_stop_name(StopID id)
{
    // if the stop is not found in data structure
    auto stop = stops_map.find(id);
    if (stop == stops_map.end()) {
        return NO_NAME;
    } else {
        return stop->second->name;
    }
}

Coord Datastructures::get_stop_coord(StopID id)
{
    auto stop = stops_map.find(id);
    if (stop == stops_map.end()) {
        return NO_COORD;
    } else {
        return stop->second->coord;
    }
}

std::vector<StopID> Datastructures::stops_alphabetically()
{
    std::vector<StopID> stops = all_stops();
    std::sort(stops.begin(), stops.end(), [=](const StopID & stop1, const StopID & stop2) {
        return stops_map.at(stop1)->name < stops_map.at(stop2)->name;
    });
    return stops;
}

std::vector<StopID> Datastructures::stops_coord_order()
{
    std::vector<StopID> stops = all_stops();
    std::sort(stops.begin(), stops.end(), [=](const StopID & stop1, const StopID & stop2) {
        return distance(stops_map.at(stop1)->coord, Coord{0,0}) < distance(stops_map.at(stop2)->coord, Coord{0,0});
    });
    return stops;
}

StopID Datastructures::min_coord()
{
    if (stops_map.size() == 0) {
        return NO_STOP;
    }
    else {
        std::shared_ptr<Stop> stop = nullptr;
        for (const auto &stopPair : stops_map) {
            if (stop == nullptr) {
                stop = stopPair.second;
                continue;
            }
            double distance1 = distance(stop->coord, Coord{0, 0});
            double distance2 = distance(stopPair.second->coord, Coord{0, 0});
            if (distance2 < distance1) {
                stop = stopPair.second;
            }
        }
        return stop->id;
    }
}

StopID Datastructures::max_coord()
{
    if (stops_map.size() == 0) {
        return NO_STOP;
    }
    else {
        std::shared_ptr<Stop> stop = nullptr;
        for (const auto &stopPair : stops_map) {
            if (stop == nullptr) {
                stop = stopPair.second;
                continue;
            }
            double distance1 = distance(stop->coord, Coord{0, 0});
            double distance2 = distance(stopPair.second->coord, Coord{0, 0});
            if (distance1 < distance2) {
                stop = stopPair.second;
            }
        }
        return stop->id;
    }
}

std::vector<StopID> Datastructures::find_stops(Name const& name)
{
    std::vector<StopID> stops;
    for (const auto &stopPair : stops_map) {
        if (stopPair.second->name == name) {
            stops.push_back(stopPair.first);
        }
    }
    return stops;
}

bool Datastructures::change_stop_name(StopID id, const Name& newname)
{
    auto stop = stops_map.find(id);
    // if stop with id is not found
    if (stop == stops_map.end()) {
        return false;
    } else {
        stops_map.at(id)->name = newname;
        return true;
    }
}

bool Datastructures::change_stop_coord(StopID id, Coord newcoord)
{
    auto stop = stops_map.find(id);
    // if stop with id is not found
    if (stop == stops_map.end()) {
        return false;
    } else {
        stops_map.at(id)->coord = newcoord;
        return true;
    }
}

bool Datastructures::add_region(RegionID id, const Name &name)
{
    auto region = regions_map.find(id);
    // if region with id is not found
    if (region != regions_map.end()) {
        return false;
    }
    else {
        auto new_region = std::make_shared<Region>(Region{id, name, std::weak_ptr<Region>(), {}});
        regions_map.insert({id, new_region});
        return true;
    }
}

Name Datastructures::get_region_name(RegionID id)
{
    auto region = regions_map.find(id);
    if (region == regions_map.end()) {
        return NO_NAME;
    }
    else {
        return regions_map.at(id)->name;
    }

}

std::vector<RegionID> Datastructures::all_regions()
{
    if (regions_map.size() == 0) {
        return {NO_REGION};
    }
    else {
        std::vector<RegionID> regions_vector;
        for (const auto &regionPair : regions_map) {
            regions_vector.push_back(regionPair.first);
        }
        return regions_vector;
    }
}

bool Datastructures::add_stop_to_region(StopID id, RegionID parentid)
{
    auto stop = stops_map.find(id);
    auto region = regions_map.find(parentid);
    // If no stop or region exists with given id or if the stop already belongs to the region
    if (stop == stops_map.end() || region == regions_map.end() || stops_map.at(id)->regionID == parentid) {
        return false;
    }
    else {
        stops_map.at(id)->regionID = parentid;
        return true;
    }
}

bool Datastructures::add_subregion_to_region(RegionID id, RegionID parentid)
{
    auto region1 = regions_map.find(id);
    auto region2 = regions_map.find(parentid);
    if (region1 == regions_map.end() || region2 == regions_map.end() || region1->second->parent.expired()) {
        return false;
    } else {
        region2->second->children.push_back(region1->second);
        region1->second->parent = region2->second;
        return true;
    }
}

std::vector<RegionID> Datastructures::stop_regions(StopID id)
{
    std::vector<RegionID> regions;
    auto stop = stops_map.find(id);
    if (stop == stops_map.end() || stop->second->regionID == "") {
        return {NO_REGION};
    }
    else {
        auto region = regions_map.at(stop->second->regionID);
        while (region != nullptr) {
            regions.push_back(region->id);
            region = region->parent.lock();
        }
        return regions;
    }
}

void Datastructures::creation_finished()
{
    // Replace this comment with your implementation
    // You don't have to use this method for anything, if you don't need it
}

std::pair<Coord,Coord> Datastructures::region_bounding_box(RegionID id)
{
    if (regions_map.find(id)== regions_map.end()) {
        return {NO_COORD, NO_COORD};
    }
    const auto descendentsOfRegion = descendants(regions_map.at(id));

    std::unordered_set<std::shared_ptr<Datastructures::Region>> regionIdSet(descendentsOfRegion.begin(), descendentsOfRegion.end());
    std::vector<int> x_coords;
    std::vector<int> y_coords;
    for (auto stop : stops_map) {
        // when the stop belongs to one of the descendent regions of the given region
        if (stop.second->regionID !="" &&
            regionIdSet.find(regions_map.at(stop.second->regionID)) != regionIdSet.end()) {
            x_coords.push_back(stop.second->coord.x);
            y_coords.push_back(stop.second->coord.y);
        }
    }
    if (x_coords.empty()) {
        return {NO_COORD, NO_COORD};
    }
    int min_x = *std::min_element(std::begin(x_coords), std::end(x_coords));
    int max_x = *std::max_element(std::begin(x_coords), std::end(x_coords));
    int min_y = *std::min_element(std::begin(y_coords), std::end(y_coords));
    int max_y = *std::max_element(std::begin(y_coords), std::end(y_coords));
    return std::pair(Coord{min_x, min_y}, Coord{max_x, max_y});
}

std::vector<StopID> Datastructures::stops_closest_to(StopID id)
{
    auto stop = stops_map.find(id);
    if (stop == stops_map.end()) {
         return {NO_STOP};
    }
    else {
        int stop_x = stop->second->coord.x;
        int stop_y = stop->second->coord.y;
        std::vector<std::pair<StopID, float>> distances;
        for (auto stopPair : stops_map) {
            if (stopPair.first == id)  {
                continue;
            }
            double distance = sqrt(pow((stop_x - stopPair.second->coord.x),2) + pow((stop_y - stopPair.second->coord.y),2));
            distances.push_back(std::pair(stopPair.first, distance));
        }
        std::partial_sort(distances.begin(), distances.begin()+5, distances.end(), [](const auto & distancePair1, const auto & distancePair2) {
            return distancePair1.second < distancePair2.second;});
        std::vector<std::pair<StopID, double>> closest_pairs(distances.begin(), distances.begin()+5);
        std::vector<StopID> closest_stops;
        for (auto distancePair : closest_pairs) {
            closest_stops.push_back(distancePair.first);
        }
        return closest_stops;
    }
}

bool Datastructures::remove_stop(StopID id)
{
    auto stop = stops_map.find(id);
    // If the stop with id is not found
    if (stop == stops_map.end()) {
        return false;
    }
    else {
        stops_map.erase(stop);
        return true;
    }
}

RegionID Datastructures::stops_common_region(StopID id1, StopID id2)
{
    auto stop1 = stops_map.find(id1);
    auto stop2 = stops_map.find(id2);
    if (stop1 == stops_map.end() || stop2 == stops_map.end()) {
        return NO_REGION;
    }
    else {
        RegionID common_region;
        std::vector<RegionID> stop1_regions = stop_regions(id1);
        std::vector<RegionID> stop2_regions = stop_regions(id2);
        // copy vector to set to find the element more efficiently
        std::unordered_set<RegionID> stop2Regions_set(stop2_regions.begin(), stop2_regions.end());
        // find the first common element of two region vectors
        for (RegionID region : stop1_regions){
            if (stop2Regions_set.find(region) != stop2Regions_set.end()) {
                return region;
            }
        }
        return NO_REGION;
    }

}

std::vector<RouteID> Datastructures::all_routes()
{
    if (routes_map.size() == 0) {
        return {NO_ROUTE};
    }
    std::vector<RouteID> routes_vector;
    routes_vector.reserve(routes_map.size());
    for (const auto &routePair : routes_map) {
        routes_vector.push_back(routePair.first);
    }
    return routes_vector;
}

bool Datastructures::add_route(RouteID id, std::vector<StopID>& stops)
{   
    auto route_iter = routes_map.find(id);
    if (route_iter != routes_map.end() || stops.size() < 2) {   // if the route already exists or one stop is given
        return false;
    }

    std::unordered_map<StopID, int> stop_index;
    int index = 0;
    for (auto const & stop: stops) {  // check if all stops exist. Terminate if the stops does not exist
        if (stops_map.find(stop) == stops_map.end()) {
            return false;
        }
        auto stop_iter = routes_with_stop.find(stop);
        if (stop_iter == routes_with_stop.end()) {
            std::vector<RouteID> routes = {id};
            routes_with_stop.insert({stop,routes});
        }
        else {
            stop_iter->second.push_back(id);
        }
        stop_index.insert({stop, index});
        ++index;
    }
    std::shared_ptr<Route> new_route = std::make_shared<Route>(Route{id, stops, stop_index, {}});
    routes_map.insert({id, new_route});
    return true;
}

std::vector<std::pair<RouteID, StopID>> Datastructures::routes_from(StopID stopid)
{
    auto stop_iter = stops_map.find(stopid);
    std::vector<std::pair<RouteID, StopID>> result = {};
    if (stop_iter != stops_map.end() && routes_with_stop.find(stopid) != routes_with_stop.end()) {
        auto routes = routes_with_stop.find(stopid)->second;
        for (auto route: routes) {
            auto index = routes_map.at(route)->stops_index.find(stopid)->second;
            if (index < routes_map.at(route)->stops.size() -1) {
                result.push_back(std::make_pair(route, routes_map.at(route)->stops.at(index+1)));
            }
        }
        return result;
    }
    return {{NO_ROUTE, NO_STOP}};
}

std::vector<StopID> Datastructures::route_stops(RouteID id)
{
    auto route = routes_map.find(id);
    if ( route == routes_map.end() ) {
        return {NO_STOP};
    }
    else {
        return routes_map.at(id)->stops;
    }
}

void Datastructures::clear_routes()
{
    routes_map.clear();
    routes_with_stop.clear();
}


std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_any(StopID fromstop, StopID tostop)
{
    return journey_least_stops(fromstop, tostop);
}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_least_stops(StopID fromstop, StopID tostop)
{
    auto fromstop_iter = stops_map.find(fromstop);
    auto tostop_iter = stops_map.find(tostop);
    if ( fromstop_iter == stops_map.end() || tostop_iter == stops_map.end()|| routes_with_stop.find(fromstop) == routes_with_stop.end() || routes_with_stop.find(tostop) == routes_with_stop.end()) {
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    }

    std::vector<std::tuple<StopID, RouteID, Distance>> result = {};
    std::vector<std::pair<Node,int>> route_nodes = BFS(fromstop, tostop);
    for (auto iter = route_nodes.rbegin(); iter != route_nodes.rend(); iter ++) {
        result.push_back(std::make_tuple(iter->first.stop_id,iter->first.route_id, iter->second));
    }
    return result;
}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_with_cycle(StopID fromstop)
{
    if ( stops_map.find(fromstop) == stops_map.end() || routes_with_stop.find(fromstop) == routes_with_stop.end() ) {
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    }
    std::vector<RouteID> routes = routes_with_stop.at(fromstop);
    for (RouteID route : routes) {;
        const auto index_map = routes_map.at(route)->stops_index;
        const auto stopindex = index_map.find(fromstop)->second;
        if (stopindex > 0) {                                                     // if the route does not stop from the fromstop
            auto previous_stop = routes_map.at(route)->stops.at(stopindex - 1);  // find the previous stop that goes to fromstop
            std::vector<std::tuple<StopID, RouteID, Distance>> journey = journey_any(fromstop, previous_stop);
            if (journey.size() != 0) {
                std::get<1>(journey.back()) = route;
                Distance dist = std::get<2>(journey.back()) + calculate_distance(std::get<0>(journey.back()),fromstop);
                journey.push_back({fromstop, NO_ROUTE, dist});
                return journey;
            }
        }
    }
    return {};
}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_shortest_distance(StopID fromstop, StopID tostop)
{

    if ( stops_map.find(fromstop) == stops_map.end() || stops_map.find(tostop) == stops_map.end() || routes_with_stop.find(fromstop) == routes_with_stop.end()|| routes_with_stop.find(tostop) == routes_with_stop.end()) {
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    }

    std::unordered_set<StopID> visited;
    std::priority_queue<std::pair<Node, Distance>, std::vector<std::pair<Node, Distance>>, distance_comparison> pri_queue;
    const auto routes = routes_with_stop.find(fromstop)->second;
    for (const auto & route : routes) {
        Node n{route, fromstop};
        pri_queue.push(std::make_pair(n,0));
        break;
    }
    std::unordered_map<Node, std::pair<Node,int>, NodeHash> parentMap;
    std::pair<Node,int> found = {{NO_ROUTE, NO_STOP}, NO_DISTANCE};

    while (!pri_queue.empty()) {          // repeat while there are vertices
        auto next = pri_queue.top();
        pri_queue.pop();                  // take the next vertex from queue
        if (visited.find(next.first.stop_id) != visited.end()) {
            continue;
        }

        if (next.first.stop_id == tostop) { // if vertex found, stop
            found = next;
            break;
        }

        const std::vector<Node> neighbors = find_neighbors(next.first);
        visited.insert(next.first.stop_id);
        for (const auto &neighbor : neighbors) {
            double nextToNeighborDistance = calculate_distance(next.first.stop_id, neighbor.stop_id);
            pri_queue.push(std::make_pair(neighbor, next.second + nextToNeighborDistance));
            parentMap[neighbor] = next;
        }
    }

    if (found.first.stop_id != tostop) {
        return {};
    }

    std::vector<std::pair<Node,int>> route_nodes = path(parentMap, fromstop, found);
    std::vector<std::tuple<StopID, RouteID, Distance>> result = {};
    result.reserve(route_nodes.size());

    for (auto iter = route_nodes.rbegin(); iter != route_nodes.rend(); iter ++) {
        result.push_back(std::make_tuple(iter->first.stop_id,iter->first.route_id, iter->second));
    }

    return result;
}

bool Datastructures::add_trip(RouteID routeid, std::vector<Time> const& stop_times)
{
    auto route_iter = routes_map.find(routeid);
    if (route_iter == routes_map.end()) {   // if the route already exists or one stop is given
        return false;
    }
    std::shared_ptr<Route> route = route_iter->second;
    route->timetables.push_back(stop_times);
    return true;
}

std::vector<std::pair<Time, Duration>> Datastructures::route_times_from(RouteID routeid, StopID stopid)
{
    auto route_iter = routes_map.find(routeid);
    auto stop_iter = stops_map.find(stopid);
    auto route_stops = routes_map.at(routeid)->stops;
    unsigned int stop_position = routes_map.at(routeid)->stops_index.at(stopid);

    if (route_iter == routes_map.end() || stop_iter == stops_map.end() || stop_position == route_stops.size()-1) {
        return {{NO_TIME, NO_DURATION}};
    }
    std::vector<std::pair<Time, Duration>> times = {};
    if ( stop_position > route_stops.size()-2 ) {   // if the route ends with the given stop
        return times;
    }
    std::vector<std::vector<Time>> timetables = route_iter->second->timetables;
    for (const auto & timetable : timetables) {
        Time time = timetable.at(stop_position);
        Time nextstop_time = timetable.at(stop_position + 1);
        Duration duration = nextstop_time - time;
        times.push_back(std::make_pair(time, duration));
    }
    return times;
}

std::vector<std::tuple<StopID, RouteID, Time> > Datastructures::journey_earliest_arrival(StopID fromstop, StopID tostop, Time starttime)
{
    if ( stops_map.find(fromstop) == stops_map.end() || stops_map.find(tostop) == stops_map.end() || routes_with_stop.find(tostop) == routes_with_stop.end() || routes_with_stop.find(fromstop) == routes_with_stop.end()) {
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    }

    std::unordered_set<StopID> visited;
    std::priority_queue<std::pair<Node, Time>, std::vector<std::pair<Node, Time>>, time_comparison> pri_queue;

    std::vector<RouteID> routes = routes_with_stop.find(fromstop)->second;

    for (const auto & route : routes) {
        Node node{route, fromstop};
        auto timetable_pair = get_earliest_departure(route, fromstop, starttime);
        if (timetable_pair.second != -1) {
            pri_queue.push(std::make_pair(node, timetable_pair.first.at(timetable_pair.second) - starttime));
        }
    }

    std::unordered_map<Node, std::pair<Node,Time>, NodeHash> parentMap;
    std::pair<Node,int> found = {{NO_ROUTE, NO_STOP}, NO_TIME};

    while (!pri_queue.empty()) {          // repeat while there are vertices
        auto next = pri_queue.top();
        pri_queue.pop();                  // take the next vertex from queue

        if (visited.find(next.first.stop_id) != visited.end()) {
            continue;
        }
        if (next.first.stop_id == tostop) { // if vertex found, stop
            found = next;
            break;
        }
        const std::vector<Node> neighbors = find_neighbors(next.first);
        visited.insert(next.first.stop_id);
        for (const auto &neighbor : neighbors) {
            double nextToNeighborTime = calculate_time(next.first, neighbor, next.second + starttime);
            if (nextToNeighborTime != NO_TIME) {
                pri_queue.push(std::make_pair(neighbor, next.second + nextToNeighborTime));
                parentMap[neighbor] = next;
            }
        }
    }

    if (found.first.stop_id != tostop) {
        return {};
    }

    std::vector<std::pair<Node,int>> route_nodes = path(parentMap, fromstop, found);
    std::vector<std::tuple<StopID, RouteID, Time>> result = {};
    for (auto iter = route_nodes.rbegin(); iter != route_nodes.rend(); iter ++) {
        result.push_back(std::make_tuple(iter->first.stop_id, iter->first.route_id, iter->second + starttime));
    }
    return result;
}

void Datastructures::add_walking_connections()
{
    // Replace this comment and the line below with your implementation
}


// helper functions
std::vector<std::shared_ptr<Datastructures::Region>> Datastructures::descendants(const std::shared_ptr<Region> region) {
    std::vector<std::shared_ptr<Region>> regions = {};
    std::vector<std::shared_ptr<Region>> to_be_processed = {region};
    while (!to_be_processed.empty()) {
        const auto next = to_be_processed.at(to_be_processed.size() - 1);
        to_be_processed.pop_back();
        regions.push_back(next);
        for (auto child : next->children) {
            to_be_processed.push_back(child.lock());
        }
    }
    return regions;
}

std::vector<std::pair<Node,int>> Datastructures::BFS(StopID fromstop, StopID tostop) {      // Breadth first search
    std::unordered_set<StopID> visited;
    std::queue<std::pair<Node,int>> queue;
    std::vector<RouteID> routes = routes_with_stop.find(fromstop)->second;
    for (const auto & route : routes) {
        Node n{route, fromstop};
        queue.push(std::make_pair(n,0));
    }
    std::unordered_map<Node, std::pair<Node,int>, NodeHash> parentMap;
    std::pair<Node, int> found =  {{NO_ROUTE, NO_STOP}, NO_DISTANCE};
    while (!queue.empty()) {          // repeat while there are vertices
        auto next = queue.front();
        queue.pop();                  // take the next vertex from queue

        if (visited.find(next.first.stop_id) != visited.end()) {
            continue;
        }
        if (next.first.stop_id == tostop) { // if vertex found, stop
            found = next;
            break;
        }
        const std::vector<Node> neighbors = find_neighbors(next.first);
        visited.insert(next.first.stop_id);
        for (const auto &neighbor : neighbors) {
            double nextToNeighborDistance = calculate_distance(next.first.stop_id, neighbor.stop_id);
            queue.push(std::make_pair(neighbor, next.second + nextToNeighborDistance));
            parentMap[neighbor] = next;
        }
    }
    if (found.first.stop_id != tostop) {
        return {};
    }
    return path(parentMap, fromstop, found);
}

std::vector<Node> Datastructures::find_neighbors(Node node) {
    // return the neighbors of the given node
    std::vector<Node> neighbors = {};
    if (routes_with_stop.find(node.stop_id) != routes_with_stop.end()) {
        auto routes = routes_with_stop.find(node.stop_id)->second;
        for (const auto route : routes) {
            std::vector<StopID> stops_inside_route = routes_map.at(route)->stops;
            auto stop_index = routes_map.at(route)->stops_index;
            const auto idx = stop_index.find(node.stop_id);
            if (idx != stop_index.end() && idx->second < stops_inside_route.size() - 1) {
                // add a node (route name, stop name) as a neighbor
                neighbors.push_back({routes_map.at(route)->id, stops_inside_route.at(idx->second+1)});
            }
        }
    }
    return neighbors;
}

std::vector<std::pair<Node,int>> Datastructures::path(const std::unordered_map<Node, std::pair<Node,int>, NodeHash>& parentMap, StopID fromstop, std::pair<Node,int> endNode) {
    std::vector<std::pair<Node,int>> result{endNode};
    auto current = endNode;
    auto previous = endNode.first.route_id;
    while (current.first.stop_id != fromstop) {
        auto iter = parentMap.find(current.first);
        current = iter->second;
        Node copy{previous, current.first.stop_id};
        previous = current.first.route_id;
        result.push_back({copy, current.second});
    }
    result.at(0).first.route_id = NO_ROUTE;
    return result;
}


Distance Datastructures::calculate_distance(StopID from, StopID to) {
    auto stop_dist_map = stop_distance.find(from);
    if (stop_dist_map != stop_distance.end()) {
        auto stop_dist = stop_dist_map->second.find(to);
        if (stop_dist != stop_dist_map->second.end()) {
            return stop_dist->second;
        }
    } else {
        stop_distance.insert({from, {}});
    }
    auto node1_coord = stops_map.at(from)->coord;
    auto node2_coord = stops_map.at(to)->coord;
    int dist = floor(sqrt(pow((node1_coord.x - node2_coord.x),2) + pow((node1_coord.y - node2_coord.y),2)));
    stop_distance.find(from)->second.insert({to, dist});
    return dist;
}


std::pair<std::vector<Time>, int> Datastructures::get_earliest_departure(RouteID routeid, StopID stopid, Time time) {
    auto stops = routes_map.at(routeid)->stops;
    auto timetables = routes_map.at(routeid)->timetables;
    const auto index = routes_map.at(routeid)->stops_index.find(stopid)->second;
    std::vector<Time> best_timetable = {};
    int best_index = -1;
    for (const auto & timetable : timetables) {
        if (timetable.at(index) >= time) {
            if (best_timetable.empty() || timetable.at(index) <= best_timetable.at(best_index)) {
                best_index = index;
                best_timetable = timetable;
            }
        }
    }
    return std::make_pair(best_timetable, best_index);
}

Time Datastructures::calculate_time(Node from, Node to, Time time) {
    auto timetable_pair = get_earliest_departure(to.route_id, from.stop_id, time);
    if (timetable_pair.second != -1) {
        const auto nextstop_arrival_time = timetable_pair.first.at(timetable_pair.second +1 );
        return nextstop_arrival_time - time;
    }
    return NO_TIME;
}
