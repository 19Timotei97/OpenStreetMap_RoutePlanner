#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // m_Model.FindClosestNode is used method to find the closest nodes to the starting and ending coordinates.
    // Stores the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


// CalculateHValue method.
// Uses the distance to the end_node for the h value.
// Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    return node->distance(*(end_node));
}


// AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.
// Uses the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// For each node in current_node.neighbors, sets the parent, the h_value, the g_value. 
// Uses CalculateHValue below to implement the h-Value calculation.
// For each node in current_node.neighbors, adds the neighbor to open_list and sets the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();

    for (RouteModel::Node* neighbour : current_node->neighbors) {
        neighbour->parent = current_node;
        neighbour->h_value = CalculateHValue(neighbour);
        neighbour->g_value += current_node->g_value + neighbour->distance(*(current_node));

        open_list.emplace_back(neighbour);
        neighbour->visited = true;
    }
}


// NextNode() method sorts the open list and returns the next node.
// Sorts the open_list according to the sum of the h value and g value.
// Creates a pointer to the node in the list with the lowest sum.
// Removes that node from the open_list.
// Returns the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(open_list.begin(), open_list.end(), [](const auto& node1, const auto& node2)
                                    { return (node1->h_value + node1->g_value) < (node2->h_value + node2->g_value); });

    RouteModel::Node* lowerSumNode = open_list.front();
    open_list.erase(open_list.begin());

    return lowerSumNode;
}


// ConstructFinalPath method returns the final path found from the A* search.
// This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// For each node in the chain, adds the distance from the node to its parent to the distance variable.
// The returned vector is in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        // Add the node to the path
        path_found.emplace_back(*(current_node));

        // Compute distance to its parent
        distance += current_node->distance(*(current_node->parent));

        // Continute up the chain of parents
        current_node = current_node->parent;
    }    

    // Add the last parent to the path
    path_found.emplace_back(*current_node);

    // Mantain the order requested
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// A* Search algorithm 
// AddNeighbors() method adds all of the neighbors of the current node to the open_list.
// NextNode() method sorts the open_list and return the next node.
// When the search has reached the end_node, ConstructFinalPath() method returns the final path that was found.
// The final path is stored in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;
    open_list.emplace_back(start_node);    

    while (open_list.size() > 0) {

        current_node = NextNode();

        if (current_node->distance(*end_node) == 0) 
        {
            m_Model.path = ConstructFinalPath(end_node);
            return;
        } 
        else 
        {
            AddNeighbors(current_node);
        }      
    }
}