#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

    end_node->g_value=std::numeric_limits<float>::max();
    end_node->h_value=0;

    start_node->h_value=CalculateHValue(start_node);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    if (current_node == nullptr) return;

    current_node->FindNeighbors();

    for(auto &neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
    current_node->visited = true;
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(),
              open_list.end(),
              [](auto node_a, auto node_b) 
              {
                return node_a->h_value + node_a->g_value <= node_b->h_value + node_b->g_value;
                } ); 

    auto next = *open_list.begin();
    open_list.erase(open_list.begin());
    return next;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    if(start_node->distance(*end_node) == 0)
    {
        std::cout << "End point is iqual to start point, please select another pair of points" << std::endl;
        path_found.push_back(*current_node);
        return path_found;
    } 

    RouteModel::Node *node = current_node;
    while(node != start_node)
    {
        distance += node->distance(*(node->parent));
        path_found.push_back(*node);

        node = node->parent;
    }
    
    path_found.push_back(*start_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    
    uint8_t i = 0;
    while(current_node != end_node)
    {
        AddNeighbors(current_node);
        current_node->visited = true;
        current_node = NextNode();
    }

    m_Model.path = ConstructFinalPath(current_node);

}