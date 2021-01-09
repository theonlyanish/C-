#include "route_model.h"
#include <iostream>

using std::cout;
using std::endl;

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    auto nodes = this->Nodes();
    for (int i = 0; i < nodes.size(); i++) {
        Node n{i, this, nodes.at(i)};
        this->m_Nodes.push_back(n);
    }

    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
    for (const Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx : Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = std::vector<const Model::Road*>{};
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    for (auto idx : node_indices) {
        Node n = parent_model->SNodes()[idx];
        if (!n.visited && this->distance(n) != 0) {
            if (closest_node == nullptr || this->distance(n) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[idx];
                }
        }
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbors() {
    for (auto & r : parent_model->node_to_road[this->index]) {
        auto node_indices = parent_model->Ways()[r->way].nodes;
        RouteModel::Node* neighbor = this->FindNeighbor(node_indices);
        if (neighbor) {
            this->neighbors.push_back(neighbor);
        }
    }
}

RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
    RouteModel::Node target_node;
    target_node.x = x;
    target_node.y = y;

    float min_dist = std::numeric_limits<float>::max();
    int closest_idx = -1;

    for (auto &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            for (auto a_node_idx : Ways()[road.way].nodes) {
                // Update closest_idx and min_dist, if needed
                auto a_node = SNodes()[a_node_idx];
                float a_distance = a_node.distance(target_node);
                if (a_distance < min_dist) {
                    closest_idx = a_node_idx;
                    min_dist = a_distance;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}
