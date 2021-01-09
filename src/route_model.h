#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

public:
    class Node : public Model::Node {
    friend std::ostream& operator<<(std::ostream &strm, const RouteModel::Node &n) {
        strm << "Node: {x:"<< n.x <<"} {y:"<< n.y << "} {g:"<< n.g_value <<
             "} {h:"<< n.h_value <<"} {visited?:"<< n.visited <<"}";
//        "} {h:"<< n.h_value <<"} {visited?:"<< (n.visited?"yes":"no") <<"}";
    }
    public:
        // Add public Node variables and methods here.
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

        float distance(const Node n) const {
            return sqrt(pow(x - n.x, 2) + pow(y - n.y, 2));
        }

        void FindNeighbors();

        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0f;
        bool visited = false;
        std::vector<Node*> neighbors;

    private:
        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;
        RouteModel::Node* FindNeighbor(std::vector<int> /*node_indices*/);

    };

    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

    auto &SNodes() { return m_Nodes; }

    auto &GetNodeToRoadMap() { return node_to_road; }

    RouteModel::Node& FindClosestNode(float, float);

private:
    // Add private RouteModel variables and methods here.
    std::vector<RouteModel::Node> m_Nodes;
    std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;

    void CreateNodeToRoadHashmap();
};

#endif