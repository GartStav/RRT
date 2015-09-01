#include "RRTTree.hpp"
#include "RRTNode.hpp"
#include <iostream>

NodeTree::NodeTree() {
    nodes_.reserve(1000);
}

std::vector<RRTNode>* NodeTree::get_nodes() {
    return &nodes_;
}

int NodeTree::add_node(std::vector<double> config, int parent_id) {
    RRTNode new_node = RRTNode(config, parent_id);
    nodes_.push_back(new_node);
    return new_node.id_;
}

int NodeTree::nearest_neighbour(std::vector<double> config, std::vector<double> weights) {

    int closest_id = nodes_.at(0).id_;
    double closest_dist = nodes_.at(0).distance_to_conf(config, weights);


    for (int i = 1; i < nodes_.size(); i++) {
        double new_dist = nodes_.at(i).distance_to_conf(config, weights);
        //std::cout << "Other Node distance is " << new_dist << std::endl;
        if (new_dist < closest_dist) {
            //std::cout << "ITS LESS " << std::endl;
            closest_id = nodes_.at(i).id_;
            closest_dist = new_dist;
        }
    }

    return closest_id;
}


