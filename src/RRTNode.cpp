#include "RRTNode.hpp"

std::vector<double> RRTNode::get_config() {
    return config_;
}

RRTNode::RRTNode(std::vector<double> config, int parent_id)  {
    config_ = config;
    parent_id_ = parent_id;

    this->id_ = current_id_;
    this->current_id_++;
    //current_id_++;
    //id_ = current_id_;
}

int RRTNode::current_id_ = 0;

double RRTNode::distance_to_conf(std::vector<double> config, std::vector<double> weights) {

    double distance = 0.0;

    bool use_weights;

    if (weights.empty()) {
        use_weights = false;
    }
    else {
        use_weights = true;
    }

    for (int i=0; i < config_.size(); i++) {
        double difference = config_.at(i) - config.at(i);
        if (use_weights) {
            difference *= weights.at(i)*difference;
        }
        distance += pow(difference, 2.0);
    }
    return sqrt(distance);

}
