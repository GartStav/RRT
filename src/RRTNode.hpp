#ifndef RRTNODE_HPP
#define RRTNODE_HPP

#include <vector>
#include <cmath>
#include <memory>

// Node of the tree build by RRT algorithm

class RRTNode {
public:

    RRTNode(std::vector<double> config, int parent_id);

    std::vector<double> get_config();

    double distance_to_conf(std::vector<double> config, std::vector<double> weights);

    int id_;

    int parent_id_;

    std::vector<double> config_;

private:

    static int current_id_;

};
#endif
