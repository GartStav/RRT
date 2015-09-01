#ifndef RRTTREE_HPP
#define RRTTREE_HPP

#include <vector>
#include "RRTNode.hpp"

class NodeTree {

public:

    NodeTree();
    std::vector<RRTNode>* get_nodes();
    int add_node(std::vector<double> config, int parent_id);
    bool delete_node(int id);
    int nearest_neighbour(std::vector<double> config, std::vector<double> weights);

private:

    std::vector<RRTNode> nodes_;

};
#endif

