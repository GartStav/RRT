#ifndef RRTCONNECT_HPP
#define RRTCONNECT_HPP

#include "RRTTree.hpp"
#include <random>
#include "openrave/openrave.h"

class RRTConnect {

public:

    RRTConnect(OpenRAVE::EnvironmentBasePtr penv, OpenRAVE::RobotBasePtr robot, std::vector<double> joints_min, std::vector<double> joints_max);

    std::pair< std::vector<RRTNode>,std::vector<RRTNode> > RunRRT(std::vector<double> q_init, std::vector<double> q_goal, double goal_bias, double step_size, int num_iter);
    std::pair< bool,std::vector<double> > GetSample(double goal_bias);
    std::pair<bool, RRTNode> PerformConnect(int closest_id, std::vector<double> sample, double step);
    void Smoothing(std::vector<RRTNode>& path, double step_size);
    std::pair<bool, std::vector<RRTNode> > con(std::vector<double> start, std::vector<double> end, double step_size);
    bool in_collision(std::vector<double> config);
    double random_value(double min, double max);

    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::RobotBasePtr robot_;

    std::vector<double> q_init_;
    std::vector<double> q_goal_;

    std::vector<double> joints_min_;
    std::vector<double> joints_max_;

    std::default_random_engine generator_;

    NodeTree tree_;

    std::vector<double> weights_;

};

#endif
