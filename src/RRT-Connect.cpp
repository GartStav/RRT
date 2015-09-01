#include "RRT-Connect.hpp"
#include <iostream>
#include "openrave/viewer.h"
#include <chrono>

RRTConnect::RRTConnect(OpenRAVE::EnvironmentBasePtr penv, OpenRAVE::RobotBasePtr robot, std::vector<double> joints_min, std::vector<double> joints_max) {

    env_ = penv;
    robot_ = robot;
    joints_min_ = joints_min;
    joints_max_ = joints_max;

    robot_->GetActiveDOFWeights(weights_);




    //std::default_random_engine generator_();

    std::cout << "Initialization of RRT done." << std::endl;

}

bool compare_configs(std::vector<double> v1, std::vector<double> v2) {
    double eps = 0.01;

//    std::cout << "Comparing configs" << std::endl;

//    int size = v1.size();

//    std::cout << "1st conf" << std::endl;
//    for (int i = 0; i < size; i++) {
//        std::cout << "joint value " << v1.at(i) << std::endl;
//    }

//    std::cout << "2nd conf" << std::endl;
//    for (int i = 0; i < size; i++) {
//        std::cout << "joint value " << v2.at(i) << std::endl;
//    }

    for (int i = 0; i < v1.size(); i++) {
//        std::cout << (fabs(v1.at(i) - v2.at(i))) << std::endl;
        if ( (float)(fabs(v1.at(i) - v2.at(i))) > eps ) return false;
    }

    std::cout << "Configs are the same!!!!!!!" << std::endl;
    return true;

}

std::pair<bool, std::vector<RRTNode> > RRTConnect::con(std::vector<double> start, std::vector<double> end, double step_size){
    double axis_dist = step_size / RRTNode(start, -1).distance_to_conf(end, weights_);
    int step_count = (int) floor(1.0 / axis_dist);

    std::vector<double> step_vec;
    for (uint i = 0; i < start.size(); i++) {
        step_vec.push_back((end.at(i) - start.at(i)) * axis_dist);
    }

    std::vector<double> current_step = start;

    std::vector<RRTNode> points;
    int steps;
    for (steps = 1; steps < step_count; steps++) {
        current_step.clear();

        for (uint i = 0; i < start.size(); i++) {
            current_step.push_back(start.at(i) + step_vec.at(i)*steps);
        }


        if (in_collision(current_step)) {
            return std::pair<bool, std::vector<RRTNode> >(false, std::vector<RRTNode>());
        }

        RRTNode new_node(current_step, -1);
        points.push_back(new_node);
        tree_.get_nodes()->push_back(new_node);
    }
    return std::pair<bool, std::vector<RRTNode> >(true, points);
}

void RRTConnect::Smoothing(std::vector<RRTNode>& path, double step_size){

    for(int i = 0; i < 200; i++) {

        int first_point = int(random_value(0.0, path.size()-1));
        int second_point = int(random_value(first_point, path.size()-1));;

        std::vector<double> start = path.at(first_point).get_config();
        std::vector<double> end = path.at(second_point).get_config();

        //std::vector<float> smoother;
        if(con(start, end, step_size).first) {

            std::pair<bool, std::vector<RRTNode> > segment;


            double norm_dist = step_size / RRTNode(start, -1).distance_to_conf(end, weights_);
            int num_steps = (int) floor(1.0 / norm_dist);

            std::vector<double> shag;
            for (uint i = 0; i < start.size(); i++) {
                shag.push_back((end.at(i) - start.at(i)) * norm_dist);
            }

            std::vector<double> next_step = start;

            std::vector<RRTNode> points;
            int s;
            for (s = 1; s < num_steps; s++) {
                next_step.clear();

                for (uint i = 0; i < start.size(); i++) {
                    next_step.push_back(start.at(i) + shag.at(i)*s);
                }


                if (in_collision(next_step)) {
                    segment = std::pair<bool, std::vector<RRTNode> >(false, std::vector<RRTNode>());
                }

                RRTNode new_node(next_step, -1);
                points.push_back(new_node);
                tree_.get_nodes()->push_back(new_node);
            }
            segment = std::pair<bool, std::vector<RRTNode> >(true, points);

            path.erase(path.begin()+first_point, path.begin()+second_point);
            path.insert(path.begin()+first_point, segment.second.begin(), segment.second.end());
        }
    }
}

std::pair< std::vector<RRTNode>,std::vector<RRTNode> > RRTConnect::RunRRT(std::vector<double> q_init, std::vector<double> q_goal, double goal_bias, double step_size, int num_iter) {

    q_init_ = q_init;
    q_goal_ = q_goal;

    tree_.add_node(q_init, 0);

    bool goal_found = false;

    int curr_id;

    std::cout << "Starting RRT search." << std::endl;


    std::vector<OpenRAVE::GraphHandlePtr> goal_handle;
    //std::vector<float> pt1;

    for (int it = 0; it < num_iter; it++) {

//        std::cout << "Iteration ";
//        std::cout << it << std::endl;

        std::pair< bool,std::vector<double> > sample = GetSample(goal_bias);

        int closest_id = tree_.nearest_neighbour(sample.second, weights_);

//        std::cout << "closest_id is ";

//        std::cout << closest_id << std::endl;

        //std::cout << "size of the tree is ";
        //std::cout << tree_.get_nodes()->size() << std::endl;

        // TODO change this block later. Draw stuff

//        robot_->SetActiveManipulator("leftarm");
//        robot_->SetActiveDOFValues(tree_.get_nodes()->at(tree_.nearest_neighbour(q_goal, weights_)).get_config());
//        OpenRAVE::RobotBase::ManipulatorPtr manip = robot_->GetActiveManipulator();
//        OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
//        std::vector<float> pt1 = {(float)trans.x, (float)trans.y, (float)trans.z};
//        std::vector<float> colors = {1,0,0,1};
//        goal_handle.push_back(env_->plot3( &pt1[0], 1, sizeof(pt), 20.0, &colors[0], 0, true ));



        //goal_handle->SetShow(true);

        std::pair<bool, RRTNode> better_sample = PerformConnect(closest_id, sample.second, step_size);

        curr_id = better_sample.second.id_;


//        robot_->SetActiveManipulator("leftarm");
//        robot_->SetActiveDOFValues(tree_.get_nodes()->at(curr_id).get_config());
//        OpenRAVE::RobotBase::ManipulatorPtr manip = robot_->GetActiveManipulator();
//        OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
//        std::vector<float> pt = {(float)trans.x, (float)trans.y, (float)trans.z};
//        std::vector<float> colors = {0,1,0,1};
//        goal_handle.push_back(env_->plot3( &pt[0], 1, sizeof(pt), 20.0, &colors[0], 0, true ));

//        if (compare_configs(better_sample.get_config(), q_goal)) {
//            goal_found = true;
//            break;
//        }

        if (sample.first && better_sample.first) {
            goal_found = true;
            break;
        }
        //goal_handle->SetShow(true);
    }

//    do {
//        std::cout << '\n' <<'Press the Enter key to continue.';
//    } while (std::cin.get() != '\n');

    std::vector<RRTNode> reversed_path;
    std::vector<RRTNode> raw_path;
    std::vector<RRTNode> smoothed_path;

    //goal_handle->SetShow(true);

    if (goal_found) {
        std::cout << "Solution is found" << std::endl;


        // TODO change this block
        while (true) {
            reversed_path.push_back(tree_.get_nodes()->at(curr_id));
            if (curr_id == 0) {
                break;
            }
            curr_id = tree_.get_nodes()->at(tree_.get_nodes()->at(curr_id).parent_id_).id_;
        }
        for(long i = reversed_path.size(); i-- > 0;) {
            raw_path.push_back(reversed_path.at(i));
        }

        for(long i = raw_path.size(); i-- > 0;) {
            smoothed_path.push_back(raw_path.at(i));
        }

        Smoothing(smoothed_path, step_size);


    } else {
        std::cout << "Goal was not found after maximum number of iterations" << std::endl;
        return std::pair< std::vector<RRTNode>,std::vector<RRTNode> > (raw_path, smoothed_path);
    }

    return std::pair< std::vector<RRTNode>,std::vector<RRTNode> > (raw_path, smoothed_path);
}

std::pair< bool,std::vector<double> > RRTConnect::GetSample(double goal_bias){

    std::vector<double> new_sample;

    int size =  q_goal_.size();

    if ( random_value(0.0, 1.0) < goal_bias) {
        return std::pair<bool, std::vector<double> >(true, q_goal_);
    }

    while(true) {
        new_sample.clear();
//        std::cout << "new sample" << std::endl;
        for (int i = 0; i < size; i++) {
            double r = random_value(joints_min_.at(i),joints_max_.at(i));
//            std::cout << "push back" << r << std::endl;
            new_sample.push_back(r);
        }
        if (!in_collision(new_sample)) {
            return std::pair<bool, std::vector<double> >(false, new_sample);
        }
    }
}

// TODO connect
std::pair<bool, RRTNode> RRTConnect::PerformConnect(int closest_id, std::vector<double> sample, double step){

    int size = sample.size();

//    std::cout << "goal conf" << std::endl;
//    for (int i = 0; i < size; i++) {
//        std::cout << "joint value " << sample.at(i) << std::endl;
//    }

//    std::cout << "start conf" << std::endl;
    std::vector<double> initial_conf = tree_.get_nodes()->at(closest_id).get_config();
//    for (int i = 0; i < size; i++) {
//        std::cout << "joint value " << initial_conf.at(i) << std::endl;
//    }

    double normalized_dist = step / tree_.get_nodes()->at(closest_id).distance_to_conf(sample, weights_);

    //int num_steps = (ceil)(tree_.get_nodes()->at(closest_id).distance_to_conf(sample, weights_)/step);
    int num_steps = (int)floor(1.0 / normalized_dist);

    std::vector<double> step_distance;

    for (uint i = 0; i < sample.size(); i++) {
        RRTNode nd = tree_.get_nodes()->at(closest_id);
        step_distance.push_back((sample.at(i) - nd.config_.at(i)) * normalized_dist);
    }

    int best_id = closest_id;


    std::vector<double> next_step;

    int s;
    for (s = 1; s < num_steps; s++) {
        next_step.clear();

//        std::cout << "Next step" << std::endl;
        for (uint i = 0; i < sample.size(); i++) {
            next_step.push_back(initial_conf.at(i) + step_distance.at(i)*s);
//            std::cout << "joint value " << next_step.at(i) << std::endl;
        }

        if (in_collision(next_step)) {
            break;
        }
        else {
            int new_id = tree_.add_node(next_step, tree_.get_nodes()->at(best_id).id_);
            best_id = tree_.get_nodes()->at(new_id).id_;
        }
    }


    bool found;
    if (s == num_steps) {
        best_id = tree_.add_node(sample, tree_.get_nodes()->at(best_id).id_);
        found = true;
    } else {
        found = false;
    }

    return std::pair<bool, RRTNode>(found, tree_.get_nodes()->at(best_id));



    //std::cout << "NEXXXTTT" << std::endl;
    //return tree_.get_nodes()->at(best_id);

}




bool RRTConnect::in_collision(std::vector<double> config){

    robot_->SetActiveDOFValues(config);

    bool check = env_->CheckCollision(robot_) || robot_->CheckSelfCollision();

    return check;
}

double RRTConnect::random_value(double min, double max){

    //std::uniform_real_distribution<double> distribution(min, max);
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<double> dist(min, max);

    //double g_prob = g_dist(gen);

    //return distribution(generator_);

    return dist(gen);

}
