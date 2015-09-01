/** \author Artem Gritsenko */

#ifndef RRTMODULE_H
#define RRTMODULE_H


#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cassert>
#include <random>

#include "RRT-Connect.hpp"

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <boost/assign/list_of.hpp>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

///using namespace OpenRAVE;



class RRTModule: public OpenRAVE::ModuleBase
{
    public:

        RRTModule(EnvironmentBasePtr penv);
        ~RRTModule();

        bool Init(std::ostream& sout);
        bool StartPlanning(std::ostream& sout, std::istream& sinput);
        bool SetStart ( std::ostream &sout, std::istream &sinput );
        bool SetGoal ( std::istream& sinput );
        bool SetGoalBias ( std::istream& sinput );
        bool SetStepSize ( std::istream& sinput );

        OpenRAVE::EnvironmentBasePtr env_;
        OpenRAVE::RobotBasePtr robot_;

        std::vector<int> joint_indexes_;
        std::vector<double> joints_min_;
        std::vector<double> joints_max_;
        std::vector<double> start_;
        std::vector<double> goal_;

        double goal_bias_;
        double step_size_;
        int num_iter_;

    //    virtual ~RRTModule();

};

#endif
