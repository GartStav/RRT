RRTModule(EnvironmentBasePtr penv) : ModuleBase(penv) {

    RegisterCommand("Init",boost::bind(&RRTModule::Init,this,_1),
                    "Initialize all setting for the planner");
    RegisterCommand("SetStart",boost::bind(&RRTModule::SetStart,this,_1,_2),
                    "Set up the start configuration for the planner");
    RegisterCommand("SetGoal",boost::bind(&RRTModule::SetGoal,this,_2),
                    "Set up the goal configuration the planner");
    RegisterCommand("SetGoalBias",boost::bind(&RRTModule::SetGoalBias,this,_2),
                    "Set up the goal bias for the planner");
    RegisterCommand("SetStepSize",boost::bind(&RRTModule::SetStepSize,this,_2),
                    "Set up the step size for the planner");
    RegisterCommand("StartPlanning",boost::bind(&RRTModule::StartPlanning,this,_1,_2),
                    "Start planner");

    env_ = penv;

}

bool RRTModule::Init(std::ostream& sout ) {

    sout << "Initialize the planner";
    std::vector<RobotBasePtr> robots = std::vector<RobotBasePtr>();
    env_->GetRobots(robots);
    robot_ = robots.at(0);
    num_iter_ = 25000;

    for (int i=0; i < robot_->GetActiveDOFIndices().size(); i++) {
        joint_indexes_.push_back(robot_->GetActiveDOFIndices().at(i));
    }
    robot_->GetDOFLimits(joints_min_, joints_max_, joint_indexes_);

    for (int i = 0; i < joint_indexes_.size(); i++) {
        if(robot_->GetJointFromDOFIndex(joint_indexes_.at(i))->IsCircular(0)) {
            joints_min_.at(i) = -2*M_PI;
            joints_max_.at(i) =  2*M_PI;
        }
    }

    std::cout << std::endl;
    for(uint i = 0; i < joint_indexes_.size(); i++){
        std::cout << "Joint Idx: " << joint_indexes_.at(i) << std::endl;
        std::cout << "Joint Name: " << robot_->GetJointFromDOFIndex(joint_indexes_.at(i))->GetName() << std::endl;
        std::cout << "Min: " << joints_min_.at(i) << std::endl;
        std::cout << "Max: " << joints_max_.at(i) << std::endl;
        std::cout << std::endl;
    }

    sout << "RRT planner initialized";

    return true;
}

bool RRTModule::SetStart ( std::ostream &sout, std::istream &sinput )
{
    std::cout << "Set Start" << std::endl;

    start_.clear();

    std::string str;
    getline(sinput, str);
    std::stringstream ss(str);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> proc_(begin, end);

    std::cout << proc_.size() << std::endl;

    for(uint i = 0; i < proc_.size(); i++) {
        std::cout << proc_.at(i) << std::endl;
    }

    for(uint i = 0; i < proc_.size(); i++) {
        if(proc_.at(i)[0] == '-'){
            switch(proc_.at(i)[1]) {
                case 's':
                    for(int j_idx = 0; j_idx < joint_indexes_.size(); j_idx++){
                        i++;
                        start_.push_back(atof(proc_.at(i).c_str()));
                    }
                    break;
            }
        }
    }

    return true;
}

bool RRTModule::SetGoal( std::istream& sinput )
{
    std::cout << "Set Goal" << std::endl;

    std::string str;
    getline(sinput, str);
    std::stringstream ss(str);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> proc_(begin, end);

    std::cout << proc_.size() << std::endl;

    for(uint i = 0; i < proc_.size(); i++) {
        std::cout << proc_.at(i) << std::endl;
    }

    for(uint i = 0; i < proc_.size(); i++) {
        if(proc_.at(i)[0] == '-'){
            switch(proc_.at(i)[1]) {
                case 'g':
                    for(int j_idx = 0; j_idx < joint_indexes_.size(); j_idx++){
                        i++;
                        goal_.push_back(atof(proc_.at(i).c_str()));
                    }
                    break;
            }
        }
    }

    return true;
}

bool RRTModule::SetGoalBias( std::istream& sinput )
{
    std::cout << "Set Goal Bias" << std::endl;
    std::string cmd;

    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "bias" )
        {
            sinput >> goal_bias_;
        }
        else break;
        if( !sinput ) {
            std::cout << "Error in setting the goal bias" << std::endl;
            return false;
        }
    }

    return true;
}

bool RRTModule::SetStepSize( std::istream& sinput )
{
    std::cout << "Set Step Size" << std::endl;
    std::string cmd;

    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "step" )
        {
            sinput >> step_size_;
        }
        else break;
        if( !sinput ) {
            std::cout << "Error in setting the step size" << std::endl;
            return false;
        }
    }

    return true;
}

bool RRTModule::StartPlanning(std::ostream& sout, std::istream& sinput)
{
    std::string input;
    sinput >> input;

    std::cout << "Starting pose: ";
    for(int j=0; j < start_.size(); j++){
        std::cout << start_.at(j) << ", ";
    }
    std::cout << std::endl;

    std::cout << "Goal Pose: ";
    for(int j=0; j < goal_.size(); j++){
        std::cout << goal_.at(j) << ", ";
    }

    std::cout << std::endl;

    std::cout << "Step size: " << step_size_ << std::endl;

    std::cout << "Goal sampling frequency: " << goal_bias_ << std::endl;

    sout << "Start planner";

    RRTConnect* rrtplanner = new RRTConnect(env_, robot_, joints_min_, joints_max_);
    std::pair< std::vector<RRTNode>,std::vector<RRTNode> > resulting_path = rrtplanner->RunRRT(start_, goal_, goal_bias_, step_size_, num_iter_);

    std::cout << "Path acquired, drawing.." << std::endl;

    // TODO redo this part
    std::vector<OpenRAVE::GraphHandlePtr> handles;
    std::vector<float> points;
    for (uint i = 0; i < resulting_path.first.size(); i++) {
        robot_->SetActiveDOFValues(resulting_path.first.at(i).get_config());
        OpenRAVE::RobotBase::ManipulatorPtr manip = robot_->GetActiveManipulator();
        OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
        points.push_back((float)trans.x);
        points.push_back((float)trans.y);
        points.push_back((float)trans.z);
    }

//    std::vector<float> points2;
//    for (uint i = 0; i < resulting_path.second.size(); i++) {
//        robot_->SetActiveDOFValues(resulting_path.second.at(i).get_config());
//        OpenRAVE::RobotBase::ManipulatorPtr manip = robot_->GetActiveManipulator();
//        OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
//        points2.push_back((float)trans.x);
//        points2.push_back((float)trans.y);
//        points2.push_back((float)trans.z);
//    }

    handles.push_back(env_->plot3(&points[0], (int)(points.size()/3), sizeof(float)*3, 7.0, OpenRAVE::RaveVector<double>(1.0, 0.3, 0.3, 1)));

    //handles.push_back(env_->plot3(&points2[0], (int)(points.size()/3), sizeof(float)*3, 7.0, OpenRAVE::RaveVector<double>(0.0, 1.0, 0.3, 1)));

    do {
        std::cout << '\n' <<'Press the Enter key to continue.';
    } while (std::cin.get() != '\n');

    env_->GetMutex().unlock();
    for (int i = 0; i < resulting_path.first.size(); i++) {
        std::vector<double> v;

        env_->GetMutex().lock();
        robot_->SetActiveDOFValues(resulting_path.first.at(i).get_config());
        robot_->GetDOFValues(v);
        robot_->GetController()->SetDesired(v);
        env_->GetMutex().unlock();

        while (!robot_->GetController()->IsDone()) {
            usleep(10000);
        }
    }

    return true;
}
