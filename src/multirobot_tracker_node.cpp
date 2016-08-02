#include "multirobot_tracker_node.h"

using namespace multirobot_tracker;

//Main loop
int main (int argc, char **argv){
    ros::init(argc,argv,"multirobot_tracker");
    ros::NodeHandle nh;

    Tracker track;

    track.controlLoop();

    return 0;
}

Tracker::Tracker()
{
    ros::NodeHandle private_nh("~");

    private_nh.param("periode_start", periode_start_, 0.2);
    private_nh.param("periode_relax", periode_relax_, 5.0);

    periode_start_ = std::max(periode_start_, 0.1);
    periode_relax_ = std::max(periode_relax_, 0.1);

    if (periode_start_ > periode_relax_)
    {
        ROS_WARN("periode_start is higher as periode_relax");
        std::swap(periode_start_, periode_relax_);
    }

    current_periode_ = periode_start_;

    std::string _str2split;
    private_nh.param("topics2seach", _str2split, std::string("base_scan,odom"));
    getTopicReferences(_str2split);

    //Cleaning possible trash information
    robots_known_vec_.clear();
    new_robot_found_ = false;
    a_robot_died_    = false;

    //Preparing publications.
    //Note: the latch is set to true to send the last message published to the new subscribers.
    //This is important in the fact that we are only sending new messages when new robots appears or dissapears
    multirobot_track_pub_ =nh_.advertise<multirobot_tracker::multiStringStamped>("multirobot_track",1,true);
    track_of_robots_msg_.header.seq = 0;

    //Sends the first message. If no robots are available, this will be the only message sent by the node.
    sendTrackOfRobots();
}


void Tracker::controlLoop()
{

    while (ros::ok())
    {
        searchForRobots();

        makeRobotsOlder();

        ros::spinOnce();        //Necessary to allow callbacks (in case of inheritance could be useful)

        if (new_robot_found_ || a_robot_died_)
        {
            sendTrackOfRobots();
        }

        updateWorkingSpeed();
        ros::Duration(current_periode_).sleep();
    }
}


void Tracker::getTopicReferences(std::string str2split)
{
    boost::split(topic_references_,str2split, boost::is_any_of(","));
}


void Tracker::searchForRobots()
{
    #ifdef DEBUG
        ROS_INFO("Searching for new robots");
    #endif

    //To solve this, I took the information from http://docs.ros.org/diamondback/api/roscpp/html/namespaceros_1_1master.html
    //note: typedef std::vector<TopicInfo> ros::master::V_TopicInfo
    ros::master::V_TopicInfo _published_topics_list;

    //Getting the list of knowing topics
    if (ros::master::getTopics(_published_topics_list))
    {
        // Seaching for every reference topic
        for (std::vector<std::string>::iterator _it_topic_ref = topic_references_.begin();
            _it_topic_ref != topic_references_.end(); ++_it_topic_ref)
        {
            //Checking in all published topics names
            for (ros::master::V_TopicInfo::iterator _it_full_topics = _published_topics_list.begin();
                 _it_full_topics!=_published_topics_list.end(); _it_full_topics++)
            {
                std::string _full_topic = _it_full_topics->name;

                #ifdef DEBUG
                    ROS_INFO("Searching %s in %s",(*_it_topic_ref).c_str(),_full_topic.c_str());
                #endif // DEBUG

                if (std::string::npos!=_full_topic.find(*_it_topic_ref))
                {
                    #ifdef DEBUG
                        ROS_INFO("\tFound in robot %s", (extractGlobalNamespace(_full_topic)).c_str());
                    #endif

                    if (!isAKnownRobot( extractGlobalNamespace(_full_topic) ))
                    {
                        new_robot_found_ = true;
                        robots_known_vec_.push_back( extractGlobalNamespace(_full_topic) );
                    }
                }
            }
        }
    }
    #ifdef DEBUG
        ROS_INFO("Next search in %f sec", current_periode_);
    #endif

}


const std::string Tracker::extractGlobalNamespace(const std::string full_topic_name)
{
    std::string _global_namespace = full_topic_name;

    //We are expecting imputs like "/global_namespace/topic_name" in case of multiple robots, or "/topic_name" in case of only one robot
    std::size_t _first_slash = full_topic_name.find_first_of("/");

    if (_first_slash != std::string::npos)
    {
        //First slash found, looking for the second slash
        std::size_t _second_slash = full_topic_name.find_first_of("/",_first_slash+1);

        if (_second_slash==std::string::npos)
        {
            //Second slash not found, only one robot should be working
            _global_namespace ="/";
        }
        else
        {
            /*
                "/global_namespace/topic_name"
                 |<-   resize  ->|
            */
            _global_namespace.resize(_second_slash);
        }
    }
    else
    {
        ROS_WARN("Node %s uses relative namespace. Option not contempled in multirobot_tracker",full_topic_name.c_str());
    }

    return _global_namespace;
}


bool Tracker::isAKnownRobot(const std::string robot_name){
    bool onList=false;

    for (std::vector <RobotKnown>::iterator _it_robot = robots_known_vec_.begin();
        _it_robot!=robots_known_vec_.end() && !onList; ++_it_robot){
        if (_it_robot->isMyName(robot_name)){
            _it_robot->increaseLife();
            onList=true;
        }
    }

    #ifdef DEBUG
        if (onList){
            ROS_INFO("\t\tThe robot %s already exists",robot_name.c_str());
        }else{
            ROS_INFO("\t\tThe robot %s is a new robot",robot_name.c_str());
        }
    #endif // DEBUG

    return onList;
}

void Tracker::makeRobotsOlder()
{
    std::vector <RobotKnown>::iterator _it_robot = robots_known_vec_.begin();

    while (_it_robot != robots_known_vec_.end())
    {
        _it_robot -> decreaseLife();
        if ( _it_robot -> stillAlive() )
        {
            ++_it_robot;
        }
        else
        {
            a_robot_died_ = true;
            robots_known_vec_.erase(_it_robot);
            _it_robot = robots_known_vec_.begin();
        }
    }
}

void Tracker::sendTrackOfRobots()
{
    // Cleaning possible residual information from the message
    track_of_robots_msg_.Strings.clear();

    // Populate the message  robots_known_vec_
    for (std::vector <RobotKnown>::iterator _it_robot=robots_known_vec_.begin(); _it_robot!=robots_known_vec_.end(); ++_it_robot)
    {
        track_of_robots_msg_.Strings.push_back(_it_robot->getName());
    }

    // Filling the header
    track_of_robots_msg_.header.stamp = ros::Time::now();
    track_of_robots_msg_.header.seq += 1;

    //Publishing the message
    multirobot_track_pub_.publish(track_of_robots_msg_);
}


void Tracker::updateWorkingSpeed()
{

    if (new_robot_found_ || a_robot_died_)
    {
        // If a change ocurs, the node spins at the maximal speed
        current_periode_ = periode_start_;
    }
    else
    {
        // No changes, slowly relax the spin ratio
        current_periode_ += 0.05*(periode_relax_ - periode_start_);
        current_periode_ = std::min(current_periode_, periode_relax_);
    }

    // Reseting values to track changes
    new_robot_found_ = false;
    a_robot_died_    = false;
}

/* Definitions of the class RobotKnown */

RobotKnown::RobotKnown(const std::string robot_name):
    robot_name_(robot_name),
    remining_life_periodes_(3)
{
}

bool RobotKnown::isMyName(const std::string robot_name){
    if (robot_name_.compare(robot_name) == 0)
    {
        return true;
    }

    return false;
}

const std::string RobotKnown::getName(){
    return robot_name_;
}


void RobotKnown::increaseLife(){
    if (remining_life_periodes_ < 3)
    {
        ++remining_life_periodes_;
    }
}

void RobotKnown::decreaseLife(){
    if (remining_life_periodes_ > 0)
    {
        --remining_life_periodes_;
    }
}

bool RobotKnown::stillAlive()
{
    return (remining_life_periodes_ > 0);
}
