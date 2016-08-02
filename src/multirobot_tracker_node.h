 /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Duisburg-Essen (UDE).
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of Duisburg-Essen nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Eduardo Ferrera */

/**

    @mainpage

    ///////////////////////////@htmlinclude manifest.html

    @b multirobot_tracker is a simple node ment to be executed in
    multirobot enviroments. It keeps track of the current namespaces
    of each robot, and publishing a list, that can be used to subcribe
    to all of them at the same time.

    <hr>

    @section usage Usage
    @verbatim
    $ multirobot_tracker
    @endverbatim

    <hr>

    @section topic ROS topics

    Publishes to (name/type):
    -@b "multirobot_track" multirobot_tracker/multiStringStamped: string vector with all robots namespaces matching the convention.

    <hr>

    @section parameters ROS parameters

    - "~periode_relax" (double) : Time [sec] between two consecutive searches when the node is relaxed, min: 0.1, default: 5.0
    - "~periode_start" (double) : Time [sec] between two consecutive searches at the start of the node, min: 0.1, default: 0.2
    - "~topics2seach"  (string) : Reference topics that every robot should be publishing, separed by commas, default: "base_scan,odom"

**/

#ifndef MULTIROBOT_TRACKER_NODE_H_INCLUDED
#define MULTIROBOT_TRACKER_NODE_H_INCLUDED

//#define DEBUG 1

#include <boost/algorithm/string.hpp>

// roscpp
#include <ros/ros.h>

// message definitions
#include "multirobot_tracker/multiStringStamped.h"

namespace multirobot_tracker
{

/* Forward declaration of dependencies. */
class RobotKnown;

/**
 * The class Tracker is in charge of ask about existing robots to the ros::master.
 * It lists the knowing robots'namespaces in a "RobotKnown" list and shares the information through the topic "multirobot_tracks".
 * Implements a smart sleeper to control the speed of work of the node.
 */
class Tracker{
    public:
        // Constructor
        /**
         * @brief Initializes the Tracker node with maximal working speed and prepares the "track" publisher.
         */
        Tracker();

        /**
         * @brief Sleeps and performs callbacks in a smart way while tracking namespaces of robots
         *
         * Uses a while loop that starts seaching fast new robots but decreases it speed with the time.
         */
        void controlLoop();

    private:
        // ROS private variables
        ros::NodeHandle nh_;

        //Publishers
        ros::Publisher multirobot_track_pub_;

        // Configuration variables
        double periode_start_ ,periode_relax_;

        // Internal variables
        double current_periode_;
        std::vector <std::string> topic_references_;
        std::vector <RobotKnown>  robots_known_vec_;
        bool new_robot_found_;
        bool a_robot_died_;
        multirobot_tracker::multiStringStamped track_of_robots_msg_;

        //Private methods
        /**
         * @brief Extracts all the topic_references_ from str2split
         *
         * Asumes that there are one or many topic names, separated by commas on str2split.
         * Reads it and uses the commas to populates the vector topic_references_
         *
         * \param str2split string readed from the parameter topics2seach
         */
        void getTopicReferences(std::string str2split);


        /**
         * @brief Checks in the ros::master if exists new robots. Adds the discovered robots to the robots_known_vec_ list
         *
         * Request to the ros::master all published topics and compares them with the one of the "topics_references_" list.
         * If meets one of the "topics_references_" topics, extracts its global namespaces and compares it with the global
         * namespaces of the knowing robots.
         * If the namespace is new, adds the new namespace/robot to the robots_known_vec_ list.
         */
        void searchForRobots();

        /**
         * @brief Extracts from an input type "/global_namespace/topic_name" the "/global_namespace/". In case of no namespaces will return "/".
         *
         * @param full_topic_name: string containing a topic.
         * @return global_namespace of the rostopic input.
         */
        const std::string extractGlobalNamespace(const std::string full_topic_name);

        /**
         * @brief Checks for the existence of "robot_name" in the list of known robots. If is in the list, increases the robot life.
         *
         * @param robot_name Namespace of one robot.
         * @return (bool)onList. Returns true if the robot_name exist in the list of knowing robots.
         */
        bool isAKnownRobot(const std::string robot_name);

        /**
         * @brief Increases the ages of all known robots. Killing the ones that are too old.
         */
        void makeRobotsOlder();

        /**
         * @brief Shares the collected information through the rostopic "/multirobot_track".
         */
        void sendTrackOfRobots();

        /**
         * @brief Changes the value of current_periode_ according to the current changes
         *
         * If something changed on the last working periode, updates the current_periode_
         * to the maximal speed (periode_start_). If not, slowly changes to the value of
         * periode_relax_
         */
        void updateWorkingSpeed();
};  //class Tracker

/**
 * @brief The class RobotKnown is defined to work as a list of robot's namespaces of the knowing robots.
 *
 * Each robot has a life. This class allows to track such life too.
 */
class RobotKnown{
    public:
        //Constructor

        /**
         * @brief Initializes a node of the class RobotKnown with his name and three periodes of life
         *
         * @param robot_name names (namespace) asociated to a knowing robot.
         */
        RobotKnown(const std::string robot_name);

        //Public methods
        /**
         * @brief Checks if the robot_name coincides with the name of the knowing robot.
         *
         * @param robot_name to check with the robotKnown's name
         * @return Returns true if the name of the robotKnown's is equal to it.
         */
        bool isMyName(const std::string robot_name);


        /**
         * @brief Returns the name of the robot.
         *
         * @return Name (namespace) of the robot.
         */
        const std::string getName();

        /**
         * @brief Increses one periode of life of a for the known robot. Allow only a maximum of 3 periodes.
         */
        void increaseLife();

        /**
         * @brief Decreases one periode of life of a for the known robot.
         */
        void decreaseLife();

        /**
         * @brief Returns false if the robot's name has not been seen in a while
         */
        bool stillAlive();

    private:
        //Private variables
        std::string robot_name_;
        unsigned int remining_life_periodes_;
};  // class RobotKnown


} // namespace multirobot_tracker

#endif // MULTIROBOT_TRACKER_NODE_H_INCLUDED
