//
// Created by gerard on 17/02/19.
//

#ifndef IROS2019_SUGGESTERNODE_H
#define IROS2019_SUGGESTERNODE_H

#include "ros/ros.h"
#include "rosplan_knowledge_msgs/GetEnumerableTypeService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "std_srvs/Empty.h"
#include <regex>
#include <fstream>
#define RESTART_KB_TRIALS 15

class SuggesterNode {
private:
    ros::NodeHandle _nh;

    ros::ServiceClient _update_kb;
    ros::ServiceClient _gen_problem;
    ros::ServiceClient _planning_server;
    ros::ServiceClient _parse_plan;
    ros::Subscriber _plan_subs;
    std::vector<std::string> _plan;
    std::string _out_file;
    std::string _planner_output_file;
    int gen_plans_since_restart;


    void setKBValues(const std::vector<int>& assignments);
    void planCb(rosplan_dispatch_msgs::CompletePlanConstPtr plan);

    double planOnce(const std::vector<int>& assignments);
public:
    SuggesterNode(ros::NodeHandle& nh);
    ~SuggesterNode() = default;
    void runExperiments();
};


#endif //IROS2019_SUGGESTERNODE_H
