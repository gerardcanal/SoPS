//
// Created by gcanal on 30/01/19.
//

#ifndef IROS2019_PLANSPACEGENERATOR_H
#define IROS2019_PLANSPACEGENERATOR_H

#include "ros/ros.h"
#include "rosplan_knowledge_msgs/GetEnumerableTypeService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "std_srvs/Empty.h"
#include <fstream>
#include <regex>

class PlanSpaceGenerator {
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

    std::map<std::string, std::vector<std::string>> _type_values; // Map preference type -> values
    std::vector<std::pair<std::string, std::string>> _pref_types; // List of preferences and its type

    void setKBValues(const std::vector<int>& assignments);
    void planCb(rosplan_dispatch_msgs::CompletePlanConstPtr plan);
    void generatePlans(int i, std::vector<int>& assignments);
    std::vector<int> getAssignIndex();
public:
    PlanSpaceGenerator(ros::NodeHandle& nh);
    ~PlanSpaceGenerator() = default;
    void generatePlans();

};


#endif //IROS2019_PLANSPACEGENERATOR_H
