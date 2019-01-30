//
// Created by gcanal on 30/01/19.
//

#ifndef IROS2019_PLANSPACEGENERATOR_H
#define IROS2019_PLANSPACEGENERATOR_H

#include "ros/ros.h"
#include "rosplan_knowledge_msgs/GetEnumerableTypeService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "std_srvs/Empty.h"

class PlanSpaceGenerator {
private:
    ros::NodeHandle _nh;

    ros::ServiceClient _update_kb;
    ros::ServiceClient _gen_problem;
    ros::ServiceClient _planning_server;
    ros::ServiceClient _parse_plan;
    ros::Subscriber _plan_subs;
    std::vector<std::string> _plan;

    std::map<std::string, std::vector<std::string>> _type_values;
    std::vector<std::pair<std::string, std::string>> _pref_types; // List of preferences and its type

    void setKBValue(const std::string& attname, const std::string& type, int value, bool add);
    void planCb(rosplan_dispatch_msgs::CompletePlanConstPtr plan);
public:
    PlanSpaceGenerator(ros::NodeHandle& nh);
    ~PlanSpaceGenerator() = default;
    void generatePlans(int i);
};


#endif //IROS2019_PLANSPACEGENERATOR_H
