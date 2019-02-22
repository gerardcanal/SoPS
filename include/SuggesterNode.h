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
#include "PlanSpaceSuggester.h"
#include <cstdlib> // rand
#include <regex>
#include <fstream>
#define RESTART_KB_TRIALS 15
#define N_RANDOM_EXPS 50
#define N_PLANNER_TRIALS_RDM 20
#define N_PLANNER_TRIALS_NONRANDOM 50

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
    PlanSpaceSuggester pss;


    void setKBValues(const Assignment& assignments);
    void planCb(rosplan_dispatch_msgs::CompletePlanConstPtr plan);
    void restartKB();

    double planOnce(const Assignment& assignments);
    void runExperiment(const Assignment &assignments, const std::string &exp_name, int trials=N_PLANNER_TRIALS_RDM);
public:
    SuggesterNode(ros::NodeHandle& nh);
    ~SuggesterNode() = default;
    void runExperiments(const std::string& planspace_path);
};


#endif //IROS2019_SUGGESTERNODE_H
