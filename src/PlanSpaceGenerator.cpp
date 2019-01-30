//
// Created by gcanal on 30/01/19.
//
#include <PlanSpaceGenerator.h>

#include "PlanSpaceGenerator.h"

PlanSpaceGenerator::PlanSpaceGenerator(ros::NodeHandle &nh) : _nh(nh){
    // Subscribe to needed services... TODO



    // SHOE
    _pref_types = {
            {"p_motor_rightleg", "t_threelevel"},
            {"p_motor_leftleg", "t_threelevel"},
            {"p_motor_rightfoot", "t_threelevel"},
            {"p_motor_leftfoot", "t_threelevel"},
            {"p_speed", "t_threelevel"},
            {"p_force", "t_threelevel"},
            {"p_information_providing", "t_infoprov"},
            {"p_petitions", "t_petitions"}
    };

    std::string srv_name = "/rosplan_knowledge_base/domain/enumerable_type";
    ros::ServiceClient getEnumerableType = _nh.serviceClient<rosplan_knowledge_msgs::GetEnumerableTypeService>(srv_name);
    for (auto it = _pref_types.begin(); it != _pref_types.end(); ++it) {
        if (_type_values.find(it->second) == _type_values.end()) {
            rosplan_knowledge_msgs::GetEnumerableTypeService params;
            params.request.type_name = it->second;
            if (!getEnumerableType.call(params)) {
                ROS_ERROR("(PlanSpaceGenerator) Failed to call service %s", srv_name.c_str());
            }
            _type_values[it->second] = params.response.values;
        }
    }


    _update_kb = _nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update");

    _gen_problem = _nh.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    _planning_server = _nh.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    _parse_plan = _nh.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");

    _plan_subs = _nh.subscribe("/rosplan_parsing_interface/complete_plan", 1, &PlanSpaceGenerator::planCb, this);
}

void PlanSpaceGenerator::generatePlans(int i) {
    if (i == _pref_types.size()) { // END
        std_srvs::Empty empty;
        if (!_gen_problem.call(empty)) {
            ROS_ERROR("(PlanSpaceGenerator) Failed to call service problem generation");
            ros::shutdown();
        }
        else if (!_planning_server.call(empty)) {
            ROS_ERROR("(PlanSpaceGenerator) Failed to call service planning server");
            ros::shutdown();
        }
        else if (!_parse_plan.call(empty)) {
            ROS_ERROR("(PlanSpaceGenerator) Failed to call service parse plan");
            ros::shutdown();
        }

        // GET PLAN
        while (_plan.empty()) {
            ros::spinOnce();
            ros::Duration(0.15).sleep();
        }
        ROS_INFO("Created plan...");

        // TODO WRITE TO FILE
    }

    // List of all preferences
    // For all preferences of the type
    for (int v = 0; v < _type_values[_pref_types[i].second].size(); ++v) {
        // For all values of the preference
        // Set KB
        setKBValue(_pref_types[i].first, _pref_types[i].second, v, true);
        // Recursive call
        generatePlans(i+1);
        // Unset KB
        setKBValue(_pref_types[i].first, _pref_types[i].second, v, false);
    }

}

void PlanSpaceGenerator::setKBValue(const std::string& attname, const std::string& type, int value, bool add) {
    rosplan_knowledge_msgs::KnowledgeItem ki;
    ki.attribute_name = attname;
    ki.instance_type = type;
    ki.function_value = value;
    ki.is_negative = add;

    rosplan_knowledge_msgs::KnowledgeUpdateService ku;
    ku.request.knowledge = ki;
    if (add) ku.request.update_type = ku.request.ADD_KNOWLEDGE;
    else ku.request.update_type = ku.request.REMOVE_KNOWLEDGE;
}

void PlanSpaceGenerator::planCb(rosplan_dispatch_msgs::CompletePlanConstPtr plan) {
    for (auto it = plan->plan.begin(); it != plan->plan.end(); ++it) {
        std::string params = "";
        for (auto pit = it->parameters.begin(); pit != it->parameters.end(); ++pit) {
            if (not params.empty()) params += ", ";
            else params += "(";
            params += pit->value;
        }
        if (not params.empty()) params += ")";
        _plan.push_back(it->name + params);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "planspace_generator");
    ros::NodeHandle n;
    PlanSpaceGenerator psg(n);
    return 0;
}

