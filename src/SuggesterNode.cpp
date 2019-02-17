//
// Created by gerard on 17/02/19.
//

#include <SuggesterNode.h>

#include "SuggesterNode.h"


SuggesterNode::SuggesterNode(ros::NodeHandle &nh) : _nh(nh) {
     _update_kb = _nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");

    _gen_problem = _nh.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    _planning_server = _nh.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    _parse_plan = _nh.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");

    _plan_subs = _nh.subscribe("/rosplan_parsing_interface/complete_plan", 1, &SuggesterNode::planCb, this);

    if (!nh.getParam("/planspace_file", _out_file)) {
        ROS_ERROR("/planspace_file parameter not set!");
        ros::shutdown();
    }
    ROS_INFO_STREAM("Writing generated plans to " << _out_file);
    if (!nh.getParam("/rosplan_planner_interface/data_path", _planner_output_file)) {
        ROS_ERROR("/rosplan_planner_interface/data_path parameter not set!");
        ros::shutdown();
    }
    _planner_output_file += "plan.pddl";
}

void SuggesterNode::setKBValues(const std::vector<int>& assignments) { // Todo: change assignment type and use dicts
    rosplan_knowledge_msgs::KnowledgeUpdateServiceArray kua;

    for (int i = 0; i < assignments.size(); ++i) {
        rosplan_knowledge_msgs::KnowledgeItem ki;
        ki.knowledge_type = ki.FUNCTION;
        //TODO ki.attribute_name = _pref_types[i].first;
        //TODO ki.instance_type = _pref_types[i].second;
        ki.function_value = assignments[i];
        ki.is_negative = false;

        kua.request.update_type.push_back(kua.request.ADD_KNOWLEDGE);
        kua.request.knowledge.push_back(ki);
    }
    if (!_update_kb.call(kua)) {
        ROS_ERROR("(PlanSpaceGenerator) Failed to call service to update kb");
        ros::shutdown();
    }
}

void SuggesterNode::planCb(rosplan_dispatch_msgs::CompletePlanConstPtr plan) {
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

double SuggesterNode::planOnce(const std::vector<int>& assignments) {
    // Set all the KB once before planning
    setKBValues(assignments);

    int nongoal_attempts = 0;
    while (nongoal_attempts < 10) {
        std_srvs::Empty empty;
        if (!_gen_problem.call(empty)) {
            ROS_ERROR("(PlanSpaceGenerator) Failed to call service problem generation");
            ros::shutdown();
        } else if (!_planning_server.call(empty)) {
            ROS_ERROR("(PlanSpaceGenerator) Failed to call service planning server");
            ros::shutdown();
        } else if (!_parse_plan.call(empty)) {
            ROS_ERROR("(PlanSpaceGenerator) Failed to call service parse plan");
            ros::shutdown();
        }

        // WAIT FOR PLAN
        int tries = 0;
        while (_plan.empty()) {
            ros::spinOnce();
            ros::Duration(0.15).sleep();
            ++tries;
            if (tries > 50) {
                --nongoal_attempts;
                ROS_WARN("TIMEOUT WAITING FOR PLAN!!");
                break;
            }
        }

        // Check if goal
        bool found_goal = false;
        for (auto it = _plan.begin(); it != _plan.end(); ++it) {
            if (*it == "goal") {
                found_goal = true;
                break;
            }
        }
        if (not found_goal) {
            ROS_WARN("Plan didn't reach the goal!");
            ++nongoal_attempts;
        }
    }
    if (nongoal_attempts == 10) {
        ROS_WARN("TOO MANY ATTEMPTS WITHOUT GOAL!");
        exit(1);
    }

    // Get reward
    // get reward: i.e. "sed -e \"s/.*Round reward: \(.*\)$/\1/;t;d\" + _planner_output_file"
    std::ifstream planfile;
    planfile.open(_planner_output_file);
    std::string line, reward;
    std::regex e(".*Round reward: (.*)$");
    std::smatch m;
    while (std::getline(planfile, line)) {
        std::regex_search(line, m, e);
        if (!m.empty()) {
            reward = m[1]; // Get first matched subgroup
            break;
        }
    }


    ++gen_plans_since_restart;

    if (gen_plans_since_restart > RESTART_KB_TRIALS) {
        // Kill KB and wait for restart
        system("rosnode kill /rosplan_knowledge_base");
        ros::service::waitForService("/rosplan_knowledge_base/update", -1);
        gen_plans_since_restart = 0;
    }
    return std::stod(reward);
}

/*    // WRITE TO FILE
    std::ofstream outfile;
    _plan.clear();
    outfile.close();*/

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "planspace_suggester");
    ros::NodeHandle n;
    SuggesterNode sn(n);

}
