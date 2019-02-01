//
// Created by gcanal on 30/01/19.
//
#include <PlanSpaceGenerator.h>

#include "PlanSpaceGenerator.h"

PlanSpaceGenerator::PlanSpaceGenerator(ros::NodeHandle &nh) : _nh(nh){

    // SHOE TODO: get from launchfile?
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

    gen_plans_since_restart = 0;
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


    _update_kb = _nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");

    _gen_problem = _nh.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    _planning_server = _nh.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    _parse_plan = _nh.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");

    _plan_subs = _nh.subscribe("/rosplan_parsing_interface/complete_plan", 1, &PlanSpaceGenerator::planCb, this);

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

#define RESTART_KB_TRIALS 15
void PlanSpaceGenerator::generatePlans(int i, std::vector<int>& assignments) {
    if (i == _pref_types.size()) { // END
        // Set all the KB once before planning
        setKBValues(assignments);
        int nongoal_attempts = 0;
        for (int k = 0; k < 3; ++k) {
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
            while (_plan.empty()) {
                ros::spinOnce();
                ros::Duration(0.15).sleep();
            }
            //ROS_INFO_STREAM("Created plan for preference...");
            // Check if goal
            bool found_goal = false;
            for (auto it = _plan.begin(); it != _plan.end(); ++it) {
                if (*it == "goal") {
                    found_goal = true;
                    break;
                }
            }
            if (not found_goal) {
                ROS_WARN("Plan didn't reach the goal with preferences: ");
                for (int i = 0; i < assignments.size(); ++i) {
                    ROS_WARN_STREAM("   " << i << ": " << _pref_types[i].first << " = " <<
                                          _type_values[_pref_types[i].second][assignments[i]] << " (" << assignments[i]
                                          << ")");
                }
                --i;
                ++nongoal_attempts;
                if (nongoal_attempts < 3) continue;
                nongoal_attempts = 0;
                //exit(-1);
                //return;
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

            // WRITE TO FILE
            std::ofstream outfile;
            outfile.open(_out_file, std::ios::out | std::ios::app);
            /*for (int i = 0; i < assignments.size(); ++i) {
                if (i > 0) outfile << ", ";
                outfile << assignments[i];
            }
            outfile << " | ";*/
            for (int i = 0; i < assignments.size(); ++i) {
                if (i > 0) outfile << ", ";
                outfile << _pref_types[i].first << "=" << _type_values[_pref_types[i].second][assignments[i]];
            }
            outfile << " | ";
            for (int i = 0; i < _plan.size(); ++i) {
                if (i > 0) outfile << "; ";
                outfile << _plan[i];
            }
            outfile << " | ";
            outfile << reward << std::endl;
            _plan.clear();
            outfile.close();
            ++gen_plans_since_restart;
        }
        if (gen_plans_since_restart > RESTART_KB_TRIALS) {
            // Kill KB and wait for restart
            system("rosnode kill /rosplan_knowledge_base");
            ros::service::waitForService("/rosplan_knowledge_base/update", -1);
            gen_plans_since_restart = 0;
        }
        ros::Duration(0.5).sleep();
        return;
    }

    // List of all preferences
    // For all preferences of the type
    int n = _type_values[_pref_types[i].second].size();
    for (int v = (assignments[i])%n; v < n; ++v) {
        // For all values of the preference
        assignments[i] = v;
        // Set KB
        //setKBValues(assignments);
        // Recursive call
        generatePlans(i+1, assignments);
        // Unset KB
        assignments[i] = 0;
        //not needed setKBValue(_pref_types[i].first, _pref_types[i].second, v, false);
    }

}

void PlanSpaceGenerator::setKBValues(const std::vector<int>& assignments) {
    rosplan_knowledge_msgs::KnowledgeUpdateServiceArray kua;

    for (int i = 0; i < assignments.size(); ++i) {
        rosplan_knowledge_msgs::KnowledgeItem ki;
        ki.knowledge_type = ki.FUNCTION;
        ki.attribute_name = _pref_types[i].first;
        ki.instance_type = _pref_types[i].second;
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

void PlanSpaceGenerator::generatePlans() {
    std::vector<int> assign(_pref_types.size(), 0);
    //assign = getAssignIndex();
    generatePlans(0, assign);
}

std::vector<int> PlanSpaceGenerator::getAssignIndex() {
    std::vector<std::pair<std::string, std::string>> init =
            {{"p_motor_rightleg", "@tl_unknown"},
            {"p_motor_leftleg", "@tl_unknown"},
            {"p_motor_rightfoot", "@tl_unknown"},
            {"p_motor_leftfoot", "@med"},
            {"p_speed", "@high"},
            {"p_force", "@med"},
            {"p_information_providing", "@ip_never"},
            {"p_petitions", "@p_never"}};

    std::vector<int> assign(init.size());
    for (int i = 0; i < init.size(); ++i) {
        std::string t;
        for (int k = 0; k < _pref_types.size(); ++k) {
            if (_pref_types[k].first == init[i].first) {
                t = _pref_types[k].second;
                break;
            };
        }
        for (int j = 0; j < _type_values[t].size();++j) {
            if (_type_values[t][j] == init[i].second) {
                assign[i] = j;
                break;
            }
        }
    }
    return assign;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "planspace_generator");
    ros::NodeHandle n;
    PlanSpaceGenerator psg(n);
    psg.generatePlans();
    return 0;
}

