//
// Created by gerard on 17/02/19.
//

#include <SuggesterNode.h>

#include "SuggesterNode.h"


SuggesterNode::SuggesterNode(ros::NodeHandle &nh, bool allow_changes) : _nh(nh) {
     _update_kb = _nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");

    _gen_problem = _nh.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
    _planning_server = _nh.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
    _parse_plan = _nh.serviceClient<std_srvs::Empty>("/rosplan_parsing_interface/parse_plan");

    _plan_subs = _nh.subscribe("/rosplan_parsing_interface/complete_plan", 1, &SuggesterNode::planCb, this);

    if (!nh.getParam("/experiments_results_file", _out_file)) {
        ROS_ERROR("/experiments_results_file parameter not set!");
        ros::shutdown();
        exit(1);
    }
    ROS_INFO_STREAM("Writing results to " << _out_file);
    if (!allow_changes) { // Planner not needed in changes experiment
        if (!nh.getParam("/rosplan_planner_interface/data_path", _planner_output_file)) {
            ROS_ERROR("/rosplan_planner_interface/data_path parameter not set!");
            ros::shutdown();
            exit(1);
        }
        _planner_output_file += "plan.pddl";
    }
}

void SuggesterNode::setKBValues(const Assignment& assignments) {
    rosplan_knowledge_msgs::KnowledgeUpdateServiceArray kua;

    for (int i = 0; i < assignments.size(); ++i) {
        rosplan_knowledge_msgs::KnowledgeItem ki;
        ki.knowledge_type = ki.FUNCTION;
        ki.attribute_name = StateDict::getPredName(assignments[i].first);//_pref_types[i].first;
        ki.instance_type = StateDict::getPredType(assignments[i].first);//_pref_types[i].second;
        ki.function_value = assignments[i].second;
        ki.is_negative = false;

        kua.request.update_type.push_back(kua.request.ADD_KNOWLEDGE);
        kua.request.knowledge.push_back(ki);
    }
    int ntrials = 0;
    const int MAX_TRIALS=10;
    while (ntrials < MAX_TRIALS) {
        if (!_update_kb.call(kua)) {
            ROS_ERROR("(PlanSpaceGenerator) Failed to call service to update kb");
        }
        else break;
        ++ntrials;
        ros::Duration(1.0).sleep();
    }
    if (ntrials >= MAX_TRIALS) {
        ROS_ERROR("(PlanSpaceGenerator) Failed too many times to call service to update kb!");
        exit(1);
    }
}

void SuggesterNode::planCb(rosplan_dispatch_msgs::CompletePlanConstPtr plan) {
    _plan.clear();
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

double SuggesterNode::planOnce(const Assignment& assignments, bool& found_reward) {
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
        else break; // Break te nongoal_attempts loop
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
    found_reward = false;
    while (std::getline(planfile, line)) {
        std::regex_search(line, m, e);
        if (!m.empty()) {
            reward = m[1]; // Get first matched subgroup
            found_reward = true;
            break;
        }
    }

    /*++gen_plans_since_restart;
    if (gen_plans_since_restart > RESTART_KB_TRIALS)*/ restartKB();
    if (not found_reward) {
        ROS_ERROR_STREAM("Failed to parse reward!");
        return 0;
    }
    return std::stod(reward);
}


void SuggesterNode::runExperiment(const Assignment &assignments, const std::string &exp_name, int trials) {
    // Run 10 tasks of each
    for (int i = 0; i < trials; ++i) {
        bool found_reward = false;
        double reward = planOnce(assignments, found_reward);
        if (not found_reward) {
            --i; // repeat iteration
            continue;
        }

        std::ofstream outfile;
        outfile.open(_out_file, std::ios::out | std::ios::app);
        // TYPE, REWARD, NCHANGES/SUGGESTIONS,
        outfile << exp_name << ", " << reward << ", " << assignments.size() << ",";

        for (auto it: assignments) {
            outfile << " " << StateDict::getPredName(it.first) << "=" << StateDict::getPredValue(StateDict::getPredName(it.first), it.second);
        }
        outfile << ",";
        for (auto it: assignments) {
            outfile << " " << it.first << "=" << it.second;
        }
        outfile << std::endl;
        outfile.close();
    }
}

void SuggesterNode::runAdditionExperiments(const std::string &planspace_path, bool changes) {
    if (not changes) {
        // FIRST/BASELINE: no preference addition
        std::cout << "Baseline experiment..." << std::endl;
        runExperiment(Assignment(), "BASELINE", N_PLANNER_TRIALS_NONRANDOM);

        // WITH N suggestions
        for (size_t k = 1; k <= StateDict::numPredicates(); ++k) {
            std::cout << "Experiments for predicates..." << std::endl;
            // Get assignments!
            Assignment new_assgns;
            PlanTree pt(planspace_path);
            pss.getMinSuggestions(pt, new_assgns, k);
            new_assgns.erase(new_assgns.begin() + k, new_assgns.end());
            runExperiment(new_assgns, "SUGGESTIONS-" + std::to_string(k), N_PLANNER_TRIALS_NONRANDOM);
        }
    }

    // RANDOM SUGGESTIONS
    /* initialize random seed: */
    srand (time(NULL));
    for (size_t k = 1; k <= StateDict::numPredicates(); ++k) {
        //if (k != 4 and k < 7) continue; //FIXME remove
        int r = 0;
        //if (k == 2) r = 49; //FIXME remove
        for (; r < N_RANDOM_EXPS; ++r) {
            std::cout << "\nRandom experiment " << r+1 << " with " << k << " random predicates..." << std::endl;
            Assignment rnd_assgns = generateRandomAssigs(k);
            runExperiment(rnd_assgns, "RANDOM-"+std::to_string(k));

            for (size_t k1 = 1; k1 <= (StateDict::numPredicates()-k); ++k1) {
                std::cout << "\nRandom experiment " << r+1 << " with " << k << " random predicates and " << k1 << " suggestions of predicates..." << std::endl;
                Assignment new_assigns = rnd_assgns;
                PlanTree pt(planspace_path);
                pss.getMinSuggestions(pt, new_assigns, k1, changes, 1);
                if (k+k1 < new_assigns.size()) new_assigns.erase(new_assigns.begin() + k+k1, new_assigns.end());
                runExperiment(new_assigns, "RAND-"+std::to_string(k)+"+SUGG-" + std::to_string(k1), N_PLANNER_TRIALS_RDM);
            }
        }
    }
}

void SuggesterNode::runChangesExperiments(const std::string &planspace_path) {
    // no need to replan!
    std::ofstream outfile;
    outfile.open(_out_file, std::ios::out | std::ios::app);
    srand(time(NULL)); // Initialize random seed

    for (size_t k = 0; k <= StateDict::numPredicates(); ++k) {
        ROS_INFO_STREAM("Changes experiment " << k << " random predicates..");
        for (int r = 0; r < N_RANDOM_CHANGE_EXPS; ++r) {
            if (r%100 == 0) ROS_INFO_STREAM("  Random trial number " << r << "...");
            Assignment rnd = generateRandomAssigs(k);

            // No changes
            {
                Assignment cpy = rnd;
                PlanTree pt(planspace_path); // Force reload tree
                auto sugg = pss.getMinSuggestions(pt, cpy, 1, false);
                auto counts = countAdditionsChanges(sugg);
                // get reward
                auto maxr_children = pss.getMaxRChildren(pt.getRoot());
                assert(maxr_children.size() > 0);
                double maxr = pt.getRoot()->children[maxr_children[0]].reward[pt.getRoot()->children[maxr_children[0]].max_reward_idx];
                outfile << "PSS, " << k << ", " << maxr << ", " << counts.first << ", " << counts.second << std::endl;
            }

            // With changes
            {
                Assignment cpy = rnd;
                PlanTree pt(planspace_path);
                auto sugg = pss.getMinSuggestions(pt, cpy, 1, true, 1);
                auto counts = countAdditionsChanges(sugg);
                // get reward
                auto maxr_children = pss.getMaxRChildren(pt.getRoot());
                assert(maxr_children.size() > 0);
                double maxr = pt.getRoot()->children[maxr_children[0]].reward[pt.getRoot()->children[maxr_children[0]].max_reward_idx];
                outfile << "PSS-CHANGE, " << k << ", " << maxr << ", " << counts.first << ", " << counts.second << std::endl;
            }
        }
    }
    outfile.close();
}

void SuggesterNode::restartKB() {
    // Kill KB and wait for restart
    system("rosnode kill /rosplan_knowledge_base &> /dev/null");
    ros::service::waitForService("/rosplan_knowledge_base/update", -1);
    gen_plans_since_restart = 0;
    ros::Duration(0.5).sleep(); // Just in case...
}

Assignment SuggesterNode::generateRandomAssigs(int k) {
    Assignment rnd_assgns;
    std::set<size_t> used;
    for (int as = 0; as < k; ++as) {
        size_t pred = rand()%StateDict::numPredicates();
        while (used.count(pred) > 0) pred = rand()%StateDict::numPredicates();
        used.insert(pred);
        int val = (rand()%(StateDict::numValues(StateDict::getPredName(pred))-1))+1; // -1 to remove the unknown, +1 to move from the 0
        rnd_assgns.push_back(std::make_pair(pred, val));
    }
    return rnd_assgns;
}

std::pair<int, int> SuggesterNode::countAdditionsChanges(const std::vector<Suggestion> &v) {
    std::pair<int, int> p(0, 0);
    for (auto it : v) {
        p.first += it.nadditions;
        p.second += it.nchanges;
    }
    return p;
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "planspace_suggester");
    ros::NodeHandle n;

    bool allow_changes = false;
    n.getParam("/allow_changes", allow_changes);
    SuggesterNode sn(n, allow_changes);

    // Load data
    //StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/jacket_types.txt");
    //StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/shoe_types.txt");
    //StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/feeding_types.txt");
    //StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/superfluous_feeding_types.txt");
    StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/superfluous_jacket_types.txt");
    //StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/superfluous_shoe_types.txt");

    //PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    //PlanTree pt("/home/gerard/code/catkin_ws/src/iros2019/shoe_plans.txt");
    //std::cout << "Tree size: " << pt.size() << std::endl;

    // RUN!
    std::string planspace_path;
    if (!n.getParam("/planspace_file", planspace_path)) {
        ROS_ERROR_STREAM("Parameter /planspace_file not set!");
        ros::shutdown();
        exit(1);
    }

    if (allow_changes) sn.runChangesExperiments(planspace_path);
    else sn.runAdditionExperiments(planspace_path);
}
