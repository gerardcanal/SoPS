//
// Created by gcanal on 05/02/19.
//

#include <PlanSpaceSuggesterNode.h>


PlanSpaceSuggesterNode::PlanSpaceSuggesterNode(const std::string &planspace_path) {
    this->planspace_path = planspace_path;
}


std::vector<size_t> PlanSpaceSuggesterNode::getMaxRChildren(NodePtr n) {
    std::vector<size_t> idxs;
    double maxr = -std::numeric_limits<double>::max();
    for (auto it = n->children.begin(); it != n->children.end(); ++it) {
        double it_r = it->second.reward[it->second.max_reward_idx];
        if (it_r > maxr) {
            idxs.clear();
            idxs.push_back(it->first);
            maxr = it->second.reward[it->second.max_reward_idx];
        }
        else if (it_r == maxr) idxs.push_back(it->first);
    }
    return idxs;
}

void PlanSpaceSuggesterNode::suggestNew(NodePtr n) {
    //State suggestions(StateDict::numPredicates());

    auto mc = getMaxRChildren(n);
    for (auto it: n->children) {
        suggestNew(it.second.child);
    }

}


int main(int argc, char* argv[]) {
    std::cout << "Hello" << std::endl;
    //StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/shoe_types.txt");
    StateDict::loadPredicates("/home/gerard/code/catkin_ws/src/iros2019/shoe_types.txt");
    //PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/shoe_planslongest.txt");
    //PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    PlanTree pt("/home/gerard/code/catkin_ws/src/iros2019/shoe_plans.txt");

    PlanSpaceSuggesterNode n("");
    n.suggestNew(pt.getRoot());
    exit(0);


    // Tests equal
    std::cout << "Equal to itself: " << pt.isEqual(pt) << std::endl;
    //PlanTree pt2("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    PlanTree pt2("/home/gerard/code/catkin_ws/src/iros2019/shoe_plans.txt");
    std::cout << "Equal to (reloaded) itself: " << pt.isEqual(pt2) << std::endl;

    pt.recomputeMaxs(Assignment({{6, 1}, {2, 3}}));
    pt.recomputeMaxs(Assignment());
    std::cout << "Equal to (reloaded) itself after recomputeMaxs: " << pt.isEqual(pt2) << std::endl;


    pt.recomputeMaxs(Assignment({{6, 2}, {1, 1}}));
    pt2.recomputeMaxs(Assignment({{6, 2}, {1, 1}}));
    std::cout << "Equal to (reloaded) itself after changing assignment: " << pt.isEqual(pt2) << std::endl;

}
