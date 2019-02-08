//
// Created by gcanal on 05/02/19.
//

#include <PlanSpaceSuggesterNode.h>


PlanSpaceSuggesterNode::PlanSpaceSuggesterNode(const std::string &planspace_path) {
    this->planspace_path = planspace_path;
}

void PlanSpaceSuggesterNode::suggestNew() {
    PlanTree pt(planspace_path);
    NodePtr n = pt.getRoot();
    State suggestions(StateDict::numPredicates());
}


int main(int argc, char* argv[]) {
    std::cout << "Hello" << std::endl;
    StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/shoe_types.txt");
    //PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/shoe_planslongest.txt");
    PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    //pt.recomputeMaxs(Assignment({{6, 1}, {2, 3}}));

    // Test equal
    std::cout << "Equal to itself: " << pt.isEqual(pt) << std::endl;
    PlanTree pt2("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    std::cout << "Equal to (reloaded) itself: " << pt.isEqual(pt2) << std::endl;

    pt.recomputeMaxs(Assignment({{6, 1}, {2, 3}}));
    pt.recomputeMaxs(Assignment());
    std::cout << "Equal to (reloaded) itself after recomputeMaxs: " << pt.isEqual(pt2) << std::endl;


    pt.recomputeMaxs(Assignment({{6, 2}, {1, 1}}));
    pt2.recomputeMaxs(Assignment({{6, 2}, {1, 1}}));
    std::cout << "Equal to (reloaded) itself after changing assignment: " << pt.isEqual(pt2) << std::endl;

}
