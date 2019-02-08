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
    PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/shoe_planslongest.txt");
    pt.recomputeMaxs(Assignment({{6, 1}, {2, 3}}));
}
