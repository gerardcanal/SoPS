//
// Created by gcanal on 05/02/19.
//

#include <PlanSpaceSuggesterNode.h>


PlanSpaceSuggesterNode::PlanSpaceSuggesterNode(const std::string &planspace_path) {
    this->planspace_path = planspace_path;
}

// Returns ids of children with max reward
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

void PlanSpaceSuggesterNode::join(std::vector<DiffResults> &a,
                                  std::vector<DiffResults> &b) {
    a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
}


std::vector<DiffResults> PlanSpaceSuggesterNode::getMaxChildDiffs(NodePtr n) {
    //State suggestions(StateDict::numPredicates());

    std::vector<DiffResults> ret;
    auto mc = getMaxRChildren(n);
    for (auto c_id: mc) {
        // Compute Diff set
        NodeInfo c = n->children[c_id];
        DiffResults d;  // diff matrix of child c_id with all the others
        for (auto c_it = n->children.begin(); c_it != n->children.end(); ++c_it) {
            if (c_it->first == c_id) continue;
            bState diff = StateDict::diff(StateDict::getState(c.state[c.max_reward_idx]),
                                          StateDict::getState(c_it->second.state[c_it->second.max_reward_idx]));
            d.S.push_back(diff);
        }

        // Compute metric
        d.node = std::make_shared<NodeInfo>(c);
        d.metric = computeNodeMetric(c_id, n, d.S);
        ret.push_back(d);

        auto childdiffs = getMaxChildDiffs(c.child);
        join(ret, childdiffs);
    }
    return ret;
}

DiffResults& PlanSpaceSuggesterNode::suggestChanges(PlanTree pt) {
    std::vector<DiffResults> allchildDiffs = getMaxChildDiffs(pt.getRoot());
    assert(allchildDiffs.size() > 0);
    size_t max_m = 0;
    for (size_t i = 1 ; i < allchildDiffs.size(); ++i) {
        if (allchildDiffs[i].metric > allchildDiffs[max_m].metric) max_m = i;
    }
    return allchildDiffs[max_m];
}


// c_id is the reference child id
double PlanSpaceSuggesterNode::computeNodeMetric(size_t c_id, NodePtr n, const std::vector<bState> &v, int strategy) {
    double rsum = 0;
    for (auto c_it = n->children.begin(); c_it != n->children.end(); ++c_it) {
        if (c_it->first == c_id) continue;
        rsum += c_it->second.reward[c_it->second.max_reward_idx];
    }
    // ret is now the sum of all the elements but c_id
    double xreward = n->children[c_id].reward[n->children[c_id].max_reward_idx];
    int nchilds = (n->children.size()-1); // -1 because we're not counting c_id!
    if (nchilds == 0) return 0;
    if (strategy == M_SUMDIFFS) {
        // SUMDIFFS is sum of the differences: x-y1 + x-y2 + ... = nx-SUM(y)
        //return ((nchilds*xreward) - rsum)/nchilds;
        return (xreward - (rsum/nchilds));// optimized version
    }
    ROS_ERROR_STREAM("[computeNodeMetric] Unknown metric " << strategy << std::endl);
    return rsum;
}

double PlanSpaceSuggesterNode::computeNodeSuggestion(const std::vector<bState> &d, NodeInfoPtr ni) {
    // TODO
    return 0;
}



int main(int argc, char* argv[]) {
    std::cout << "Hello" << std::endl;
    StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/shoe_types.txt");
    //StateDict::loadPredicates("/home/gerard/code/catkin_ws/src/iros2019/shoe_types.txt");
    //PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/shoe_planslongest.txt");
    PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    std::cout << "Tree size: " << pt.size() << std::endl;
    //PlanTree pt("/home/gerard/code/catkin_ws/src/iros2019/shoe_plans.txt");

    PlanSpaceSuggesterNode n("");
    n.suggestChanges(pt);
    exit(0);


    // Tests equal
    std::cout << "Equal to itself: " << pt.isEqual(pt) << std::endl;
    PlanTree pt2("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    //PlanTree pt2("/home/gerard/code/catkin_ws/src/iros2019/shoe_plans.txt");
    std::cout << "Equal to (reloaded) itself: " << pt.isEqual(pt2) << std::endl;

    pt.recomputeMaxs(Assignment({{6, 1}, {2, 3}}));
    pt.recomputeMaxs(Assignment());
    std::cout << "Equal to (reloaded) itself after recomputeMaxs: " << pt.isEqual(pt2) << std::endl;


    pt.recomputeMaxs(Assignment({{6, 2}, {1, 1}}));
    pt2.recomputeMaxs(Assignment({{6, 2}, {1, 1}}));
    std::cout << "Equal to (reloaded) itself after changing assignment: " << pt.isEqual(pt2) << std::endl;

}
