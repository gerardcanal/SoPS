//
// Created by gcanal on 05/02/19.
//

#include <PlanSpaceSuggester.h>
#include <cmath>

PlanSpaceSuggester::PlanSpaceSuggester() {
    //this->planspace_path = planspace_path;
}

// Returns ids of children with max reward
std::vector<size_t> PlanSpaceSuggester::getMaxRChildren(NodePtr n) {
    std::vector<size_t> idxs;
    double maxr = -std::numeric_limits<double>::max();
    for (auto it = n->children.begin(); it != n->children.end(); ++it) {
        if (it->second.max_reward_idx == -1) continue; // Pruned child!
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

void PlanSpaceSuggester::join(std::vector<DiffResults> &a,
                                  std::vector<DiffResults> &b) {
    a.insert(a.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
}


// Returns the diff matrices of the childs with max reward (recursive to whole tree starting at node n)
// Each diffresults includes the diff matrix, the associated metric and the node source of comparison.
// Mask: see SateDict::diff() GetPromisingNodes
std::vector<DiffResults> PlanSpaceSuggester::getMaxChildDiffs(NodePtr n, const bState& mask) {
    //State suggestions(StateDict::numPredicates());

    std::vector<DiffResults> ret;
    auto mc = getMaxRChildren(n);
    for (auto c_id: mc) {
        // Compute Diff set
        NodeInfo c = n->children[c_id];
        DiffResults d;  // diff matrix of child c_id with all the others
        for (auto c_it = n->children.begin(); c_it != n->children.end(); ++c_it) {
            if (c_it->first == c_id) continue;
            if (c.max_reward_idx == -1 or c_it->second.max_reward_idx == -1) continue;
            bState diff = StateDict::diff(StateDict::getState(c.state[c.max_reward_idx]),
                                          StateDict::getState(c_it->second.state[c_it->second.max_reward_idx]), mask);
            d.S.push_back(diff);
        }

        // Compute metric
        if (not d.S.empty() and sumMat(d.S) > 0) { // If it's empty it means there's only one valid child.
            d.node = std::make_shared<NodeInfo>(c);
            d.metric = computeNodeMetric(c_id, n, d.S);
            ret.push_back(d);
        }

        auto childdiffs = getMaxChildDiffs(c.child, mask);
        join(ret, childdiffs);
    }
    return ret;
}

// Assignment is the fixed values for the predicates
Suggestion PlanSpaceSuggester::suggestAdditions(PlanTree pt, const Assignment &assignment) {
    bState mask = StateDict::computeMask(assignment);
    std::vector<DiffResults> allchildDiffs = getMaxChildDiffs(pt.getRoot(), mask);
    assert(allchildDiffs.size() > 0);
    size_t max_m = 0;
    int equal = 0;
    double metric_sum = allchildDiffs[0].metric; // Sums all the metrics
    for (size_t i = 1 ; i < allchildDiffs.size(); ++i) {
        if (allchildDiffs[i].metric > allchildDiffs[max_m].metric) {
            max_m = i;
            equal = 0;
        }
        else if (allchildDiffs[i].metric == allchildDiffs[max_m].metric) ++equal;
        metric_sum += allchildDiffs[i].metric;
        //std::cout <<allchildDiffs[i].metric <<std::endl;
    }

    // Compute deviation
    double mean = metric_sum / allchildDiffs.size();
    double sigma = 0;
    for (size_t i = 0 ; i < allchildDiffs.size(); ++i) {
        double aux = allchildDiffs[i].metric-mean;
        sigma += aux*aux;
    }
    sigma /= (allchildDiffs.size()-1);

    //std::cout << equal << " " << allchildDiffs[max_m].metric << " " << std::sqrt(sigma) <<  std::endl;

    return computeNodeSuggestion(allchildDiffs[max_m].S, allchildDiffs[max_m].node);
}


Suggestion PlanSpaceSuggester::suggestWithChanges(PlanTree pt, int K, const Assignment &changeable, const Assignment &assignment) {
    Suggestion s = suggestAdditions(pt, assignment);
    for (size_t i = 0; i < s.assignments.size(); ++i) {
        // Check if it appears in the changeable list
        for (size_t j = 0; j < changeable.size(); ++j) {
            if ((changeable[j].first == s.assignments[i].first)) {
                if ((changeable[j].second != s.assignments[i].second) and (abs(changeable[j].second - s.assignments[i].second) < K)) {
                    std::cout << "CHANGING PREDICATES: " << StateDict::getPredName(changeable[j].first) << " (" <<
                                 changeable[j].first << ") from " << changeable[j].second  << " to " <<
                                 s.assignments[i].second << std::endl;
                    // DO nothing as we return the full suggestion
                    s.changed = true;
                }
                else {
                    std::cout << "IGNORING CHANGE IN PREDICATES: " << StateDict::getPredName(changeable[j].first) << " (" <<
                              changeable[j].first << ") from " << changeable[j].second  << " to " <<
                              s.assignments[i].second << std::endl;
                    s.assignments[i].second = changeable[j].second; // Set the suggestion to the already known value
                    s.changed = false;
                }
            }
        }
    }
    return s;
}


// c_id is the reference child id
double PlanSpaceSuggester::computeNodeMetric(size_t c_id, NodePtr n, const std::vector<bState> &d, int strategy) {
//#define COMPUTEVARIANCE // for debug
    double rsum = 0;
    int nchilds = 0;
    for (auto c_it = n->children.begin(); c_it != n->children.end(); ++c_it) {
        if (c_it->first == c_id) continue;
        if (c_it->second.max_reward_idx == -1) continue;
        auto s = StateDict::getState(c_it->second.state[c_it->second.max_reward_idx]);
    //for (auto ittt = s.begin(); ittt != s.end(); ++ittt)
    //    std::cout << *ittt << " ";
    //std::cout << c_it->second.reward[c_it->second.max_reward_idx] << std::endl;
        rsum += c_it->second.reward[c_it->second.max_reward_idx];
        ++nchilds;
    }
    assert(n->children[c_id].max_reward_idx != -1); // Shouldn't be 0 as n must be a maxreward child
    double xreward = n->children[c_id].reward[n->children[c_id].max_reward_idx]; // Reward of the maximum

    // ret is now the sum of all the elements but c_id
    //int nchilds = (n->children.size()-1); // -1 because we're not counting c_id!
    if (nchilds == 0) return 0;
    if (strategy == M_SUMDIFFS) {
        #ifdef COMPUTEVARIANCE
        double mean = rsum / nchilds;
        double sigma = 0;
        for (auto c_it = n->children.begin(); c_it != n->children.end(); ++c_it) {
            if (c_it->first == c_id or c_it->second.max_reward_idx == -1) continue;
            double aux = c_it->second.reward[c_it->second.max_reward_idx]-mean;
            sigma += aux*aux;
        }
        sigma /= nchilds - 1;
        std::cout << "s = " << std::sqrt(sigma) << " " << " s² = " << sigma << std::endl;
        #endif
        // SUMDIFFS is sum of the differences: x-y1 + x-y2 + ... = nx-SUM(y)
        //return ((nchilds*xreward) - rsum)/nchilds;
        return (xreward - (rsum/nchilds));// optimized version
    }
    else if (strategy == M_SUMDIFFS_NORM) {
        // Divide by rmax to normalize in range 0-1
        // sum(rmax - r_i)/(n*Rmax) = n*Rmax-sum(r_i)/n*Rmax = 1 - sum(r_i)/n*rmax
        return 1-(rsum/(nchilds*xreward));
    }
    ROS_ERROR_STREAM("[computeNodeMetric] Unknown metric " << strategy << std::endl);
    return rsum;
}

// d is a matrix of differences
// ni is the selected node with max metric
Suggestion PlanSpaceSuggester::computeNodeSuggestion(const std::vector<bState> &d, NodeInfoPtr ni) {
    // Sum differences. The column with more differences will be the suggested one
    assert(d.size() > 0);
    assert(ni->max_reward_idx != -1);
    Suggestion sugg;
    sugg.ndiffs = 0; // Holds the maximum
    for (size_t i = 0; i < d[0].size(); ++i) {  // iterate columns!
        int s = 0; // Sum of diffs
        // Sum column
        for (size_t j = 0; j < d.size(); ++j) s += d[j][i];
        if (s > sugg.ndiffs) {
            sugg.assignments.clear();
            sugg.ndiffs = s;
            sugg.assignments.emplace_back(std::make_pair(i, StateDict::getState(ni->state[ni->max_reward_idx])[i]));
        }
        else if (s == sugg.ndiffs) {
            sugg.assignments.emplace_back(std::make_pair(i, StateDict::getState(ni->state[ni->max_reward_idx])[i]));
        }
    }
    sugg.reward = ni->reward[ni->max_reward_idx];
    return sugg;
}

// Previous assignments are already passed
std::vector<Suggestion> PlanSpaceSuggester::getMinSuggestions(PlanTree &pt, Assignment& assignment, int n, bool changes, int chK) {
    if (not changes and not assignment.empty()) pt.recomputeMaxs(assignment);

    std::vector<Suggestion> sgg;
    double curr_r = 0;
    NodeInfo max_r_child = pt.getRoot()->children[getMaxRChildren(pt.getRoot())[0]];
    if (max_r_child.max_reward_idx == -1) return sgg; // No suggestions left, we're done!
    //double max_r = max_r_child.reward[max_r_child.max_reward_idx];
    int i = 0;

    Assignment initial = assignment;
    if (changes) assignment.clear(); // Remove the pre-set assignments for changes

    while ((n < 0 or i < n) and /*curr_r < max_r or*/ (assignment.size() < StateDict::numPredicates())) {
        // Get Suggestion
        Suggestion s;
        if (changes) s = suggestWithChanges(pt, chK, initial, assignment);
        else s = suggestAdditions(pt, assignment);
        assignment.insert(assignment.end(), s.assignments.begin(), s.assignments.end());

        // Update curr_r
        //curr_r = s.reward;

        // Print suggestion
        std::cout << "Suggestion " << i+1 << ": ";
        for (auto ai = s.assignments.begin(); ai != s.assignments.end(); ++ai) {
            std::string predname = StateDict::getPredName(ai->first);
            std::cout << predname << " = " << StateDict::getPredValue(predname, ai->second) << " (" << ai->first
                      << " = " << ai->second << ") ";
        }
        std::cout << curr_r << " " << s.reward << " " << s.ndiffs << std::endl;

        // Store suggestion
        sgg.push_back(s);

        // Update tree with the suggestion
        pt.recomputeMaxs(assignment);
        ++i;
    }
    return sgg;
}

int PlanSpaceSuggester::sumMat(std::vector<bState> v) {
    int s = 0;
    for (size_t i = 0; i < v.size(); ++i) {
        for (size_t j = 0; j < v[i].size(); ++j) s += v[i][j];
    }
    return s;
}


int main_(int argc, char* argv[]) {
    std::cout << "Hello" << std::endl;
    StateDict::loadPredicates("/home/gcanal/Dropbox/PrefsIROS19/domains/shoe_types.txt");
    //StateDict::loadPredicates("/home/gerard/code/catkin_ws/src/iros2019/shoe_types.txt");
    //PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/shoe_planslongest.txt");
    PlanTree pt("/home/gcanal/Dropbox/PrefsIROS19/planspace/shoe_plans.txt");
    //PlanTree pt("/home/gerard/code/catkin_ws/src/iros2019/shoe_plans.txt");
    std::cout << "Tree size: " << pt.size() << std::endl;

    PlanSpaceSuggester n;
    Assignment a;
    n.getMinSuggestions(pt, a);
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
