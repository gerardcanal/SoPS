//
// Created by gcanal on 05/02/19.
//

#ifndef IROS2019_PLANSPACESUGGESTERNODE_H
#define IROS2019_PLANSPACESUGGESTERNODE_H

#include <ros/ros.h>
#include <iostream>
#include "PlanTree.h"


// Metrics
#define M_SUMDIFFS 1
#define M_SUMDIFFS_NORM 2
#define M_DIFFMU 3
#define M_IG 4

struct DiffResults {
    std::vector<bState> S;
    NodeInfoPtr node;
    double metric; // IG...
};

struct Suggestion {
    Assignment assignments;
    int ndiffs;
    double reward; // Suggestion reward -> from the node that generated the suggestion
    int nchanges = 0;
    int nadditions = 0;
};

class PlanSpaceSuggester {
private:
    std::vector<size_t> getMaxRChildren(NodePtr n);
    void join(std::vector<DiffResults> &a, std::vector<DiffResults> &b);

public:
    PlanSpaceSuggester();

    std::vector<DiffResults> getMaxChildDiffs(NodePtr root, const bState& mask);
    Suggestion suggestAdditions(PlanTree pt, const Assignment &assignment);
    Suggestion suggestWithChanges(PlanTree pt, int K, const Assignment &changeable, const Assignment &assignment=Assignment());
    std::vector<Suggestion> getMinSuggestions(PlanTree& pt, Assignment& assignment, int n=-1 , bool changes=false, int chK=0);

    Suggestion computeNodeSuggestion(const std::vector<bState> &v, NodeInfoPtr ni);
    double computeNodeMetric(size_t c_id, NodePtr n, const std::vector<bState> &v, int strategy = M_SUMDIFFS_NORM);

    int sumMat(std::vector<bState> vector);
};


#endif //IROS2019_PLANSPACESUGGESTERNODE_H
