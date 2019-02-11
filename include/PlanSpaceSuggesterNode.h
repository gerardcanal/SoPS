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
#define M_DIFFMU 2
#define M_IG 3

struct DiffResults {
    std::vector<bState> S;
    NodeInfoPtr node;
    double metric; // IG...
};

class PlanSpaceSuggesterNode {
private:
    std::string planspace_path;

    std::vector<size_t> getMaxRChildren(NodePtr n);
    void join(std::vector<DiffResults> &a, std::vector<DiffResults> &b);

public:
    PlanSpaceSuggesterNode(const std::string& planspace_path);

    std::vector<DiffResults> getMaxChildDiffs(NodePtr root);
    DiffResults & suggestChanges(PlanTree pt);

    double computeNodeSuggestion(const std::vector<bState> &v, NodeInfoPtr ni);
    double computeNodeMetric(size_t c_id, NodePtr n, const std::vector<bState> &v, int strategy = M_SUMDIFFS);
};


#endif //IROS2019_PLANSPACESUGGESTERNODE_H
