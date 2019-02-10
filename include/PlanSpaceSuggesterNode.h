//
// Created by gcanal on 05/02/19.
//

#ifndef IROS2019_PLANSPACESUGGESTERNODE_H
#define IROS2019_PLANSPACESUGGESTERNODE_H

#include <iostream>
#include "PlanTree.h"

class PlanSpaceSuggesterNode {
private:
    std::string planspace_path;

    std::vector<size_t> getMaxRChildren(NodePtr n);
public:
    PlanSpaceSuggesterNode(const std::string& planspace_path);

    void suggestNew(NodePtr root);
    void suggestChanges();

};


#endif //IROS2019_PLANSPACESUGGESTERNODE_H
