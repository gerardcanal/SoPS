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

public:
    PlanSpaceSuggesterNode(const std::string& planspace_path);

    void suggestNew();
    void suggestChanges();

};


#endif //IROS2019_PLANSPACESUGGESTERNODE_H
