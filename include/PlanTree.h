//
// Created by gcanal on 04/02/19.
//

#ifndef IROS2019_PLANTREE_H
#define IROS2019_PLANTREE_H

#include <vector>
#include <map>
#include <string>
#include <memory> // shared_ptr
#include <cassert>
#include <iostream>


class ActionDict {
private:
    static std::map<std::string, int> _action_ids;
    static std::vector<std::string> _actions;

public:
    int addAction(const std::string& a);
    std::string getAction(int i);
    int getActionId(const std::string& a);
    bool hasAction(const std::string& a);
};

// Tree node
class _Node; // Fwd declaration
typedef std::shared_ptr<_Node> NodePtr;
class _Node {
public:
    int action_id; // Node action. -1 Means root node!
    std::map<int, NodePtr> children; // int is the action id from the ActionDict
    std::map<int, double> max_rewards; // int is the action id, double the reward of the child i

    explicit _Node(int a_id);
    ~_Node();
    void addChild(int a_id, double reward);
    NodePtr getChild(int i);
    bool hasChild(int i);
};


class PlanTree {
private:
    NodePtr root;
public:
    PlanTree();
    PlanTree(const std::string& planspace_path);

    void loadTreeFromFile(const std::string& planspace_path);
};


#endif //IROS2019_PLANTREE_H
