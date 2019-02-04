//
// Created by gcanal on 04/02/19.
//

#include <PlanTree.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ActionDict class

// Static members
std::map<std::string, int> ActionDict::_action_ids;
std::vector<std::string> ActionDict::_actions;

int ActionDict::addAction(const std::string &a) {
    if (!hasAction(a)) {
        int i = (int)_actions.size();
        _action_ids[a] = i; // That's the index of the next added action
        _actions.push_back(a); // Add the action
        return i;
    }
    return _action_ids[a];
}

std::string ActionDict::getAction(int i) {
    assert(i < _actions.size());
    return _actions[i];
}

int ActionDict::getActionId(const std::string &a) {
    assert(hasAction(a));
    return _action_ids[a];
}

bool ActionDict::hasAction(const std::string &a) {
    return _action_ids.find(a) != _action_ids.end();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Node class

_Node::_Node(int a_id) : action_id(a_id) {}

_Node::~_Node() {
    for (auto it: children) {
        it.second.reset(); // Not really needed but..
    }
}


void _Node::addChild(int a_id, double reward) {
    if (hasChild(a_id)) { // node already exists
        if (max_rewards[a_id] < reward) max_rewards[a_id] = reward;
    }
    else {
        children[a_id] = std::make_shared<_Node>(a_id); // new node
        max_rewards[a_id] = reward;
    }
}

NodePtr _Node::getChild(int i) {
    assert(hasChild(i));
    return children[i];
}

bool _Node::hasChild(int i) {
    return children.find(i) != children.end();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PlanTree class

PlanTree::PlanTree() {
    root = std::make_shared<_Node>(-1);
}

PlanTree::PlanTree(const std::string &planspace_path) : PlanTree() {
    loadTreeFromFile(planspace_path);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
    std::cout << "Hello" << std::endl;
}