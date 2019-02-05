//
// Created by gcanal on 04/02/19.
//

#include <PlanTree.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// StateDict class
// Static members
std::map<std::string, std::vector<std::string>> StateDict::_type_values;
std::map<std::string, std::pair<std::string, int>> StateDict::_state_types;
std::vector<State> StateDict::_states;
std::regex StateDict::re_csv("([^,=]+)=([^,=]+)(:?,\\s?)?"); // [^,=] means "not , or =". Matches: (.*)=(.*),?

int StateDict::getIndex(const std::vector<std::string>& v, const std::string& s) {
    return int(std::find(v.begin(), v.end(), s) - v.begin());
}

int StateDict::addState(const std::string& s) {
    // s has format pred=val, pred=val...
    State newstate(_state_types.size());

    std::smatch m;
    std::regex_iterator<std::string::const_iterator> rend;
    for (auto rit = std::regex_iterator<std::string::const_iterator>(s.begin(), s.end(), re_csv); rit != rend; ++rit) {
        // rit->str(i) returns ith match. 1 is the predicate, 2 is the value

        std::cout << rit->str(1) << " -- " << rit->str(2) << std::endl;
        int i = getIndex(_type_values[_state_types[rit->str(1)].first], rit->str(2));
        //TODO newstate[] = i;
    }
}

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


void _Node::addChild(int a_id, double reward, int state) {
    if (hasChild(a_id)) { // node already exists
        if (max_rewards[a_id] < reward) {
            max_rewards[a_id] = reward;
            states[a_id] = state;
        }
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

void PlanTree::loadTreeFromFile(const std::string &planspace_path) {
    std::ifstream planspace_file;
    planspace_file.open(planspace_path);

    // Regular expressions to parse lines.
    std::regex sections("(.*)\\|(.*)\\|(.*)");
    std::regex csv("(.*, )");

    std::string line;
    while (std::getline(planspace_file, line)) {
        // Parse line. Format is: "pref=val, pref=val | action1, action2 | R"
        std::smatch m;
        if (not std::regex_search(line, m, sections)) continue;
        StateDict::addState(m[1]); // State
        //m[2]; // Action sequence / plan
        double reward = std::stod(m[3]); // Reward

        // Insert in tree
    }

    planspace_file.close();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////