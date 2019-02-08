//
// Created by gcanal on 04/02/19.
//

#include <PlanTree.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// StateDict class
// Static members
std::map<std::string, std::vector<std::string>> StateDict::_type_values;
std::map<std::string, std::pair<std::string, size_t>> StateDict::_state_types;
std::vector<State> StateDict::_states;
std::map<State, size_t> StateDict::_states_ids;
std::regex StateDict::re_csv("([^,=]+)=([^,=]+)(:?,)?\\s?"); // [^,=] means "not , or =". Matches: (.*)=(.*),?
bool StateDict::init = false;

size_t StateDict::getIndex(const std::vector<std::string>& v, const std::string& s) {
    auto i = std::find(v.begin(), v.end(), s);
    assert(i != v.end());
    return (i - v.begin());
}

State StateDict::parseState(const std::string& s) {
    assert(init);
    // s has format pred=val, pred=val...
    State newstate(_state_types.size());

    std::smatch m;
    std::regex_iterator<std::string::const_iterator> rend;
    for (auto rit = std::regex_iterator<std::string::const_iterator>(s.begin(), s.end(), re_csv); rit != rend; ++rit) {
        // rit->str(i) returns ith match. 1 is the predicate, 2 is the value
        //std::cout << "'" << rit->str(1) << "' -- '" << rit->str(2) << "'" <<  std::endl;
        size_t i = getIndex(_type_values[_state_types[rit->str(1)].first], rit->str(2));
        newstate[_state_types[rit->str(1)].second] = int(i);
    }
    return newstate;
}

size_t StateDict::addState(const State& s) {
    auto find = _states_ids.find(s);
    if (find != _states_ids.end()) return find->second;

    size_t i = _states.size();
    _states.push_back(s);
    _states_ids[s] = i;
    return i;
}

void StateDict::loadPredicates(const std::string &path) {
    std::ifstream pred_definiton_file;
    pred_definiton_file.open(path);

    // Regular expressions to parse lines.
    std::regex re_type("([^:]+):"); // Format type: val1, val2, val3...
    std::regex re_type_list("(?:[^:]+:\\s?)?([^,]+)(?:,\\s?)?"); // Format type: val1, val2, val3...
    std::regex re_pred("([^:]+):\\s?([^\\s]*)"); // Format pred: type

    std::string line;
    bool parsing_types = false;
    bool parsing_preds = false;
    std::regex_iterator<std::string::const_iterator> rend;
    std::smatch m;
    int pid = 0;
    while (std::getline(pred_definiton_file, line)) {
        if (line == "TYPES") {
            parsing_types = true;
            parsing_preds = false;
        }
        else if (line == "PREDICATES") {
            parsing_preds = true;
            parsing_types = false;
            pid = 0;
        }

        else if (parsing_types) {
            if (not std::regex_search(line, m, re_type)) continue;
            std::string type = m[1];
            for (auto rit = std::regex_iterator<std::string::const_iterator>(line.begin(), line.end(), re_type_list);
                    rit != rend; ++rit) {
                _type_values[type].push_back(rit->str(1));
            }
        }
        else if (parsing_preds) {
            if (not std::regex_search(line, m, re_pred)) continue;
            _state_types[m[1]] = {m[2], pid++};
        }
    }
    pred_definiton_file.close();
    init = true;
}

size_t StateDict::numStateOptions() { // Computes the number of options
    size_t ret = 1;
    for (auto it: _state_types) {
        ret *= (_type_values[it.second.first].size() -1); // Remove the unknowns
    }
    return ret;
}

void StateDict::initialize(size_t n) {
    _states.reserve(n);
}

bool StateDict::hasState(const State &s) {
    return _states_ids.find(s) != _states_ids.end();;
}

size_t StateDict::getStateId(const State &s) {
    assert(hasState(s));
    return _states_ids[s];
}

size_t StateDict::numPredicates() {
    assert(init);
    return _state_types.size();
}


// diff(a, b) is a boolean vector where d[i] = a[i] != b[i];
bState StateDict::diff(const State &a, const State &b) {
    assert(a.size() == b.size());
    bState d(a.size());
    for (size_t i = 0; i < a.size(); ++i) d[i] = a[i] != b[i];
    return d;
}

bool StateDict::match(const Assignment &a, const State &s) {
    for (auto it = a.begin(); it != a.end(); ++it) {
        if (s[it->first] != it->second) return false;
    }
    return true;
}

State StateDict::getState(size_t i) {
    assert(i < _states.size());
    return _states[i];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ActionDict class

// Static members
std::map<std::string, size_t> ActionDict::_action_ids;
std::vector<std::string> ActionDict::_actions;

size_t ActionDict::addAction(const std::string &a) {
    if (!hasAction(a)) {
        size_t i = _actions.size();
        _action_ids[a] = i; // That's the index of the next added action
        _actions.push_back(a); // Add the action
        return i;
    }
    return _action_ids[a];
}

std::string ActionDict::getAction(int i) {
    assert(size_t(i) < _actions.size());
    return _actions[i];
}

size_t ActionDict::getActionId(const std::string &a) {
    assert(hasAction(a));
    return _action_ids[a];
}

bool ActionDict::hasAction(const std::string &a) {
    return _action_ids.find(a) != _action_ids.end();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Node class

_Node::_Node(size_t a_id) : action_id(a_id) {}

_Node::~_Node() {
    for (auto it: children) {
        it.second.child.reset(); // Not really needed but..
    }
}


NodePtr _Node::addChild(size_t a_id, double reward, size_t state) {
    if (hasChild(a_id)) { // node already exists
        auto ni = children.find(a_id);
        ni->second.reward.push_back(reward);
        ni->second.state.push_back(state);
        if (ni->second.reward[ni->second.max_reward_idx] < reward) {
            ni->second.max_reward_idx = ni->second.reward.size()-1;
        }
        return ni->second.child;
    }
    else {
        NodePtr child = std::make_shared<_Node>(a_id);
        children[a_id] = NodeInfo(child, reward, state);
        return child;
    }
}

NodeInfo _Node::getChild(size_t i) {
    assert(hasChild(i));
    return children[i];
}

bool _Node::hasChild(size_t i) {
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
    std::regex sections("(.*) \\| (.*) \\| (.*)");
    std::regex action_separator("([^;]*)(?:;\\s?)?");

    size_t numplans = StateDict::numStateOptions();
    StateDict::initialize(numplans);

    std::string line, str_plan;
    std::regex_iterator<std::string::const_iterator> rend;

    while (std::getline(planspace_file, line)) {
        // Parse line. Format is: "pref=val, pref=val | action1, action2 | R"
        std::smatch m;
        if (not std::regex_search(line, m, sections)) continue;
        State state = StateDict::parseState(m[1]);
        double reward = std::stod(m[3]); // Reward

        size_t state_id;
        if (StateDict::hasState(state)) state_id = StateDict::getStateId(state);
        else state_id = StateDict::addState(state); // State

        // Insert in tree
        str_plan = m[2]; // Action sequence / plan
        NodePtr current = root;
        for (auto rit = std::regex_iterator<std::string::const_iterator>(str_plan.begin(), str_plan.end(), action_separator); rit != rend; ++rit) {
            // rit->str(1) is the action
            if (rit->str(1).empty()) continue;
            size_t a_id = ActionDict::addAction(rit->str(1));
            current = current->addChild(a_id, reward, state_id); // Current will be the new child. AddChild updates the child if needed
        }
    }

    planspace_file.close();
}

NodePtr PlanTree::getRoot() {
    return root;
}

void PlanTree::recomputeMaxs(const Assignment &a) {
    recomputeMaxs(root, a);
}

void PlanTree::recomputeMaxs(NodePtr root, const Assignment &a) {
    for (auto it = root->children.begin(); it != root->children.end(); ++it) {
        int max_r = -1;
        // Iterate over all the rewards that lead to this state
        for (size_t i = 0; i < it->second.reward.size(); ++i) {
            if (StateDict::match(a, StateDict::getState(it->second.state[i]))
                and ((max_r == -1) or (it->second.reward[i] > it->second.reward[max_r]))) max_r = i;
        }
        it->second.max_reward_idx = max_r;
        recomputeMaxs(it->second.child, a);
    }
}

bool PlanTree::isEqual(const PlanTree &p) {
    return areEqual(root, p.root);
}

bool PlanTree::areEqual(NodePtrConst a, NodePtrConst b) {
    bool equal = a->action_id == b->action_id;
    equal &= a->children.size() == b->children.size();
    for (auto it = a->children.begin(); equal and it != a->children.end(); ++it) {
        auto bchild = b->children.find(it->first);
        if (bchild == b->children.end()) return false;
        equal &= it->first == bchild->first;

        equal &= it->second.state.size() == bchild->second.state.size();
        for (size_t j = 0; equal and j < it->second.state.size(); ++j)
            equal &= it->second.state[j] == bchild->second.state[j];

        equal &= it->second.reward.size() == bchild->second.reward.size();
        for (size_t j = 0; equal and j < it->second.reward.size(); ++j)
            equal &= it->second.reward[j] == bchild->second.reward[j];

        equal &= it->second.max_reward_idx == bchild->second.max_reward_idx;
        equal &= areEqual(it->second.child, bchild->second.child);
    }
    return equal;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////