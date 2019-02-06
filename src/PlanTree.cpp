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
        if (StateDict::hasState(state)) {

        }
        else {
            state_id = StateDict::addState(state); // State
        }

        std::vector<std::string> plan;
        str_plan = m[2]; // Action sequence / plan
        for (auto rit = std::regex_iterator<std::string::const_iterator>(str_plan.begin(), str_plan.end(), action_separator); rit != rend; ++rit) {
            if (rit->str(1).empty()) continue;
            plan.push_back(rit->str(1));
        }

        // Insert in tree
    }

    planspace_file.close();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////