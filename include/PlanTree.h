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
#include <fstream>
#include <regex>
#include <algorithm>

typedef std::vector<int> State; // Represents a state assignment of relevant predicates
class StateDict {
private:
    // Type values
    static std::map<std::string, std::vector<std::string>> _type_values; // Map state type -> values
    // Name to types
    static std::map<std::string, std::pair<std::string, size_t>> _state_types; // List of predicates and its type, and the index of the order of the predicate
    static std::vector<State> _states; // List of ALL states
    static std::map<State, size_t> _states_ids; // List of ALL states
    static std::regex re_csv;

    static inline size_t getIndex(const std::vector<std::string>& v, const std::string& s);
    static bool init;

    // Format pred=val, pred=val, pred=val
    // Adds the state, returns the index to it in _states
public:
    static void loadPredicates(const std::string& path);
    static size_t addState(const State& s);
    static State parseState(const std::string& s);
    static size_t numStateOptions();
    static void initialize(size_t n);
    static size_t getStateId(const State& s);
    static bool hasState(const State& s);
};

class ActionDict {
private:
    static std::map<std::string, size_t> _action_ids;
    static std::vector<std::string> _actions;

public:
    static size_t addAction(const std::string& a);
    static std::string getAction(int i);
    static size_t getActionId(const std::string& a);
    static bool hasAction(const std::string& a);
};

// Tree node
class _Node; // Fwd declaration
typedef std::shared_ptr<_Node> NodePtr;
class _Node {
public:
    int action_id; // Node action. -1 Means root node!
    std::map<int, NodePtr> children; // int is the action id from the ActionDict
    std::map<int, double> max_rewards; // int is the action id, double the reward of the child i
    std::map<int, int> states; // Will store the preferences for each node. The key is the action id, value the state id

    explicit _Node(int a_id);
    ~_Node();
    void addChild(int a_id, double reward, int state);
    NodePtr getChild(int i);
    bool hasChild(int i);
};


class PlanTree {
private:
    NodePtr root;
public:
    PlanTree();
    explicit PlanTree(const std::string& planspace_path);

    void loadTreeFromFile(const std::string& planspace_path);
};


#endif //IROS2019_PLANTREE_H
