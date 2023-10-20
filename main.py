from pddl_parser_root.pddl_parser.planner import Planner
from pddl_parser_root.pddl_parser.PDDL import PDDL_Parser
from Graph import SearchNode, Path
import math
import numpy as np 

def generate_graph(states, actions, predicates):
    num_predicates = len(predicates)
    print('states', states)
    # print('actions', actions)
    # print('predicates', predicates)
    next_state = states
    for a in actions:
        next_state = states
        if can_perform_action(states, a):
            print(a.name, a.del_effects)
            next_state.union(next_state, a.add_effects, a.del_effects)
            print(next_state)

    return

def BFS(states, actions, pos_goal, neg_goal):
    start_node = SearchNode(states)
    q = [start_node]
    visited = set()
    while q:
        current_node = q.pop(0)
        if check_goal(current_node.state, pos_goal, neg_goal):
            return Path(current_node)
        for a in actions:
            if can_perform_action(current_node.state, a):
                diff = current_node.state.symmetric_difference(a.del_effects)
                next_state = frozenset.union(diff, a.add_effects)
                next_node = SearchNode(next_state, current_node, action=a.name)
                if next_node.state not in visited:
                    q.append(next_node)
                    visited.add(next_state)
    return False

def check_goal(state, positive_goals, negative_goals):
    """return true if state satisfies goal state"""
    if positive_goals.intersection(state) != positive_goals:
        return False
    if negative_goals.intersection(state):
        return False
    return True       

def can_perform_action(state, action):
    """
    Given an action and state, return true true if the action can be performed, false o.w. 
    """
    if action.positive_preconditions.intersection(state) != action.positive_preconditions:
        return False
    if action.negative_preconditions.intersection(state):
        return False
    return True
def find_ff_heuristic():
    return 

def solve():
    # Solve the activity planning problem using EHC
    return 

def main():
    plan = ''
    parser = PDDL_Parser()
    domain_filename = 'blockworld.pddl'
    problem_filename = 'pb1.pddl'
    parser.parse_domain(domain_filename)
    parser.parse_problem(problem_filename)

   # G = generate_graph(parser.state, parser.actions, parser.predicates)
    # plan = solve()
    # print(plan)
    B = BFS(parser.state, parser.actions, parser.positive_goals, parser.negative_goals)
    print(B.path)
    return 

if __name__ == "__main__":
    main()