from pddl_parser_root.pddl_parser.planner import Planner
from pddl_parser_root.pddl_parser.PDDL import PDDL_Parser
from Graph import graph

def generate_graph()
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
    G = generate_graph()
    plan = solve()
    
    return plan

if __name__ == "__main__":
    print(plan)
    main()