import subprocess
import os
import sys
sys.path.append('pddl-parser')
sys.path.append('pddl-parser/pddl_parser')
from PDDL import PDDL_Parser

def main():
    parser = PDDL_Parser
    #os.chdir(os.path.join(os.path.abspath(os.path.curdir), u'pddl-parser'))
    #os.system('python3 -B -m pddl_parser.PDDL ../blockworld.pddl ../pb1.pddl')

if __name__ == "__main__":
    main()