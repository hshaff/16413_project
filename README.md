# 16413_project README
PADM Project Section 1 -- Activity Planning

Assumptions: 
In the process of designing the problem, we made the following assumptions about the environment.
1) There is one drawer that can be opened
2) There is one countertop that objects can be placed on
3) There is one stovetop that objects can be placed on
4) The arm of the robot can only hold one object at a time (meaning the robot cannot be holding an object while opening the drawer for example)
5) The drawer begins closed and empty
6) The only objects that can be picked up and placed down are the sugar box and the spam box

In the following phases of the project, the solution can be made more robust by including additional drawers, countertops, and stovetops if the assumptions are incorrect. Additionally, for simplicity when designing the problem, we assumed that the only objects that can be picked up or placed were the sugar and the spam. This can also be made more robust if this assumption is incorrect.

Files: 
    blockworld.pddl -- The PDDL domain script
    pb1.pddl --  The PDDL problem script
    main.py -- The main run script

Key Functions: 
    pickup-sugar -- the arm picks up the sugar box if the arm is not already holding an object and the sugar is on the       stovetop
    place-sugar -- the arm places the sugar on the countertop if the arm is holding an object, the sugar is no longer on the stovetop, and the countertop is clear
    pickup-spam -- the arm picks up the spam box if the arm is not already holding an object and the spam is on the countertop
    place-spam -- the arm places the spam in the drawer if the arm is holding an object, the drawer has been opened, and the spam is no longer on the countertop
    open-drawer -- the arm opens the drawer if the drawer is closed and the arm is not already holding an object
    close-drawer -- the arm closes the drawer if the drawer is open and the arm is not already holding an object

Solver:

Challenges:

PADM Project Section 2 -- Motion Planning
TODO

PADM Project Section 3 -- Trajectory Optimization
TODO