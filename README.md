# Pathfinder
C++ demo to navigate polygonal obstacles

This program uses the C++ standard library list container to develop
an algorithm for navigating from one point to another around 
polygonal obstacles (note that the start point of each obstacle must 
be copied to the end of the list to form a closed figure). (Mathematical) vector 
methods are implemented in the class "CPair" and employed throughout 
non-member functions below to perform this task. There is also a 
function written to optimize the avoidance path by cutting corners 
in the route wherever possible.
 
The "DEBUGGING" flag can be turned on to see more details of the 
process. Some (almost correct) WxMaxima code is generated at the end
of the program output to help the user visualize what the 
program has accomplished.
