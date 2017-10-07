To run my code, compile with mex:
mex planner.c WorldInfo.cpp Node.cpp

Then from within Matlab define robotstart, targetstart and call runtest; e.g.,
robotstart = [ 1 1 0 ];
targetstart = [ 20 10 0 ];
runtest('map3.txt', robotstart, targetstart);

The approach I took was to:

1. Each cycle perform Dijkstra from the goal to the start node -- I use the Dijkstra distances as my heuristic.  After reaching the start node, I expand Dijkstra another 100 nodes.

2. At any one time there is one thread running A Star.  If A Star expands a node that doesn't have a Dijkstra cost associated with it, I use the dumb euclidian distance (Euclidian distance squared).

Once I've computed A star once, I keep my path in a global variable.  I simply return the primitive from the path that's described in the global variable.  If the global variable is not set, then I wait for A star to complete its first iteration before returning a primitive.

Because I used globals, I had to be very careful with when to invalidate my globals.  Namely, I check the memory location of the map variable to determine if the map has changed -- if it has then I delete my globals.  Likewise, if for whatever reason, my computed best path does not arrive at my robot's position, then I invalidate the path by deleting it and waiting for the A star thread to recompute another best path.

As for the g-value -- I wanted to bias the algorithm to taking larger steps, so I set the g-value of a node to the parent's g-value + (50/distanceToParent).  In doing so, I bias the algorithm to take moves that have larger distance to parent.  Finally, I also used an epsilon of 0.1.  This helps the algorithm converge faster.

My code is incredibly memory efficient -- I spent a lot of time trying to make the map a bool* so that I could run it on the Linux computer lab machines; unfortunately, it was a fruitless effort.
