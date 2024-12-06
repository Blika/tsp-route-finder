# tsp-route-finder
Travelling Salesman Problem solution, Dijkstra's algorithm used.
Gnuplot is required.

# What does it do?
It generated 100 nodes inside of a circle. Each node is connected to 2-6 nearby nodes.
Once they all have been connected, you are shown a graph with all possible routes. You can only travel between nodes that are connected to each other.

Using CLI you can choose a starting node and your destination. Then, using Dijkstra's algorithm, the best path is shown to you on a graph.
Keep in mind that sometimes there are small "islands" of nodes that are not connected to the "mainland". These "islands" are unreachable from the "mainland", and vice-versa.

# Bugs
After picking the destination node and resizing Gnuplot's window, the first plot with all the routes may disappear on some systems. This bug is on Gnuplot's side.
