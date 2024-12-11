# tsp-route-finder
TSP solution.
Gnuplot is required.

# System requirements
Modern high-core CPU (good DRAM controller and solid L3 cache capacity and speed) is advised.

# What does it do?
It generates 100 nodes inside of a circle. Each node is connected to 2-6 nearby nodes.</br>
Once they all have been connected, you are shown a plot with all possible routes. You can only travel between nodes that are connected to each other.

Using CLI you can choose a starting node and your destination. After some time, you are shown an optimal path to visit all existing nodes.

# Algorithm
Iterating through each possible path is pretty time consuming (provided we have 100 nodes). Instead we divide the circle by chunks.</br>
For each chunk we calculate and store each possible best path between all nodes inside that chunk. We also store chuks' connections to each other.</br>
Since we cached all possible paths between nodes in every chunk, all we need to do is to find the best path of chunks.</br>
If we visited all the chunks and ended up in a wrong one, we use Dijkstra's algorithm to reach our final destination.</br>

Found path would not be the best one, but optimal still.
