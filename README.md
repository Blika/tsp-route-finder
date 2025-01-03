# tsp-route-finder
TSP solution.
Gnuplot is required.

# What does it do?
It generates 100 nodes inside of a circle. Each node is connected to 2-6 nearby nodes.</br>
Once they all have been connected, you are shown a plot with all possible routes. You can only travel between nodes that are connected to each other.

Using CLI you can choose a starting node and your destination.

# Algorithm
Iterating through each possible path is pretty time consuming (provided we have 100 nodes). Instead we divide the circle by chunks.</br>
For each chunk we calculate and store each possible best path between all nodes inside that chunk. We also store chuks' connections to each other.</br>
Since we cached all possible paths between nodes in every chunk, all we need to do is to find the best chunk path.</br>
If we visited all the chunks and ended up in a wrong one, we use Dijkstra's algorithm to reach our final destination.</br>

You can also opt to use a faster algorithm that picks one path instantly. This does not search for the best possible path, but it is drastically faster.
