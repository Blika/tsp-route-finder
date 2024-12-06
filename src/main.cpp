#include "graph/graph.hpp"
#include "node/node.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <random>
#include <queue>
#include <vector>
#include <matplot/matplot.h>

const float usd_per_unit = 10.f;
const float min_neighbors = 2.f;
const float max_neighbors = 6.f;
const uint32_t node_count = 100;
const float min_r = 400.f;
const float max_r = 700.f;
static Graph* graph;
static std::vector<std::vector<float>> distanceMatrix(node_count, std::vector<float>(node_count, std::numeric_limits<float>::infinity()));
static std::vector<double> xv,yv;
static std::vector<std::string> txt;

float rnd(const float& min, const float& max){
    std::default_random_engine rnd_eng{std::random_device{}()};
    std::uniform_real_distribution<float> rnd_dist(min,max);
    return rnd_dist(rnd_eng);
}

void drawCircle(){
    double xc = 0;
    double yc = 0;
    std::vector<double> theta =  matplot::linspace(0, 2 *  matplot::pi);
    std::vector<double> x = matplot::transform(theta, [=](auto theta) { return graph->getRadius() * std::cos(theta) + xc; });
    std::vector<double> y = matplot::transform(theta, [=](auto theta) { return graph->getRadius() * std::sin(theta) + yc; });
    matplot::plot(x, y);
    matplot::axis(matplot::equal);
    matplot::scatter(xv, yv, 2.f);
    matplot::text(xv,yv,txt)->font_size(8);
}

void plotGraph(){
    srand(time(0));
    float radius = rnd(min_r, max_r);
    graph = new Graph(radius);
    for(uint32_t i = 0; i < node_count; ++i){
        float x,y;
        do{
            x = ((double) rand() / RAND_MAX) * 2 * radius - radius;
            y = ((double) rand() / RAND_MAX) * 2 * radius - radius;
        }while(x * x + y * y > radius * radius);
        xv.push_back(x);
        yv.push_back(y);
        std::string t = " " + std::to_string(i);
        txt.push_back(t);
        graph->addNode(new Node(i, x, y));
    }
    for(uint32_t i = 0; i < node_count; ++i){
        Node* node = graph->getNode(i);
        std::unordered_map<float, std::queue<uint32_t>> d_to_n;
        std::vector<float> distances;
        for(uint32_t j = 0; j < node_count; ++j){
            if(j == i) continue;
            Node* n = graph->getNode(j);
            float d = node->distance(n);
            distances.push_back(d);
            d_to_n[d].push(j);
        }
        std::sort(distances.begin(), distances.end());
        uint32_t connections = static_cast<uint32_t>(rnd(min_neighbors,max_neighbors));
        uint32_t c = 0;
        uint32_t total = node->getTotalConnections();
        while(total < connections){
            if(c >= node_count) break;
            float distance = distances[c];
            uint32_t n_i = d_to_n[distance].front();
            d_to_n[distance].pop();
            if(d_to_n[distance].empty()){
                c++;
            }
            Node* n = graph->getNode(n_i);
            if(n->hasConnection(node) || n->getTotalConnections() >= max_neighbors){
                continue;
            }
            node->addConnection(n);
            n->addConnection(node);
            distanceMatrix[i][n_i] = distance;
            distanceMatrix[n_i][i] = distance;
            matplot::plot({node->getX(), n->getX()}, {node->getY(), n->getY()}, "r");
            matplot::hold(true);
            total++;
        }
    }
}

void dijkstraPath(const uint32_t& start, const uint32_t& dest){
    std::vector<float> dist(node_count, std::numeric_limits<float>::infinity());
    std::vector<uint32_t> prev(node_count, -1);
    dist[start] = 0;
    std::priority_queue<std::pair<float, uint32_t>, std::vector<std::pair<float, uint32_t>>, std::greater<std::pair<float, uint32_t>>> pq;
    pq.push({0, start});

    while(!pq.empty()){
        auto [currentDist, u] = pq.top();
        pq.pop();
        if(currentDist > dist[u]){
            continue;
        }
        for(uint32_t v = 0; v < node_count; ++v){
            if(distanceMatrix[u][v] < std::numeric_limits<float>::infinity()){
                double newDist = dist[u] + distanceMatrix[u][v];
                if(newDist < dist[v]){
                    dist[v] = newDist;
                    prev[v] = u;
                    pq.push({newDist, v});
                }
            }
        }
    }

    std::vector<float> px, py;
    Node* pn = nullptr;
    float totalDist = 0.f;
    for(uint32_t at = dest; at != -1; at = prev[at]){
        Node* node = graph->getNode(at);
        px.push_back(node->getX());
        py.push_back(node->getY());
        if(pn == nullptr){
            pn = node;
        }else{
            totalDist += distanceMatrix[pn->getIndex()][node->getIndex()];
            pn = node;
        }
    }
    if(totalDist != 0.f){
        float usd = totalDist * usd_per_unit;
        std::cout << "Found path is " << totalDist << " units long" << '\n';
        std::cout << "It would cost you " << usd << "$" << '\n';
    }else{
        std::cout << "No path found" << '\n';
        std::cout << "You are free of charge!" << '\n';
    }
    matplot::plot(px,py,"r");
    matplot::hold(true);
}

int main(){
    try{
        auto s1 = matplot::subplot(1, 2, 0);
        plotGraph();
        drawCircle();

        while(true){
            std::string from,dest;
            uint32_t f,d;
            std::cout << "From: ";
            std::cin >> from;
            f = std::min<uint32_t>(node_count-1,std::max<uint32_t>(std::atoi(from.c_str()), 0));
            std::cout << "To: ";
            std::cin >> dest;
            d = std::min<uint32_t>(node_count-1,std::max<uint32_t>(std::atoi(dest.c_str()), 0));

            std::cout << "Looking for path between " << f << " and " << d << "..." <<'\n';
            
            auto s2 = matplot::subplot(1, 2, 1);
            dijkstraPath(f,d);
            drawCircle();
            matplot::show();

            std::string a;
            std::cout << "Try again? (1 - yes, 0 - no)";
            std::cin >> a;
            if(a == "1"){
                matplot::cla(s2);
            }else{
                break;
            }
        }
        
    }catch(const std::exception& e){
		std::cerr << e.what() << '\n';
	}
    delete graph;
    return 0;
}