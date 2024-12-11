#include "graph.hpp"
#include "chunk.hpp"
#include "../path/path_finder.hpp"
#include "../threadpool/threadpool.hpp"
#include "../node/node.hpp"
#include <queue>
#include <random>
#include <matplot/matplot.h>

Graph::Graph(const float& r)
    : radius{r}{
    distanceMatrix = std::vector<std::vector<float>>(PathFinder::getInstance()->node_count, std::vector<float>(PathFinder::getInstance()->node_count, std::numeric_limits<float>::infinity()));
}

Graph::~Graph(){
    for(auto&[k,v]: nodes){
        delete nodes[k];
    }
    for(auto&[k,v]: chunks){
        delete v;
    }
}

void Graph::drawCircle(){
    double xc = 0;
    double yc = 0;
    std::vector<double> theta =  matplot::linspace(0, 2 *  matplot::pi);
    std::vector<double> x = matplot::transform(theta, [=](auto theta) { return radius * std::cos(theta) + xc; });
    std::vector<double> y = matplot::transform(theta, [=](auto theta) { return radius * std::sin(theta) + yc; });
    matplot::plot(x, y);
    matplot::axis(matplot::equal);
    matplot::scatter(xv, yv, 2.f);
    matplot::text(xv,yv,txt)->font_size(8);
}

void Graph::plot(const int16_t& node_count, const float& min_neighbors, const float& max_neighbors){
    srand(time(0));
    for(int16_t i = 0; i < node_count; ++i){
        float x,y;
        do{
            x = ((double) rand() / RAND_MAX) * 2 * radius - radius;
            y = ((double) rand() / RAND_MAX) * 2 * radius - radius;
        }while(x * x + y * y > radius * radius);
        xv.push_back(x);
        yv.push_back(y);
        std::string t = " " + std::to_string(i);
        txt.push_back(t);
        addNode(new Node(i, x, y));
        int chunkX = (int)std::floor(x) >> 4;
        int chunkY = (int)std::floor(y) >> 4;
        int64_t chunkIndex = ((chunkX << 16) | ((chunkY) & 0xffff));
        if(!chunks.contains(chunkIndex)){
            chunks[chunkIndex] = new Chunk(chunkX, chunkY);
        }
        chunks[chunkIndex]->addNode(nodes[i]);
    }
    for(int16_t i = 0; i < node_count; ++i){
        Node* node = getNode(i);
        std::vector<std::pair<float, int16_t>> distances;
        for(int16_t j = 0; j < node_count; ++j){
            if(j == i) continue;
            Node* n = getNode(j);
            float d = node->distance(n);
            distances.push_back(std::pair(d,j));
        }
        std::sort(distances.begin(), distances.end());
        int16_t connections = static_cast<int16_t>(PathFinder::getInstance()->rnd(min_neighbors,max_neighbors));
        int16_t c = 0;
        int16_t total = node->getTotalConnections();
        int16_t sameChunk = 0;
        while(total < connections && !distances.empty() && c < node_count){
            if(c >= node_count) break;
            auto p = distances.front();
            distances.erase(distances.begin());
            Node* n = getNode(p.second);
            if(n->hasConnection(node) || n->getTotalConnections() >= max_neighbors){
                continue;
            }
            if(n->getChunk() == node->getChunk()){
                sameChunk++;
            }else{
                node->getChunk()->connections[n->getChunk()->getIndex()].push_back(std::pair(i,p.second));
                n->getChunk()->connections[node->getChunk()->getIndex()].push_back(std::pair(p.second,i));
            }
            totalDistance += p.first;
            node->addConnection(n);
            n->addConnection(node);
            distanceMatrix[i][p.second] = p.first;
            distanceMatrix[p.second][i] = p.first;
            matplot::plot({node->getX(), n->getX()}, {node->getY(), n->getY()}, "r");
            matplot::hold(true);
            total++;
        }
        
        if(sameChunk == 0){
            while(!distances.empty() && c < node_count){
                auto p = distances.front();
                distances.erase(distances.begin());
                Node* n = getNode(p.second);
                if(n->hasConnection(node) || n->getTotalConnections() >= max_neighbors || n->getChunk() != node->getChunk()){
                    continue;
                }
                node->addConnection(n);
                n->addConnection(node);
                totalDistance += p.first;
                distanceMatrix[i][p.second] = p.first;
                distanceMatrix[p.second][i] = p.first;
                matplot::plot({node->getX(), n->getX()}, {node->getY(), n->getY()}, "r");
                matplot::hold(true);
                break;
            }
        }
    }
    
    for(auto&[xy,ch]: chunks){
        std::cout << xy << ": ";
        for(auto&[in,n]: ch->getNodes()){
            std::cout << in << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "Caching data..." << '\n';
    for(auto&[xy,ch]: chunks){
        for(auto&[in,n]: ch->getNodes()){
            for(auto&[in1,n1]: ch->getNodes()){
                PathFinder::getInstance()->getThreadPool()->assignNewTask(std::bind(&Graph::cachePaths, this, xy,in,in1),1);
            }
        }
    }
    PathFinder::getInstance()->getThreadPool()->wait();
}

void Graph::cachePaths(const int64_t& xy,const int16_t& from, const int16_t& to){
    std::pair<float,std::vector<int16_t>> p = PathFinder::getInstance()->findPathResursive(from,to,-1,0.f,{},{},chunks[xy]->getNodeCount(),0.f);
    cachedPaths[xy][from][to] = p;
    /*{
        std::lock_guard<std::mutex> lock(cachedPathsMutex);
        std::cout << "PATHS (" << from << " to " << to << "): " << xy <<": ";
        for(auto& k: cachedPaths[xy][from][to].second){
            std::cout << k << " ";
        }
        std::cout << std::endl;
    }*/
}

size_t Graph::getChunkCount(){
    return chunks.size();
}

Chunk* Graph::getChunk(const int64_t& xy){
    if(!chunks.contains(xy)) return nullptr;
    return chunks[xy];
}

std::unordered_map<int64_t, Chunk*>& Graph::getChunks(){
    return chunks;
}

float Graph::getTotalDistance(){
    return totalDistance;
}

float Graph::getRadius(){
    return radius;
}

void Graph::addNode(Node* node){
    if(hasNode(node->getIndex())) return;
    nodes[node->getIndex()] = node;
}

bool Graph::hasNode(const int16_t& index){
    return nodes.contains(index);
}

Node* Graph::getNode(const int16_t& index){
    if(!hasNode(index)) return nullptr;
    return nodes[index];
}