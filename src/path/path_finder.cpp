#include "path_finder.hpp"
#include "../graph/graph.hpp"
#include "../graph/chunk.hpp"
#include "../node/node.hpp"
#include "../threadpool/threadpool.hpp"
#include <algorithm>
#include <iostream>
#include <queue>
#include <random>
#include <matplot/matplot.h>

PathFinder* PathFinder::instance = nullptr;

PathFinder::PathFinder(){
    PathFinder::instance = this;
    threadpool = new ThreadPool();
    graph = new Graph(rnd(min_r, max_r));
    matplot::subplot(1, 2, 0);
    graph->plot(node_count, min_neighbors, max_neighbors);
    graph->drawCircle();
}

PathFinder::~PathFinder(){
    delete threadpool;
    delete graph;
}

PathFinder* PathFinder::getInstance(){
    return instance;
}

ThreadPool* PathFinder::getThreadPool(){
    return threadpool;
}

Graph* PathFinder::getGraph(){
    return graph;
}

float PathFinder::rnd(const float& min, const float& max){
    std::default_random_engine rnd_eng{std::random_device{}()};
    std::uniform_real_distribution<float> rnd_dist(min,max);
    return rnd_dist(rnd_eng);
}

std::pair<float, std::vector<int16_t>> PathFinder::dijkstraPath(const int16_t& start, const int16_t& dest){
    std::vector<float> dist(node_count, std::numeric_limits<float>::infinity());
    std::vector<int16_t> prev(node_count, -1);
    dist[start] = 0.f;
    std::priority_queue<std::pair<float, int16_t>, std::vector<std::pair<float, int16_t>>, std::greater<std::pair<float, int16_t>>> pq;
    pq.push({0.f, start});

    while(!pq.empty()){
        auto [currentDist, u] = pq.top();
        pq.pop();
        if(currentDist > dist[u]){
            continue;
        }
        for(int16_t v = 0; v < node_count; ++v){
            if(graph->distanceMatrix[u][v] < std::numeric_limits<float>::infinity()){
                double newDist = dist[u] + graph->distanceMatrix[u][v];
                if(newDist < dist[v]){
                    dist[v] = newDist;
                    prev[v] = u;
                    pq.push({newDist, v});
                }
            }
        }
    }

    Node* pn = nullptr;
    float totalDist = 0.f;
    std::vector<int16_t> path;
    for(int16_t at = dest; at != -1; at = prev[at]){
        path.push_back(at);
        Node* node = graph->getNode(at);
        if(pn == nullptr){
            pn = node;
        }else{
            totalDist += graph->distanceMatrix[pn->getIndex()][node->getIndex()];
            pn = node;
        }
    }
    std::reverse(path.begin(), path.end());
    return std::pair<float, std::vector<int16_t>>(totalDist, path);
}

std::pair<int16_t, std::vector<int64_t>> PathFinder::findChunkPath(int64_t currentChunk, const int64_t& endPoint, const int64_t& prevChunk, int16_t distance, std::unordered_map<int64_t, int64_t> visitedChunks, std::vector<int64_t> path, int16_t minDistance){
    if(minDistance > 0 && minDistance <= distance) return std::pair<int16_t,std::vector<int64_t>>(0,{});
    visitedChunks[currentChunk] += 1;
    path.push_back(currentChunk);
    if(endPoint == currentChunk){
        return std::pair<int16_t,std::vector<int64_t>>(distance,path);
    }
    Chunk* chunk = graph->getChunk(currentChunk);
    std::pair<int16_t,std::vector<int64_t>> foundPath(minDistance,{});
    for(auto&[k,v]: chunk->connections){
        if(k == prevChunk && graph->getChunk(currentChunk)->connections.size() > 1) continue;
        if(visitedChunks.contains(k) && (visitedChunks[k] > 2 || (visitedChunks[currentChunk] > 1))) continue;
        auto p = findChunkPath(k, endPoint, currentChunk, distance + 1, visitedChunks, path, foundPath.first);
        if(foundPath.first == 0 || (p.first > 0 && p.first < minDistance)){
            foundPath.first = p.first;
            foundPath.second = p.second;
        }
    }
    return foundPath;
}

void PathFinder::visitAll(int16_t currentNode, const int16_t& endPoint, float distance, std::unordered_map<int64_t, bool> unvisitedChunks, std::vector<int16_t> path){
    if((foundPath.first > 0.f && foundPath.first <= distance) || distance >= (graph->getTotalDistance() * 2)) return;
    Chunk* currentChunk = graph->getNode(currentNode)->getChunk();
    unvisitedChunks.erase(currentChunk->getIndex());
    path.push_back(currentNode);
    if(unvisitedChunks.size() == 0){
        Chunk* endChunk = graph->getNode(endPoint)->getChunk();
        if(currentChunk == endChunk){
            std::pair<float,std::vector<int16_t>> fp = graph->cachedPaths[currentChunk->getIndex()][currentNode][endPoint];
            distance += fp.first;
            path.insert(path.end(), fp.second.begin(), fp.second.end());
        }else{
            for(auto& [t,d] : graph->cachedPaths[currentChunk->getIndex()][currentNode]){
                distance += d.first;
                path.insert(path.end(), d.second.begin(), d.second.end());
                currentNode = t;
                break;
            }
            std::pair<float, std::vector<int16_t>> res = dijkstraPath(currentNode, endPoint);
            distance += res.first;
            path.insert(path.end(), res.second.begin(), res.second.end());
        }
        {
            std::lock_guard<std::mutex> lock(foundPathMutex);
            if(foundPath.first == 0.f || (distance > 0.f && distance < foundPath.first)){
                std::cout << "Path found " << distance << ", previous " << foundPath.first << std::endl;
                foundPath.first = distance;
                foundPath.second = path;
            }
        }
        return;
    }
    for(auto&[ch,b]: unvisitedChunks){
        std::unordered_map<int64_t, int64_t> vc;
        for(auto&[xy,ch]: graph->getChunks()){
            vc[xy] = 0;
        }
        auto p = findChunkPath(currentChunk->getIndex(), ch, INT64_MAX, 0, vc, {}, 0);
        std::vector<int64_t> pth = p.second;
        pth.erase(pth.begin());
        Chunk* chunk = currentChunk;
        std::unordered_map<int64_t, bool> uc = unvisitedChunks;
        for(auto i = 0; i < pth.size(); ++i){
            int64_t c = pth[i];
            //std::cout << c << '\n';
            auto ft = chunk->connections[c];
            std::vector<std::pair<float, std::pair<std::pair<int16_t, int16_t>,std::vector<int16_t>>>> pairs;
            for(auto&p: ft){
                auto k = graph->cachedPaths[chunk->getIndex()][currentNode][p.first];
                pairs.push_back(std::pair(k.first, std::pair(p, k.second)));
            }
            std::sort(pairs.begin(), pairs.end());
            auto p = pairs.front();
            distance += p.first;
            path.insert(path.end(), p.second.second.begin(), p.second.second.end());
            path.push_back(p.second.first.second);
            distance += graph->distanceMatrix[p.second.first.first][p.second.first.second];
            if(uc.contains(c)) uc.erase(c);
            chunk = graph->getChunk(c);
            currentNode = path.back();
        }
        threadpool->assignNewTask(std::bind(&PathFinder::visitAll, this, path.back(), endPoint, distance, uc, path),1);
        //visitAll(path.back(), endPoint, distance, uc, path);
    }
}

std::pair<float,std::vector<int16_t>> PathFinder::findPathResursive(int16_t currentNode, const int16_t& endPoint, const int16_t& prevNode, float distance, std::unordered_map<int16_t, int16_t> visitedNodes, std::vector<int16_t> path, const int16_t& max, float minDistance){
    if(minDistance > 0.f && minDistance <= distance) return std::pair<float,std::vector<int16_t>>(0.f,{});
    visitedNodes[currentNode] = visitedNodes.contains(currentNode) ? visitedNodes[currentNode] + 1 : 1;
    path.push_back(currentNode);
    if(visitedNodes.size() == max){
        if(endPoint != currentNode){
            std::pair<float, std::vector<int16_t>> res = dijkstraPath(currentNode, endPoint);
            distance += res.first;
            path.insert(path.end(), res.second.begin(), res.second.end());
        }
        return std::pair<float,std::vector<int16_t>>(distance,path);
    }
    std::pair<float,std::vector<int16_t>> foundPath(minDistance,{});
    std::unordered_map<int16_t, bool> map = graph->getNode(currentNode)->getConnections();
    Node* node = graph->getNode(currentNode);
    std::vector<int16_t> vec;
    for(auto&[k,v]: map){
        Node* n = graph->getNode(k);
        if(node->getChunk() == n->getChunk()){
            vec.push_back(k);
        }
    }
    for(auto& k: vec){
        if(k == prevNode && vec.size() > 1) continue;
        if(k == endPoint && visitedNodes.size() != max - 1 && vec.size() > 1) continue;
        if(visitedNodes.contains(k) && (visitedNodes[k] > 2 || (visitedNodes[currentNode] > 1)) && vec.size() > 1) continue;
        Node* n = graph->getNode(k);
        auto p = findPathResursive(k, endPoint, currentNode, distance + graph->distanceMatrix[currentNode][k], visitedNodes, path, max, foundPath.first);
        if(foundPath.first == 0.f || (p.first > 0.f && p.first < minDistance)){
            foundPath.first = p.first;
            foundPath.second = p.second;
        }
    }
    return foundPath;
}

void PathFinder::drawPath(std::vector<int16_t>& path){
    std::vector<float> px, py;
    for(int16_t& p: path){
        Node* node = graph->getNode(p);
        px.push_back(node->getX());
        py.push_back(node->getY());
    }
    matplot::plot(px,py,"b");
    matplot::hold(true);
}

void PathFinder::run(){
    while(true){
        std::string from,dest;
        int16_t f,d;
        std::cout << "From: ";
        std::cin >> from;
        f = std::min<int16_t>(node_count-1,std::max<int16_t>(std::atoi(from.c_str()), 0));
        std::cout << "To: ";
        std::cin >> dest;
        d = std::min<int16_t>(node_count-1,std::max<int16_t>(std::atoi(dest.c_str()), 0));

        std::cout << "Searching for path between " << f << " and " << d << "..." <<'\n';
        
        auto s2 = matplot::subplot(1, 2, 1);

        std::unordered_map<int64_t, bool> unvisitedChunks;
        for(auto&[xy,ch]: graph->getChunks()){
            unvisitedChunks[xy] = true;
        }
        visitAll(f, d, 0.f, unvisitedChunks, {});
        threadpool->wait();
        std::vector<int16_t> fixedPath;
        int16_t prev = -1;
        for(auto& p: foundPath.second){
            if(prev == -1 || prev != p){
                prev = p;
                fixedPath.push_back(p);
            }else{
                continue;
            }
        }
        /*Chunk* ch = graph->getNode(f)->getChunk();
        auto path = findPathResursive(f,d,-1,0.f,{},{},ch->getNodeCount(),0.f);*/
        if(foundPath.first == 0.f){
            std::cout << "No path found" << '\n';
        }else{
            std::cout << foundPath.first << ", " << (foundPath.first*usd_per_unit) << "$" <<'\n';
            for(int16_t& p: fixedPath){
                std::cout << p << " ";
            }
            std::cout << std::endl;
            drawPath(fixedPath);
        }
        graph->drawCircle();
        matplot::show();
        std::string a;
        std::cout << "Try again? (1 - yes, 0 - no) ";
        std::cin >> a;
        if(a == "1"){
            foundPath.first = 0.f;
            foundPath.second.clear();
            matplot::cla(s2);
        }else{
            break;
        }
    }
}