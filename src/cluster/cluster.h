#ifndef PLAYBACK_CLUSTER_H
#define PLAYBACK_CLUSTER_H

#include <chrono>
#include <string>
#include <vector>
#include "../kdtree/kdtree.h"


namespace euclidianCluster {

    void pointsInProximity(const std::vector<std::vector<float>> &points, int point, kdTree::tree *tree, float distanceTol,
                      std::vector<int> &cluster, std::vector<bool> &pointProcessed);

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, kdTree::tree *tree, float distanceTol);
}

#endif /*PLAYBACK_CLUSTER_H*/