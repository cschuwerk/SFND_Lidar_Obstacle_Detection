#include "cluster.h"

void
euclidianCluster::pointsInProximity(const std::vector<std::vector<float>> &points, int point, kdTree::tree *tree, float distanceTol,
                  std::vector<int> &cluster, std::vector<bool> &pointProcessed) {

    //Get nearby points (including this point itself)
    std::vector<int> nearbyPoints = tree->search(points[point], distanceTol);
    for (int index : nearbyPoints) {
        // Add point to the cluster and mark it as processed
        pointProcessed[point] = true;
        cluster.push_back(point);

        // If this point has not been processed yet, find all points in proximity
        if (!pointProcessed[index]) {
            pointsInProximity(points, index, tree, distanceTol, cluster, pointProcessed);
        }
    }

}

std::vector<std::vector<int>>
euclidianCluster::euclideanCluster(const std::vector<std::vector<float>> &points, kdTree::tree *tree, float distanceTol) {

    std::vector<std::vector<int>> clusters;
    std::vector<bool> pointProcessed(points.size(), false);

    for (unsigned int i = 0; i < points.size(); ++i) {
        if (!pointProcessed[i]) {
            std::vector<int> cluster;
            pointsInProximity(points, i, tree, distanceTol, cluster, pointProcessed);
            clusters.push_back(cluster);
        }
    }

    return clusters;

}