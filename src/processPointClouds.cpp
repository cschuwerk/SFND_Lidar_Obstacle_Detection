// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time filtering process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_voxel (new pcl::PointCloud<PointT>());

    // Voxel-grid filtering
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(filterRes, filterRes, filterRes);
    voxel_filter.filter(*cloud_voxel);

    // Crop Box filter
    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> box_filter;
    box_filter.setMax(maxPoint);
    box_filter.setMin(minPoint);
    box_filter.setInputCloud(cloud_voxel);
    box_filter.filter(*cloud_region);


    // Filter the points above the car
    std::vector<int> car_indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(car_indices);

    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndicesPtr point_indices (new pcl::PointIndices());
    for(int point_index : car_indices) {
        point_indices->indices.push_back(point_index);
    }
    extract.setInputCloud(cloud_region);
    extract.setIndices(point_indices);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacle (new pcl::PointCloud<PointT>());

    // Extract the inliers (plane) from the original cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane);
    std::cout << "PointCloud representing the planar component: " << plane->width * plane->height << " data points." << std::endl;

    // Extract the negative (obstacles) from the cloud
    extract.setNegative (true);
    extract.filter (*obstacle);

    // Return the <obstacle, plane> PCL pair
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	// TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    // Optional setting
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given PCL data." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    srand(time(NULL));

    int num_points = (int) cloud->points.size();
    int i1,i2,i3;
    double a,b,c,d,dist;
    double x1,y1,z1,x2,y2,z2,x3,y3,z3;

    // For max iterations
    for(int i=0; i<maxIterations; ++i) {

        typename pcl::PointIndices::Ptr inliersTemp (new pcl::PointIndices ());

        // Randomly sample subset and fit line
        i1 = rand() % num_points;
        i2 = rand() % num_points;
        i3 = rand() % num_points;

        while (i1 == i2)
            i2 = rand() % num_points;
        while(i2 == i3 || i1==i3)
            i3 = rand() % num_points;

        x1 = cloud->points[i1].x;
        y1 = cloud->points[i1].y;
        z1 = cloud->points[i1].z;
        x2 = cloud->points[i2].x;
        y2 = cloud->points[i2].y;
        z2 = cloud->points[i2].z;
        x3 = cloud->points[i3].x;
        y3 = cloud->points[i3].y;
        z3 = cloud->points[i3].z;

        // Estimate plane
        a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        d = -(a*x1 + b*y1+ c*z1);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for(int n=0; n<cloud->points.size(); ++n) {
            dist = fabs(a*cloud->points[n].x + b*cloud->points[n].y + c*cloud->points[n].z + d) / sqrt(a*a + b*b + c*c);
            if (dist <= distanceThreshold) {
                inliersTemp->indices.push_back(n);
            }
        }
        if(inliersTemp->indices.size() > inliers->indices.size()) {
            inliers = inliersTemp;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation custom took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Build the KdTree datastracture
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    // Extract the cluster indices using the PCL built-in Euclidian search method
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract(clusterIndices);

    // Output just for debugging
    //std::cout << "Found " << clusterIndices.size() << " clusters"  << std::endl;
    //for(auto cluster : clusterIndices) {
    //    std::cout << "Cluster len: " << cluster.indices.size() << std::endl;
    //}

    // Extract the clusters from the original PCL
    // Would be better to write this into internal class method and use it here and in SeparateClouds()
    for(auto cluster : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cluster_pcl (new pcl::PointCloud<PointT>());

        for(int index : cluster.indices) {
            cluster_pcl->push_back(cloud->points[index]);
        }
        cluster_pcl->width = cluster_pcl->points.size();
        cluster_pcl->height = 1;
        cluster_pcl->is_dense = true;
        clusters.push_back(cluster_pcl);
        //std::cout << "PointCloud representing the planar component: " << cluster_pcl->width * cluster_pcl->height << " data points." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}