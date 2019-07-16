/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

namespace lidarPerception {

    void perceptionPipeline(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pcl_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_data) {

        // Filter the PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pcl_processor->FilterCloud(pcl_data, 0.2 , Eigen::Vector4f (-5.0, -6.0, -3.0, 1), Eigen::Vector4f ( 20.0, 6.0, 5.0, 1));

        // Segment the street from the remainder
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pcl_processor->SegmentPlane(filterCloud, 100, 0.2);

        // Cluster the objects
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pcl_processor->Clustering(segmentedCloud.first, 0.5, 50, 1000);

        // Render the PCL
        //renderPointCloud(viewer,filterCloud,"inputCloud");    // Render filter result
        //renderPointCloud(viewer,segmentedCloud.first,"inputCloud", Color(1,0,0)); // Render Segmentation
        renderPointCloud(viewer,segmentedCloud.second,"streetCloud", Color(0,1,0));

        // Render the clusters
        int clusterId = 0;
        std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
        {
            // Render the cluster
            //std::cout << "cluster size ";
            //pcl_processor->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

            // Calculate the bounding box and render it
            Box box = pcl_processor->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
            ++clusterId;
        }
    }


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
    void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
    {

        viewer->setBackgroundColor (0, 0, 0);

        // set camera position and angle
        viewer->initCameraParameters();
        // distance away in meters
        int distance = 16;

        switch(setAngle)
        {
            case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
            case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
            case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
            case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
        }

        if(setAngle!=FPS)
            viewer->addCoordinateSystem (1.0);
    }
}



int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    lidarPerception::initCamera(setAngle, viewer);

    // PCL Streaming
    ProcessPointClouds<pcl::PointXYZI>* pcl_processor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pcl_processor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // For PCL Streaming
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pcl_processor->loadPcd((*streamIterator).string());
        lidarPerception::perceptionPipeline(viewer, pcl_processor, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        // End for PCL streaming

        viewer->spinOnce ();
    } 
}