/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Lesson 1: Intro
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data = lidar->scan();
    //renderRays(viewer, lidar->position, pcl_data); // Render the LIDAR rays
    //renderPointCloud(viewer,pcl_data,"rendered pcl"); // Render the genereated PCL data

    ProcessPointClouds<pcl::PointXYZ>* pcl_processor = new ProcessPointClouds<pcl::PointXYZ>();

    // Lesson 2: Segmentation
    // Segment the LIDAR PCL data into ground and obstacle
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pcl_processor->SegmentPlane(pcl_data, 100, 0.3);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud2 = pcl_processor->SegmentPlaneCustom(pcl_data, 100, 0.3);
    //renderPointCloud(viewer,segmentCloud2.first,"obstacleCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud2.second,"planeCloud",Color(0,1,0));

    // Lesson 3 Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pcl_processor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        // Render the cluster
        std::cout << "cluster size ";
        pcl_processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // Calculate the bounding box and render it
        Box box = pcl_processor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
  
}

// Lesson 4
void perceptionPipeline(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pcl_processor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_data) {

    // Filter the PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pcl_processor->FilterCloud(pcl_data, 0.2 , Eigen::Vector4f (-10.0, -5.0, -3.0, 1), Eigen::Vector4f ( 30.0, 5.0, 5.0, 1));

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    // Create a PCL processor
    ProcessPointClouds<pcl::PointXYZI>* pcl_processor = new ProcessPointClouds<pcl::PointXYZI>();

    // Load the PCL data from file
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_data = pcl_processor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Run the perception pipeline
    perceptionPipeline(viewer, pcl_processor, pcl_data);

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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer); // lessons 1-3
    //cityBlock(viewer);

    // PCL Streaming
    ProcessPointClouds<pcl::PointXYZI>* pcl_processor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pcl_processor->streamPcd("../src/sensors/data/pcd/data_2");
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
        perceptionPipeline(viewer, pcl_processor, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        // End for PCL streaming

        viewer->spinOnce ();
    } 
}