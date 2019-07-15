/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult, inliersTemp;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int num_points = (int) cloud->points.size();
	int i1,i2, res_i1, res_i2;
	double a,b,c,dist;

	// For max iterations
	for(int i=0; i<maxIterations; ++i) {

	    inliersTemp.clear();

        // Randomly sample subset and fit line
        i1 = rand() % num_points;
        i2 = rand() % num_points;
        while (i1 == i2)
            i2 = rand() % num_points;
        std::cout << "Random variables " << i1 << " " << i2 << std::endl;

        // Estimate line
        a = cloud->points[i1].y - cloud->points[i2].y;
        b = cloud->points[i2].x - cloud->points[i1].x;
        c = cloud->points[i1].x*cloud->points[i2].y - cloud->points[i2].x*cloud->points[i1].y;

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for(int n=0; n<cloud->points.size(); ++n) {
            dist = fabs(a*cloud->points[n].x + b*cloud->points[n].y + c) / sqrt(a*a + b*b);
            if (dist < distanceTol) {
                inliersTemp.insert(n);
            }
        }

        if(inliersTemp.size() > inliersResult.size()) {
            inliersResult = inliersTemp;
            std::cout << "New winner: " << i << std::endl;
        }

	}

	// Return indicies of inliers from fitted line with most inliers

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Custom RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult, inliersTemp;
    srand(time(NULL));

    // TODO: Fill in this function
    int num_points = (int) cloud->points.size();
    int i1,i2,i3;
    double a,b,c,d,dist;
    double x1,y1,z1,x2,y2,z2,x3,y3,z3;

    // For max iterations
    for(int i=0; i<maxIterations; ++i) {

        inliersTemp.clear();

        // Randomly sample subset and fit line
        i1 = rand() % num_points;
        i2 = rand() % num_points;
        i3 = rand() % num_points;
        while (i1 == i2)
            i2 = rand() % num_points;
        while(i2 == i3 || i1==i3)
            i3 = rand() % num_points;

        std::cout << "Random variables " << i1 << " " << i2 << " " << i3  << std::endl;
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
        b = (z2-z1)*(x2-x1) - (x2-x1)*(z3-z1);
        c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        d = -(a*x1 + b*y1+ c*z1);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for(int n=0; n<cloud->points.size(); ++n) {
            dist = fabs(a*cloud->points[n].x + b*cloud->points[n].y + c*cloud->points[n].z + d) / sqrt(a*a + b*b + c*c);
            //std:cout << dist << std::endl;
            if (dist < distanceTol) {
                inliersTemp.insert(n);
            }
        }

        if(inliersTemp.size() > inliersResult.size()) {
            inliersResult = inliersTemp;
            //std::cout << "New winner: " << i << std::endl;
        }

    }

    // Return indicies of inliers from fitted line with most inliers

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Custom RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
