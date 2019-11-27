/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"



//line coefficents are a, b, c
//point to compare the distance with the line is defined by: (x, y)


std::vector<float> findLineCoefficents(float x1, float y1, float x2, float y2)
{
	//line equation Ax + By + C = 0
	//line equation (y1 - y2)x + (x2 - x1)y + (x1 * y2 - x2 * y1) = 0
	float a = y1 - y2;
	float b = x2 - x1;
	float c = x1 * y2 - x2 * y1;
	std::vector<float> lineCoefficents = {a, b, c};
	return lineCoefficents;
}

float getPointDistance(float a, float b, float c, float x, float y)
{
	//apply the equation: d = |Ax + By + C| / sqrt(A^2 + B^2)
	float d = fabs(a * x + b * y + c) / sqrt(a * a + b * b);
	return d;
}

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
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int point1Index, point2Index, maxCountPointsOnLine = 0, bestpoint1Index = 0, bestpoint2Index = 0;
	// For max iterations 
	for (auto index = 0 ; index < maxIterations ; index++ )
	{
	// Randomly sample subset and fit line
	point1Index = rand() % cloud->points.size();
	point2Index = point1Index;

	pcl::PointXYZ point1 = cloud->points[point1Index];
	pcl::PointXYZ point2 = cloud->points[point2Index];

	std::vector<float> lineCoeffs = findLineCoefficents(point1.x, point1.y, point2.x, point2.y);

	float a = lineCoeffs[0];
	float b = lineCoeffs[1];
	float c = lineCoeffs[2];

	int countPointsOnLine = 0;

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	for(int i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ aPoint = cloud->points[i];
		float x = aPoint.x, y = aPoint.y;
		float dist = getPointDistance(a, b, c, x, y);
		if(dist < distanceTol)
		{
				countPointsOnLine++;
		}
	}
	if(countPointsOnLine > maxCountPointsOnLine)
	{
		maxCountPointsOnLine = countPointsOnLine;
		bestpoint1Index = point1Index;
		bestpoint2Index = point2Index;
	}

	}
	// Return indicies of inliers from fitted line with most inliers
	pcl::PointXYZ bestPoint1 = cloud->points[bestpoint1Index];
	pcl::PointXYZ bestPoint2 = cloud->points[bestpoint2Index];
	std::vector<float> bestLineCoeffs = findLineCoefficents(bestPoint1.x, bestPoint1.y,bestPoint2.x, bestPoint2.y);
	float bestA = bestLineCoeffs[0];
	float bestB = bestLineCoeffs[1];
	float bestC = bestLineCoeffs[2];

	for(int i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ aPoint = cloud->points[i];
		float x = aPoint.x, y = aPoint.y;
		float dist = getPointDistance(bestA, bestB, bestC, x, y);
		if(dist < distanceTol)
		{
			inliersResult.insert(i);
		}
	}


	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 0, 0);

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
