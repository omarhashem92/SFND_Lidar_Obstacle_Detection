
/*#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
*/
#include "Clustering.h"
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"


//template<typename PointT>
//std::vector<typename pcl::PointCloud<PointT>::Ptr> myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> myClustering( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    std::vector<std::vector<float>> points;
	KdTree* kdTree = new KdTree;
    for (int index = 0; index < cloud->points.size(); index++)
    {
        auto point = cloud->points[index];
        std::vector<float> aPoint = {point.x, point.y, point.z/*, point.intensity */};
        kdTree->insert(aPoint,index);
        points.push_back(aPoint);
    }

    std::vector<std::vector<int>> clustersIndices = euclideanCluster(points, kdTree, clusterTolerance);

    for(auto cluster : clustersIndices)
    {
         pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(int point_index : cluster)
  		clusterCloud->points.push_back(cloud->points[point_index]);
        if(cluster.size() >= minSize && cluster.size() <= maxSize)
            clusters.push_back(clusterCloud);

    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    return clusters;

}






void proximity(int idx, std::vector<int> &cluster, std::vector<bool> &isProcessed, const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{
	if(isProcessed[idx] == false)
	{
	isProcessed[idx] = true;
	}
	cluster.push_back(idx);
	auto nearbyPointIndices = tree->search(points[idx],distanceTol);
	for(auto nearbyPointIdx : nearbyPointIndices)
	{
		if(!isProcessed[nearbyPointIdx])
		{
			proximity(nearbyPointIdx, cluster, isProcessed, points, tree, distanceTol);
		}
	}
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> isProcessed(points.size(), false);   //fil constructor
	for(int i = 0; i < points.size(); ++i)
	{
		auto point = points[i];
		if(!isProcessed[i])
		{
			std::vector<int> cluster;
			proximity(i, cluster, isProcessed, points, tree, distanceTol);
			clusters.push_back(cluster);
		}

        
	}

	return clusters;

}