#include "Segment.h"
#include "ransac.h"
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



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlanes(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
{


    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceThreshold);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      //break;
    }

    typename pcl::PointCloud<PointT>::Ptr postInLiers;
    typename pcl::PointCloud<PointT>::Ptr outLiers;



    for(idx = 0; idx <cloud->points.size() ; idx++)
    {
        PointT point = cloud->points[idx];
		if(inliers.count(idx))
        {
			postInLiers->points.push_back(idx);
        }
		else
        {
			outLiers->points.push_back(idx);
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult.first = postInLiers;
    segResult.second = outLiers;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;

}
