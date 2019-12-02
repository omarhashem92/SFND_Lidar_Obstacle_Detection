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

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::VoxelGrid<PointT> voxGrid;
    voxGrid.setInputCloud (cloud);
    voxGrid.setLeafSize (filterRes, filterRes, filterRes);
    voxGrid.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox< PointT >regionOfInterest;
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.setInputCloud(cloud_filtered);
    regionOfInterest.filter(*cloudRegion);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    //typename pcl::PointCloud<PointT>::Ptr obsPointCloud = new pcl::PointCloud<PointT>();  //check later the reason of error here
    //typename pcl::PointCloud<PointT>::Ptr roadPointCloud = new pcl::PointCloud<PointT>();
    typename pcl::PointCloud<PointT>::Ptr obsPointCloud (new pcl::PointCloud<PointT>), roadPointCloud (new pcl::PointCloud<PointT>);

    for(auto index : inliers->indices)
    {
        roadPointCloud->points.push_back(cloud->points[index]);
    }

     // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*roadPointCloud);
    extract.setNegative (true);
    extract.filter (*obsPointCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsPointCloud, roadPointCloud);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); //Indicies for the Inliners(road line)

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);    //RANSAC is the algorithm for the segmentation. to classify which points are belong to the road and which belong to obstacles.
    seg.setMaxIterations (maxIterations);   //maximum iterations for drawing a line and checking for the points that belong to that line with the min distance.
    seg.setDistanceThreshold (distanceThreshold);   //thickness of the road and any other point that is not belong to the road (line) is considered an obstacle.
    
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      //break;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    //typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    
  for (pcl::PointIndices getIndices : clusterIndices)
  {
       
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new  pcl::PointCloud<PointT>);
        for(int a_point_idx : getIndices.indices)
        {
            PointT point = cloud->points[a_point_idx];
            cloud_cluster->points.push_back(point);
         }
        clusters.push_back(cloud_cluster);
   
    /*
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (int index : getIndices.indices)
      cloud_cluster->points.push_back (cloud_cluster->points[index]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    */
  
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