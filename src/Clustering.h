
#ifndef CLUSTER_H_
#define CLUSTER_H_
//#include <chrono>
//#include <string>
#include "kdtree.h"


//template<typename PointT>
//std::vector<typename pcl::PointCloud<PointT>::Ptr> myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> myClustering( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

void proximity(int idx, std::vector<int> &cluster, std::vector<bool> &isProcessed, const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol);
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);





#endif