#ifndef SEGMENT_H_
#define SEGMENT_H_

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlanes(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);



#endif
