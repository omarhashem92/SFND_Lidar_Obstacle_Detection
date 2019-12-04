#ifndef RANSAC_H_
#define RANSAC_H_

#include <unordered_set>


std::vector<float> findLineCoefficents(float x1, float y1, float x2, float y2);
std::vector<float> findLineCoefficentsPlane(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3);

float getPointDistance(float a, float b, float c, float x, float y);
float getPointDistancePlane(float a, float b, float c, float d, float x, float y, float z);

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData();
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D();
pcl::visualization::PCLVisualizer::Ptr initScene();


std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol);
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol);





#endif 
