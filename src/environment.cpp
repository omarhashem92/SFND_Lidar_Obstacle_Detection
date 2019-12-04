/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quiz/ransac/Segment.h"
#include "Clustering.h"
//#include "Clustering.cpp"
#define SEG_MAX_ITERATIONS      150
#define SEG_DISTANCE_TOLERANCE  0.2


#define CLU_DISTANCE_TOLERANCE  0.3
#define CLU_MIN_SIZE            20
#define CLU_MAX_SIZE            525


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
    
    // TODO:: Create lidar sensor 
    Lidar *ptrLidar = new Lidar(cars , 0);    //creating object of lidar
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanedPointCloud = ptrLidar->scan();  
    //renderRays(viewer , ptrLidar->position , scanedPointCloud);
    //renderPointCloud(viewer , scanedPointCloud , "Point Cloud");   //rendering point clouds only and to remove the cars set renderScene to false


    // TODO:: Create point processor
    
    ProcessPointClouds <pcl::PointXYZ>*ptrProcessPointClouds = new ProcessPointClouds<pcl::PointXYZ>;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ptrProcessPointClouds->SegmentPlane(scanedPointCloud, 100, 0.2);

    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"roadCloud",Color(0,1,0));
    
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = ptrProcessPointClouds->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      ptrProcessPointClouds->numPoints(cluster);
      //renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      Box box = ptrProcessPointClouds->BoundingBox(cluster);
     // renderBox(viewer,box,clusterId);
      ++clusterId;
    }
}
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  typedef pcl::PointXYZI pointT ;
  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  
  // Filteration
    float filterRes = 0.2f;//The voxel box resultion in meters
    float minX = -7.5f, maxX = 32.0f;//longitudinal view
    float minY = -6.0f, maxY = 7.0f;//lateral view
    float minZ = -2.0f, maxZ = 0.6f;//controls the height of the lidar
    Eigen::Vector4f minPoint(minX,minY,minZ,1);
    Eigen::Vector4f maxPoint(maxX,maxY,maxZ,1);
    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes , minPoint, maxPoint);
    renderPointCloud(viewer,filterCloud,"filterCloud");
  
  
  // Segmentation
    //std::pair<pcl::PointCloud<pointT>::Ptr, pcl::PointCloud<pointT>::Ptr> segmentCloud = Ransac(filterCloud, SEG_MAX_ITERATIONS, SEG_DISTANCE_TOLERANCE);
    //std::pair<pcl::PointCloud<pointT>::Ptr, pcl::PointCloud<pointT>::Ptr> segmentCloud = SegmentPlanes(filterCloud, SEG_MAX_ITERATIONS, SEG_DISTANCE_TOLERANCE );
    
    std::pair<pcl::PointCloud<pointT>::Ptr, pcl::PointCloud<pointT>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, SEG_MAX_ITERATIONS, SEG_DISTANCE_TOLERANCE);

    auto obstCloud = segmentCloud.first;
    auto planeCloud = segmentCloud.second;
  //renderPointCloud(viewer,obstCloud,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,planeCloud,"planeCloud",Color(0,1,0));


   
  //Clustering
    auto cloudCluster = myClustering(segmentCloud.first, CLU_DISTANCE_TOLERANCE, CLU_MIN_SIZE, CLU_MAX_SIZE);
    //auto cloudCluster = pointProcessorI->Clustering(segmentCloud.first, CLU_DISTANCE_TOLERANCE, CLU_MIN_SIZE, CLU_MAX_SIZE);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudCluster)
    {
      std::cout << "cluster size ";
      pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId));
      Box box = pointProcessorI->BoundingBox(cluster);
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;



    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer,pointProcessorI,inputCloudI);

    while (!viewer->wasStopped ())
{

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
        streamIterator = stream.begin();

    viewer->spinOnce ();
}
}