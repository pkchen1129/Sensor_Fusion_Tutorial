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
    // --------------------s--------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // We'll initiate it on the heap
    Lidar* lidar = new Lidar(cars , 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud,"inputcloud");
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    // For segmentation 
    std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    // renderPointCloud(viewer, SegmentCloud.first, "Obstcloud", Color(0,1,0)); //Red
    renderPointCloud(viewer, SegmentCloud.second, "Planecloud", Color(0,1,0)); //Green


    //For Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(SegmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){ //Iterate through the cloudClusters(Different cluster)
        std::cout << "Cluster Size";
        pointProcessor.numPoints(cluster);//Print the size of the cloud
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        //Generating bounding box
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI,pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
    
    //For calling one .pcd only
    /*
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    */

    inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (30,8,1,1));
    // Segmentation for the plane(road)
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(inputCloud, 25, 0.3);
    renderPointCloud(viewer, segmentCloud.second, "Planecloud", Color(0,1,0)); //Green

    
    // Clustering for different cars
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.53, 10, 500); //segmentcloud.first would be the obstacle
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){ //Iterate through the cloudClusters(Different cluster)
        std::cout << "Cluster Size";
        pointProcessorI.numPoints(cluster);//Print the size of the cloud
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        //Generating bounding box
        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    // renderPointCloud(viewer,filterCloud,"inputCloud");
    // renderPointCloud(viewer,inputCloud,"filterCloud");

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

    // For Stream PCD
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI; //Create on stack
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // simpleHighway(viewer);
    while (!viewer->wasStopped ())
    {
        //Clear Viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load pcd and run obstacle detectioni process
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();

    } 
}