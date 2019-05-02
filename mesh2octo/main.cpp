#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <string>
#include <chrono>
#include <thread>

#include "utils.h"

using namespace std;
using namespace pcl;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    PolygonMesh mesh;
    string fileName(argv[1]);

    if(fileName.find(".stl") == string::npos && fileName.find(".STL") == string::npos){
        // TODO - toto zaiste dokončiť poriadne
        io::loadPolygonFilePLY(fileName, mesh);
    }else{
        io::loadPolygonFileSTL(fileName, mesh);
    }

    PointCloud<PointXYZ>::Ptr ptrPcl(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr pSampled(new PointCloud<PointXYZ>);
    fromPCLPointCloud2(mesh.cloud, *ptrPcl);
    visualization::PCLVisualizer::Ptr pclVis;

    subsamplePcl(mesh, pSampled);

    /**
     * @brief resolution
     */
    float resolution = atoi(argv[2]);
    float res_2 = resolution/2;

    octree::OctreePointCloud<PointXYZ>::Ptr pOctreePcl(new octree::OctreePointCloud<PointXYZ>(resolution));
    pOctreePcl->setInputCloud(pSampled);
    pOctreePcl->addPointsFromInputCloud();

    double mix, max, miy, may, miz, maz;
    pOctreePcl->getBoundingBox(mix, miy, miz, max, may, maz);
    auto res = pOctreePcl->getResolution();
    auto depth = pOctreePcl->getTreeDepth();
    auto leafCount = pOctreePcl->getLeafCount();

    PointXYZ centroid;
    getCenterOfOctree(pOctreePcl, centroid);
    cout<<"Centroid: "<<centroid<<"\n";

#ifndef NDEBUG
    cout<<"Res: "<<res<<", Depth: "<<depth<<", Leaf Count: "<<leafCount<<"\n";

    vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> voxelCenters;
    pOctreePcl->getOccupiedVoxelCenters(voxelCenters);
    pclVis = simpleVis(ptrPcl);

    for(auto center:voxelCenters){
        pclVis->addCube(Eigen::Vector3f(center.x, center.y, center.z), Eigen::Quaternionf(1,0,0,0), res, res, res, to_string(rand()));
    }

    while(!pclVis->wasStopped()){
        pclVis->spinOnce(30);
        this_thread::sleep_for(33ms);
    };
#endif //NDEBUG
    return EXIT_SUCCESS;
}
