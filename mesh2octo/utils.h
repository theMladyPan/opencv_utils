#ifndef UTILS_H
#define UTILS_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <chrono>
#include <thread>

#define DENSITY 1

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

pcl::visualization::PCLVisualizer::Ptr simpleVis ();

class minMaxXYZ{
public:
    double Xmin;
    double Xmax;
    double Ymin;
    double Ymax;
    double Zmin;
    double Zmax;
    double height;
    double width;
    double depth;
    minMaxXYZ() = default;
    minMaxXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pPcl);
    void begin(pcl::PointXYZ &point);
    void roundValues();
};

void subsamplePcl(const pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr pSampledPcl);

std::ostream& operator<<(std::ostream& os, const minMaxXYZ& obj);

pcl::PointXYZ getCenterOfPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl);

double average(const std::vector<double>& vec);

void getCenterOfOctree(const pcl::octree::OctreePointCloud<pcl::PointXYZ>::Ptr pOctree, pcl::PointXYZ &centroid);

void showForXSeconds(pcl::visualization::PCLVisualizer::Ptr visualizer, int X);

#endif // UTILS_H
