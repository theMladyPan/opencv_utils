#include "utils.h"

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr simpleVis (){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

void minMaxXYZ::begin(pcl::PointXYZ &point) {
    Xmax = point.x;
    Xmin = point.x;
    Ymax = point.y;
    Ymin = point.y;
    Zmax = point.z;
    Zmin = point.z;
}

minMaxXYZ::minMaxXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr pPcl) {
    auto point = pPcl->points.front();
    Xmax = point.x;
    Xmin = point.x;
    Ymax = point.y;
    Ymin = point.y;
    Zmax = point.z;
    Zmin = point.z;

    for(pcl::PointXYZ point:pPcl->points){
        double x = point.x;
        double y = point.y;
        double z = point.z;

        if(x < Xmin){
            Xmin = x;
        }else if(x > Xmax){
            Xmax = x;
        }
        if(y < Ymin){
            Ymin = y;
        }else if(y > Ymax){
            Ymax = y;
        }
        if(z < Zmin){
            Zmin = z;
        }else if(z > Zmax){
            Zmax = z;
        }
    }
    height = Xmax - Xmin;
    width = Ymax - Ymin;
    depth = Zmax - Zmin;
}

void minMaxXYZ::roundValues() {
    using std::round;
    Xmax = round(Xmax);
    Xmin = round(Xmin);
    Ymax = round(Ymax);
    Ymin = round(Ymin);
    Zmax = round(Zmax);
    Zmin = round(Zmin);
}

std::ostream& operator<<(std::ostream& os, const minMaxXYZ& obj){
    os << "Xmin: " << obj.Xmin << ", Xmax: " << obj.Xmax << ", Ymin: " << obj.Ymin << ", Ymax: " << obj.Ymax << ", Zmin: " << obj.Zmin << ", Zmax: " << obj.Zmax;
    return os;
}

double average(const std::vector<double> &vec){
    double sum = 0;
    int n = 0;
    for(auto elem:vec){
        sum += elem;
        n++;
    }
    return sum/n;
}

pcl::PointXYZ getCenterOfPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pInPcl) {
    std::vector<double> vX;
    std::vector<double> vY;
    std::vector<double> vZ;
    pcl::PointXYZ centroid;

    for(pcl::PointXYZ point:*pInPcl){
        vX.push_back(point.x);
        vY.push_back(point.y);
        vZ.push_back(point.z);
    }
    centroid.x = average(vX);
    centroid.y = average(vY);
    centroid.z = average(vZ);

    return centroid;
}

void swap(pcl::PointXYZ &point1, pcl::PointXYZ &point2){
    pcl::PointXYZ temp(point1);
    point1 = point2;
    point2 = temp;
}

pcl::PointXYZ operator- (const pcl::PointXYZ &lh, const pcl::PointXYZ &rh){
    pcl::PointXYZ sub;
    sub.x = lh.x - rh.x;
    sub.y = lh.y - rh.y;
    sub.z = lh.z - rh.z;
    return sub;
}

pcl::PointXYZ operator+ (const pcl::PointXYZ &lh, const pcl::PointXYZ &rh){
    pcl::PointXYZ add;
    add.x = lh.x + rh.x;
    add.y = lh.y + rh.y;
    add.z = lh.z + rh.z;
    return add;
}

double absLen(const pcl::PointXYZ &vec){
    return pow( ((vec.x*vec.x) + (vec.y*vec.y) + (vec.z*vec.z)), 0.5f);
}

pcl::PointXYZ operator* (const pcl::PointXYZ &lh, const double &rh){
    pcl::PointXYZ mul;
    mul.x = lh.x * rh;
    mul.y = lh.y * rh;
    mul.z = lh.z * rh;
    return mul;
}

pcl::PointXYZ operator/ (const pcl::PointXYZ &lh, const double &rh){
    pcl::PointXYZ mul;
    mul.x = lh.x / rh;
    mul.y = lh.y / rh;
    mul.z = lh.z / rh;
    return mul;
}

void subsampleVector(const unsigned int &indice1, const unsigned int &indice2, pcl::PointCloud<pcl::PointXYZ> &originalPcl, pcl::PointCloud<pcl::PointXYZ>::Ptr pSampledPcl){
    pcl::PointXYZ point1 = originalPcl.at(indice1);
    pcl::PointXYZ point2 = originalPcl.at(indice2);
    if(point1.z > point2.z){
        swap(point1, point2);
    }
    // point2 now has higher Z coordinate
    pcl::PointXYZ R = point2 - point1;

    // TODO let it be flexible precision
    double lenVec = absLen(R);
    if(lenVec > 2*DENSITY){
        // vector is long, needs subsambling:
        for(int n=0; n<lenVec; n++){
            pcl::PointXYZ An = point1 + ((R / lenVec) * n);
            pSampledPcl->push_back(An);
        }
    }
    pSampledPcl->push_back(point1);
    pSampledPcl->push_back(point2);
}

void subsamplePcl(const pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr pSampledPcl) {
    pcl::PointCloud<pcl::PointXYZ> originalPcl;
    pcl::fromPCLPointCloud2(mesh.cloud, originalPcl);
    auto polygons = mesh.polygons; // = vector<Vertices> = vector<vector<unsigned int>>
    for(auto polygon:polygons){
        subsampleVector(polygon.vertices.at(0), polygon.vertices.at(1), originalPcl, pSampledPcl);
        subsampleVector(polygon.vertices.at(1), polygon.vertices.at(2), originalPcl, pSampledPcl);
        subsampleVector(polygon.vertices.at(0), polygon.vertices.at(2), originalPcl, pSampledPcl);
    }

}


void showForXSeconds(pcl::visualization::PCLVisualizer::Ptr visualizer, int X){
    using namespace std::chrono_literals;
    for(int i=0;i<X*16;i++){
        visualizer->spinOnce(30);
        std::this_thread::sleep_for(30ms);
    }
}

void getCenterOfOctree(const pcl::octree::OctreePointCloud<pcl::PointXYZ>::Ptr pOctree, pcl::PointXYZ &centroid){
    std::vector<double> vX;
    std::vector<double> vY;
    std::vector<double> vZ;

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxelCenters;
    pOctree->getOccupiedVoxelCenters(voxelCenters);

    for(pcl::PointXYZ point:voxelCenters){
        vX.push_back(point.x);
        vY.push_back(point.y);
        vZ.push_back(point.z);
    }
    centroid.x = average(vX);
    centroid.y = average(vY);
    centroid.z = average(vZ);
}

void iterateNodes(pcl::octree::OctreePointCloud<pcl::PointXYZ>::Ptr pOctree){
    uint curDepth;
    std::vector<int> indices;

    for(auto nodeIt = pOctree->depth_begin(); nodeIt!= pOctree->depth_end(); nodeIt++){
        if(nodeIt.isLeafNode()){
            curDepth = nodeIt.getCurrentOctreeDepth();
            auto curContainer = nodeIt.getLeafContainer();
            indices.clear();
            curContainer.getPointIndices(indices);
            auto index = curContainer.getPointIndex();
            std::cout<<index<<"\n";
            std::cout<<curDepth;

        }
    }
}
