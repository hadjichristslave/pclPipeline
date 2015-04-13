#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace pcl;

int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ( "../resources/table_scene_lms400.pcd" , *cloud);
    std::cout << " loaded file " << std::endl;

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model ( new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud)); 
    pcl::RandomSampleConsensus<pcl::PointXYZ> sac (model, .03);

    bool result = sac.computeModel ();


    boost::shared_ptr<std::vector<int> > inliers (new std::vector<int>);

    sac.getInliers(* inliers);


    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    std::cout<< "entering visualization " << std::endl;
    for(unsigned int i=0;i<inliers->size(); ++i){
        pcl::PointXYZ basic_point;
        basic_point.x = cloud->points[i].x;
        basic_point.y = cloud->points[i].y;
        basic_point.z = cloud->points[i].z;
        inliers_cloud->points.push_back(basic_point);
    }
//    viewer->spin();
    std::cout << "Found model with " << inliers->size () << " inliers" << std::endl;
    Eigen::VectorXf coeff;
    sac.getModelCoefficients (coeff);
    std::cout << " ,planenormalis: " << coeff[0] << "," << coeff[1] << "," << std::endl;

    //Get the outliers cloud of RANSAC 
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> eifilter (true); // Initializing with true will allow us to extract the removed indices
    eifilter.setInputCloud (cloud);
    eifilter.setIndices (inliers);
    eifilter.setNegative(true);
    eifilter.filter (*cloud_out);

    // Get the convex elements of the surfacefaces

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull ;
    chull.setInputCloud (inliers_cloud);
    chull.reconstruct (* cloud_hull);

    // Extract polygons of the rest of the cloud

    ExtractPolygonalPrismData<PointXYZ> ex ;
    ex.setInputCloud (cloud_out);
    ex.setInputPlanarHull (cloud_hull);
    PointIndices::Ptr output ( new PointIndices);
    ex.segment (* output);

    pcl::PointIndices ind = * output;
    std::cout << ind.indices.size() << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr polygons (new pcl::PointCloud<pcl::PointXYZ>);

    for(unsigned int i=0;i<ind.indices.size(); ++i){
        pcl::PointXYZ basic_point;
        basic_point.x = cloud_out->points[ind.indices[i]].x;
        basic_point.y = cloud_out->points[ind.indices[i]].y;
        basic_point.z = cloud_out->points[ind.indices[i]].z;
        polygons->points.push_back(basic_point);
    }
    viewer->addPointCloud<pcl::PointXYZ> (polygons, "Inlier cloud" );
    viewer->spin();



    return 1;
}
