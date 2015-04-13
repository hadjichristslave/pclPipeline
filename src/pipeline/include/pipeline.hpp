#ifndef PIPELINE_H 
#define PIPELINE_H

// Cloud viewing headers
#include <pcl/visualization/cloud_viewer.h>
// Cloud loading headers
#include <iostream>
// Statistical outlier removal headers
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
// Ransac headers
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
// ICP headers
#include <pcl/registration/icp.h>
#include <Eigen/Geometry> 
#include <Eigen/Core>
// FPFH headers
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace pcl;
class Pipeline 
{
    public:
        inline const void view( const PointCloud< PointXYZRGB >::Ptr cloud , const string name);
        inline const void removeStatisticalOutliers( PointCloud< PointXYZRGB >::Ptr  cloud);
        inline const void planeEstimation( PointCloud< PointXYZRGB >::Ptr cloud);
        inline const void ICPTransform( PointCloud< PointXYZRGB >::Ptr cloud, const  PointCloud< PointXYZRGB >::Ptr target_cloud);
        inline FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33> fpfhEst( const PointCloud< PointXYZRGB >::Ptr cloud);


    protected:
        std::string mName; 

    private:
        //TODO must put those in a config file
        // Config options for the filtes
        static const bool   debug         = true;
        static const double planeCoverage = .3;
        static const int    Neighbors     = 50;
        static const double NeighborDev   = 1.5;
        static const int    MaxIterations = 100;
        static const double DistThreshold = .02;
};
// Remove statistical outliers from cloud
// new cloud overwrites old one
inline const void Pipeline::removeStatisticalOutliers( PointCloud< PointXYZRGB >::Ptr  cloud ){
    PointCloud<PointXYZRGB>::Ptr m_ptrCloud(cloud);
    StatisticalOutlierRemoval<PointXYZRGB> sor;
    sor.setInputCloud (m_ptrCloud);
    sor.setMeanK (Neighbors);
    sor.setStddevMulThresh (NeighborDev);
    sor.filter ( * cloud); 
}
/// Do plane estimation on the cloud.
// New cloud overwrites old
inline const void Pipeline::planeEstimation( PointCloud< PointXYZRGB >::Ptr cloud){
    PointCloud<PointXYZRGB>::Ptr cloud_f (new PointCloud< PointXYZRGB >);
    SACSegmentation<PointXYZRGB> seg;
    PointIndices::Ptr inliers (new PointIndices);
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (MaxIterations);
    seg.setDistanceThreshold (DistThreshold);

    int i=0, nr_points = (int) cloud->points.size ();
    // Find planes until we have covered at least 70% of the cloud surface 
    while (cloud->points.size () > planeCoverage * nr_points){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud ( cloud );
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0){
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the planar inliers from the input cloud
        ExtractIndices<PointXYZRGB> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud = *cloud_f;
    }
}
// Perform ICP on cloud ocmpared to the first one in the batch
// New cloud overwrites old one
inline const void  Pipeline::ICPTransform( PointCloud< PointXYZRGB >::Ptr cloud, const  PointCloud< PointXYZRGB >::Ptr target_cloud ){
    IterativeClosestPoint< PointXYZRGB, PointXYZRGB > icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(target_cloud);
    PointCloud< PointXYZRGB > othercloud;
    icp.align(othercloud);
    copyPointCloud( othercloud, * cloud );
}
inline const void Pipeline::view(const PointCloud< PointXYZRGB >::Ptr cloud, const string name){
    if(debug){
        cout << "Displaying " << name << endl;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->addPointCloud< pcl::PointXYZRGB > ( cloud ,name );
        viewer->spin();
    }
}

// Fast point feature histogram for pointcloud cloud
 inline FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33>  Pipeline::fpfhEst( const PointCloud< PointXYZRGB>::Ptr cloud){
       
    PointCloud< PointNormal >::Ptr normals (new PointCloud< PointNormal > );
    pcl::search::KdTree< PointXYZRGB >::Ptr tree (new pcl::search::KdTree< PointXYZRGB >);

    pcl::copyPointCloud (*cloud, *normals);

    pcl::NormalEstimation< PointXYZRGB, PointNormal > ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (.03);
    ne.compute (*normals);

    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);

    fpfh.setSearchMethod (tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (0.05);
    fpfh.compute (*fpfhs);

    return fpfh;
}
#endif 
