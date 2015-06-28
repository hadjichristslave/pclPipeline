#ifndef PIPELINE_H 
#define PIPELINE_H


#include <typeinfo>
// Cloud viewing headers
#include <pcl/visualization/cloud_viewer.h>
// Cloud loading headers
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h> 
// Statistical outlier removal headers
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
// Ransac headers
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
//Downsampling headers
#include <pcl/filters/voxel_grid.h>
// ICP headers
#include <pcl/registration/icp.h>
#include <Eigen/Geometry> 
#include <Eigen/Core>
// FPFH headers
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/opencv.hpp>
// Nearest search
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;

class Pipeline
{
    public:
         Pipeline();
         const void view( const PointCloud< PointXYZRGB >::Ptr cloud , const string name);
         const void removeStatisticalOutliers( PointCloud< PointXYZRGB >::Ptr  cloud);
         const void planeEstimation( PointCloud< PointXYZRGB >::Ptr cloud);
         const void downsample( PointCloud< PointXYZRGB >::Ptr cloud, double leafSize);
         const void ICPTransform( PointCloud< PointXYZRGB >::Ptr cloud, const  PointCloud< PointXYZRGB >::Ptr target_cloud);
         vector< vector< float > > fpfhEst( const PointCloud< PointXYZRGB >::Ptr cloud);
         vector< vector< int >  > colourInformationExtractor( const PointCloud< PointXYZRGB >::Ptr cloud);
         std::vector<float> histogramCompare( float histA[33], float histB[33]);
         int getBin(int DiscR, int DiscG, int DiscB);
         int getColourBins(int r , int g , int b);
         int getBinIndex(int r);
         float KLDivergence( cv::Mat *  mat1, cv::Mat  * mat2);
         void normalizeVec( vector < int > inputVec);

    private:
        // Config options for the filtes
        static const bool   debug         = true;
        //stat outlier removal
        static const int    Neighbors     = 25;
        static const double NeighborDev   = .1;
        //Plane estimation
        static const double planeCoverage = .3;
        static const int    MaxIterations = 25;
        static const double DistThreshold = .005;
        //KD tree search
        static const int    K             = 26;
        // Discretization of colours. each part of RGB spectrum will be discretized into colourBins parts
        static const int    colourBins    = 3;
        static const int    RGBMIN        = 0;
        static const int    RGBMAX        = 255;
        vector<int> bins;
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        static const int    signatureLength = 33;
        //fpfh
        static const double neRadiusSearch   = 0.02;
        static const double fpRadSearch      = 0.04;

};
 Pipeline::Pipeline(void){
    int step = (int)(RGBMAX - RGBMIN)/colourBins;
    int curStep =step;
    for( int i=1;i<colourBins;i++, curStep+=step)
        bins.push_back( curStep);
    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);
}
 const void Pipeline::removeStatisticalOutliers( PointCloud< PointXYZRGB >::Ptr  cloud ){
    StatisticalOutlierRemoval<PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (Neighbors);
    sor.setStddevMulThresh (NeighborDev);
    sor.filter ( * cloud); 
}
 const void Pipeline::downsample( PointCloud< PointXYZRGB >::Ptr cloud, double leafSize){
    pcl::VoxelGrid<PointXYZRGB> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (leafSize, leafSize, leafSize);
    vg.setDownsampleAllData (false);
    vg.filter (*cloud); 
}
 const void Pipeline::planeEstimation( PointCloud< PointXYZRGB >::Ptr cloud){
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
 const void  Pipeline::ICPTransform( PointCloud< PointXYZRGB >::Ptr cloud, const  PointCloud< PointXYZRGB >::Ptr target_cloud ){
    IterativeClosestPoint< PointXYZRGB, PointXYZRGB > icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(target_cloud);
    PointCloud< PointXYZRGB > othercloud;
    icp.align(othercloud);
    copyPointCloud(othercloud,*cloud);
}
 const void Pipeline::view(const PointCloud< PointXYZRGB >::Ptr cloud, const string name){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud< pcl::PointXYZRGB > (cloud,name);
    viewer->spin();
}
// Fast point feature histogram for pointcloud cloud
 vector< vector< float >  >  Pipeline::fpfhEst( const PointCloud< PointXYZRGB>::Ptr cloud){
    vector< vector <  float > > pointRelations;
    PointCloud< PointNormal >::Ptr normals (new PointCloud< PointNormal > );
    pcl::search::KdTree< PointXYZRGB >::Ptr tree (new pcl::search::KdTree< PointXYZRGB >);
    pcl::copyPointCloud (*cloud,*normals);
    pcl::NormalEstimation< PointXYZRGB, PointNormal > ne;
    ne.setInputCloud (cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(neRadiusSearch);
    ne.compute (*normals);
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(10);
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (fpRadSearch);
    fpfh.compute (*fpfhs);
    kdtree.setInputCloud (cloud);
    for(int i=0;i< cloud->points.size();i++){
        pcl::PointXYZRGB searchPoint = cloud->points[i];
        float currentHist[signatureLength];
        for (int o=0;o<signatureLength;o++) currentHist[o] = fpfhs->points[i].histogram[o];
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j){
                float neighborHist[signatureLength];
                for(int o=0;o<signatureLength;o++) neighborHist[o] = fpfhs->points[ pointIdxNKNSearch[j]].histogram[o];
                pointRelations.push_back(histogramCompare(currentHist, neighborHist));
            }
    }
    return pointRelations;
}
 vector< vector<int> >  Pipeline::colourInformationExtractor( const PointCloud< PointXYZRGB >::Ptr cloud){
    // Init vars
    vector< vector< int > > colourDistributions;
    colourDistributions.resize( cloud->points.size());
    for(int i=0;i<cloud->points.size();i++)
        colourDistributions[i].resize(colourBins*colourBins*colourBins,0); 
    kdtree.setInputCloud (cloud);
    for(int i=0;i< cloud->points.size();i++){
        pcl::PointXYZRGB searchPoint = cloud->points[i];
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j){
                PointXYZRGB  p = cloud->points[ pointIdxNKNSearch[j] ];
                colourDistributions[i][getColourBins( (int)p.r , (int)p.g, (int)p.b)]++;
            }
    }
    return colourDistributions;
}
 int Pipeline::getBin(int DiscR, int DiscG, int DiscB){ 
    return colourBins*colourBins*DiscR + colourBins*DiscG + DiscB;
}
 int Pipeline::getBinIndex(int r){
    for(int i=0;i<bins.size();i++)
        if( r<bins[i]) return i;
    return bins.size();
}
 int Pipeline::getColourBins(int r, int g, int b){
    int rbin = getBinIndex(r);
    int gbin = getBinIndex(g);
    int bbin = getBinIndex(b);
    return getBin(rbin , gbin , bbin);
}
 std::vector<float>  Pipeline::histogramCompare( float histA[signatureLength], float histB[signatureLength]){
    const int N = sizeof(histA) / sizeof(int);
    float maxA = *std::max_element(histA, histA+N);
    float maxB = *std::max_element(histB, histB+N);
    cv::Mat M1 = cv::Mat(1,signatureLength, cv::DataType<float>::type , histA);
    cv::Mat M2 = cv::Mat(1,signatureLength, cv::DataType<float>::type , histB);
    int histSize = signatureLength;
    float rangeA[] = {0, maxA+1};//ranges are exclusive hence + 1
    float rangeB[] = {0, maxB+1};// see above
    const float* histRangeA = {rangeA};
    const float* histRangeB = {rangeB};
    bool uniform = true;
    bool accumulate = false;
    cv::Mat a1_hist, a2_hist;
    // normalization means SQRT( sum(component*component)) = 1A
    cv::calcHist(&M1, 1, 0, cv::Mat(), a1_hist, 1, &histSize, &histRangeA, uniform, accumulate );
    cv::calcHist(&M2, 1, 0, cv::Mat(), a2_hist, 1, &histSize, &histRangeB, uniform, accumulate );
    normalize(a1_hist, a1_hist,  0, 1, CV_MINMAX);
    normalize(a2_hist, a2_hist,  0, 1, CV_MINMAX);
    cv::Mat sig1(signatureLength ,2, cv::DataType<float>::type);  
    cv::Mat sig2(signatureLength ,2, cv::DataType<float>::type); 
    for(int i=0;i<histSize;i++){
        float binval = a1_hist.at<float>(i);
        sig1.at< float >(i, 0) = binval;
        sig1.at< float >(i, 1) = i;
        float binval2 = a2_hist.at< float>(i);
        sig2.at< float >(i, 0) = binval2;
        sig2.at< float >(i, 1) = i;
    }
    float emd           = cv::EMD(sig1, sig2, CV_DIST_L2);
    float compar_hell   = (float)cv::compareHist(a1_hist, a2_hist, CV_COMP_HELLINGER );
    // WARNING!!! KLDivergence changegs the values of the histograms so it should be called last.
    float kld =  KLDivergence( &a1_hist, &a2_hist);
    
    vector<float> distances;
    distances.push_back(kld); 
    distances.push_back(emd); 
    distances.push_back(compar_hell);
    return distances;
}
// Two normalized vectors of the same size
 float Pipeline::KLDivergence( cv::Mat * mat1, cv::Mat * mat2){
    float sum1 = 0,sum2 = 0;
    for(int i=0;i<mat1->rows;i++){
       sum1 += mat1->at<float>(i,0);
       sum2 += mat2->at<float>(i,0);
    }
    for(int i=0;i<mat1->rows;i++){
        mat1->at<float>(i,0) /= sum1;
        mat2->at<float>(i,0) /= sum2;
    }
    float result = 0.;
    for(int i=0;i< mat1->rows;i++)
        if(  mat1->at<float>(0,i) !=0 ){
            float ratio = mat1->at<float>(i,0)/ mat2->at<float>(0,i);
            if(ratio>0 && ratio  != std::numeric_limits<float>::infinity() )
                result += mat1->at<float>(i,0) * log(ratio); 
        }
    return result;
}
 void Pipeline::normalizeVec( vector< int > inputVec){
   float sum = 0;
   for(int i = 0; i < inputVec.size(); i++) sum+= inputVec[i]; 
   float normalized;
   for(int i=0;i< inputVec.size(); i++){
       normalized = (float)inputVec[i]/sum;
   }
