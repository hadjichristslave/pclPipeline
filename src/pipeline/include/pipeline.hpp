#ifndef PIPELINE_H 
#define PIPELINE_H


#include <typeinfo>
// Cloud viewing headers
#include <pcl/visualization/cloud_viewer.h>
// Cloud loading headers
#include <iostream>
#include <algorithm>
#include <vector>
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
        inline Pipeline();
        inline const void view( const PointCloud< PointXYZRGB >::Ptr cloud , const string name);
        inline const void removeStatisticalOutliers( PointCloud< PointXYZRGB >::Ptr  cloud);
        inline const void planeEstimation( PointCloud< PointXYZRGB >::Ptr cloud);
        inline const void downsample( PointCloud< PointXYZRGB >::Ptr cloud, double leafSize);
        inline const void ICPTransform( PointCloud< PointXYZRGB >::Ptr cloud, const  PointCloud< PointXYZRGB >::Ptr target_cloud);
        inline vector< FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33>  > fpfhEst( const PointCloud< PointXYZRGB >::Ptr cloud);
        inline vector< vector< int >  > colourInformationExtractor( const PointCloud< PointXYZRGB >::Ptr cloud);
        inline double histogramCompare( float histA[33], float histB[33]);
        inline int getBin(int DiscR, int DiscG, int DiscB);
        inline int getColourBins(int r , int g , int b);
        inline int getBinIndex(int r);

    private:
        // Config options for the filtes
        static const bool   debug         = true;
        //stat outlier removal
        static const int    Neighbors     = 50;
        static const double NeighborDev   = 1.5;
        //Plane estimation
        static const double planeCoverage = .3;
        static const int    MaxIterations = 100;
        static const double DistThreshold = .02;
        //KD tree search
        static const int    K             = 10;
        // Discretization of colours. each part of RGB spectrum will be discretized into colourBins parts
        // Any value above creates too sparse a matrix to be usefull
        static const int    colourBins    = 4;
        static const int    RGBMIN        = 0;
        static const int    RGBMAX        = 255;
        vector<int> bins;
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
};

inline Pipeline::Pipeline(void){
    int step = (int)(RGBMAX - RGBMIN)/colourBins;
    int curStep =step;
    for( int i=1;i<colourBins;i++, curStep+=step)
        bins.push_back( curStep);



    pointIdxNKNSearch.resize(K);
    pointNKNSquaredDistance.resize(K);


}

// Remove statistical outliers from cloud

inline const void Pipeline::removeStatisticalOutliers( PointCloud< PointXYZRGB >::Ptr  cloud ){
    PointCloud<PointXYZRGB>::Ptr m_ptrCloud(cloud);
    StatisticalOutlierRemoval<PointXYZRGB> sor;
    sor.setInputCloud (m_ptrCloud);
    sor.setMeanK (Neighbors);
    sor.setStddevMulThresh (NeighborDev);
    sor.filter ( * cloud); 
}
// Downsample using voxel grid downsampling 
// Leafzise around .1 is ok
inline const void Pipeline::downsample( PointCloud< PointXYZRGB >::Ptr cloud, double leafSize){
    cout << "before "  << cloud->points.size() << " points"<< endl;
    pcl::VoxelGrid<PointXYZRGB> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (leafSize, leafSize, leafSize);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud); 
    cout << cloud->points.size () << " points\n";
}
/// Do plane estimation on the cloud.
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
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addPointCloud< pcl::PointXYZRGB > ( cloud ,name );
    viewer->spin();
}

// Fast point feature histogram for pointcloud cloud
inline vector < FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33> >  Pipeline::fpfhEst( const PointCloud< PointXYZRGB>::Ptr cloud){

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

    kdtree.setInputCloud (cloud);
    for(int i=0;i< cloud->points.size();i++){
        pcl::PointXYZRGB searchPoint = cloud->points[i];
        float currentHist[33];
        for (int o=0;o<33;o++)
            currentHist[o] = fpfhs->points[i].histogram[o];
//        cout << " comparing histogram : "  << fpfhs->points[i] << endl << endl;
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j){
                if (j==i)
                    continue;
//                cout << " i is : "  << i << " j is " << j << endl << endl;
                float neighborHist[33];
//                cout<< " with histogram : " << fpfhs->points[ pointIdxNKNSearch[j]] << endl <<  endl;
                for(int o=0;o<33;o++)
                    neighborHist[o] = fpfhs->points[ pointIdxNKNSearch[j]].histogram[o];
                histogramCompare(currentHist, neighborHist);
            }
    }

    vector < FPFHEstimation< PointXYZRGB, PointNormal, FPFHSignature33 > > vec;
    return vec;
    //return fpfh;
}
// Extract colour information from the neighbors of every pixel
inline vector< vector<int> >  Pipeline::colourInformationExtractor( const PointCloud< PointXYZRGB >::Ptr cloud){
    // Init vars
    vector< vector< int > > colourDistributions;
    colourDistributions.resize( cloud->points.size());
    for(int i=0;i<cloud->points.size();i++)
        colourDistributions[i].resize(colourBins*colourBins*colourBins,0); 


    //Find NN's. It must be noted that the 10 Nearest neighbors might be light years away
    // Positionsal(XYZ) as well as ANGULAR(FPFH) and even COLOUR information will diverge from close Neighbors have
    // and that will cause the process to cluster the points in different clusters
    kdtree.setInputCloud (cloud);

    for(int i=0;i< cloud->points.size();i++){
        pcl::PointXYZRGB searchPoint = cloud->points[i];
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j){
                PointXYZRGB  p = cloud->points[ pointIdxNKNSearch[j] ];
                colourDistributions[i][getColourBins( (int)p.r , (int)p.g, (int)p.b)]++;
            }
        }
    }
    return colourDistributions;
}

// Get the bin given the discretized RGB values 
// More info on http://en.wikipedia.org/wiki/Color_histogram
inline int Pipeline::getBin(int DiscR, int DiscG, int DiscB){ 
    return colourBins*colourBins*DiscR + colourBins*DiscG + DiscB;
}
inline int Pipeline::getBinIndex(int r){
    for(int i=0;i<bins.size();i++)
        if( r<bins[i]) return i;
    return bins.size()-1;
}
// The number of colourBins defines in how many values the rgb spectrum will be divided.
// The idea is to discretize the spectrum and find to what bin each particle belongs
inline int Pipeline::getColourBins(int r, int g, int b){
    int rbin = getBinIndex(r);
    int gbin = getBinIndex(g);
    int bbin = getBinIndex(b);
    return getBin(rbin , gbin , bbin);
}

inline double Pipeline::histogramCompare( float histA[33], float histB[33]){
    const int N = sizeof(histA) / sizeof(int);
    float maxA = *std::max_element(histA, histA+N);
    float maxB = *std::max_element(histB, histB+N);
    float max = maxA>maxB?maxA:maxB;

    for(int i=0;i<33;i++){  
      if(histA[i] != histA[i])
          histA[i] =0.0;
      if(histB[i] != histB[i])
          histB[i] =0.0;
  }

    cv::Mat M1 = cv::Mat(1,33, cv::DataType<float>::type , histA);
    cv::Mat M2 = cv::Mat(1,33, cv::DataType<float>::type , histB);

    int histSize = 33;;
    float rangeA[] = {0, maxA+1};
    float rangeB[] = {0, maxB+1};
    const float* histRangeA = {rangeA};
    const float* histRangeB = {rangeB};
    bool uniform = true;
    bool accumulate = false;
    cv::Mat a1_hist, a2_hist;

    cv::calcHist(&M1, 1, 0, cv::Mat(), a1_hist, 1, &histSize, &histRangeA, uniform, accumulate );
    normalize(a1_hist, a1_hist,  0, 1, CV_MINMAX);
    cv::calcHist(&M2, 1, 0, cv::Mat(), a2_hist, 1, &histSize, &histRangeB, uniform, accumulate );
    normalize(a2_hist, a2_hist,  0, 1, CV_MINMAX);

    cv::Mat sig1(33 ,2, cv::DataType<float>::type);  
    cv::Mat sig2(33 ,2, cv::DataType<float>::type); 

    for(int i=0;i<histSize;i++){
        float binval = a1_hist.at<float>(i);
        sig1.at< float >(i, 0) = binval;
        sig1.at< float >(i, 1) = i;
        binval = a2_hist.at< float>(i);
        sig2.at< float >(i, 0) = binval;
        sig2.at< float >(i, 1) = i;
    }
    float emd = cv::EMD(sig1, sig2, CV_DIST_L2);

//    double compar_chi    = cv::compareHist(a1_hist, a2_hist, CV_COMP_CHISQR);
//    double compar_bh     = cv::compareHist(a1_hist, a2_hist, CV_COMP_BHATTACHARYYA);
//    double compar_hell   = cv::compareHist(a1_hist, a2_hist, CV_COMP_HELLINGER );
     cout << emd << endl; //printf("similarity %5.5f %%\n", (1-emd)*100 );  
    return 0.0;
}

#endif 
