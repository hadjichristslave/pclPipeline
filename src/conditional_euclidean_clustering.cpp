#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/time.h>
#include <boost/lexical_cast.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>


typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

    bool
enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
    if (fabs (point_a.intensity - point_b.intensity) < 5.0f){
        std::cout << " true intensity difference 5 1  limit " << fabs (point_a.intensity - point_b.intensity) << std::endl;
        return (true);
    }
    else
        return (false);
}

    bool
enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
    if (fabs (point_a.intensity - point_b.intensity) < 5.0f){
        std::cout << " true intensity difference 5 2 limit " << fabs (point_a.intensity - point_b.intensity) << std::endl;
        return (true);
    }
    //if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
    //       return (true);
    return (false);
}

    bool
customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
    if (squared_distance < 10)
    {
        if (fabs (point_a.intensity - point_b.intensity) < 1.0f){
            std::cout << " true intensity difference 8 3 limit with distance:" << fabs (point_a.intensity - point_b.intensity) << "-" << squared_distance << std::endl;
            std::cout << " normal estimation" <<  fabs (point_a_normal.dot (point_b_normal)) << std::endl;
            return (true);
        }
        if (fabs (point_a_normal.dot (point_b_normal)) < 0.06){
            std::cout << " normal fix" <<  fabs (point_a_normal.dot (point_b_normal))   <<   std::endl;
            return (true);
        }
    }
    else
    {
        //        if (fabs (point_a.intensity - point_b.intensity) < 0.15f){
        //           std::cout << " true intensity difference 5 4 limit with distance:" << fabs (point_a.intensity - point_b.intensity) << "-" << squared_distance << std::endl;
        //        return (true);
        //        }
    }
    return (false);
}


int main (int argc, char** argv)
{
    // Data containers used
    pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr buffercloud( new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
    pcl::console::TicToc tt;

    // Load the input point cloud
    std::cerr << "Loading...\n", tt.tic ();

    std::string ply       = argv[2];

    std::string input = argv[1];
    input = "../resources/" + input;


    if(ply =="ply"){
        input += ".ply";
        pcl::io::loadPLYFile (input, *cloud_in);
        pcl::io::loadPLYFile (input, *buffercloud);       
    }else{
        input +=".pcd";
        pcl::io::loadPCDFile(input, *cloud_in);
        pcl::io::loadPCDFile(input, *buffercloud);        
    }
    std::cout<< " Test " << std::endl;



    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";


    for(int i=0;i<cloud_in->points.size();++i){
        double r = (double) buffercloud->points[i].r;
        double g = (double) buffercloud->points[i].g;
        double b = (double) buffercloud->points[i].b;
        r /=255; g /= 255; b/=255;
        double gamma = .5;
        cloud_in->points[i].intensity = pow(r,1/gamma) * .21 +  \
                                        pow(g,1/gamma) * .72 + \
                                        pow(b,1/gamma) * .07;

    }
    // Downsample the cloud using a Voxel Grid class
    std::cerr << "Downsampling...\n", tt.tic ();
    pcl::VoxelGrid<PointTypeIO> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (.0001, .0001, .0001);
    //vg.setLeafSize(80.0,80.0,80.0);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_out);
    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->points.size () << " points\n";


    // Set up a Normal Estimation class and merge data in cloud_with_normals
    std::cerr << "Computing normals...\n", tt.tic ();
    pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (.03);
    ne.compute (*cloud_with_normals);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerNorm (new pcl::visualization::PCLVisualizer ("3D Viewer"));


    viewerNorm->addPointCloudNormals<PointTypeIO, PointTypeFull>(cloud_out, cloud_with_normals, 1, 0.05, "normals");
    viewerNorm->addPointCloud<PointTypeIO> (cloud_out, "cloud_in");

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<PointTypeIO, PointTypeFull, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud_out);
    fpfh.setInputNormals (cloud_with_normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointTypeIO>::Ptr tree (new pcl::search::KdTree<PointTypeIO> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (0.05);

    // Compute the features
    fpfh.compute (*fpfhs);
    pcl::PCDWriter writer;

    writer.write<pcl::FPFHSignature33>("../resources/test.pcd" , *fpfhs, false);

//    viewerNorm->addPointCloud(pfhs, "cloud in");
 //   viewerNorm->spin();

    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    // Set up a Conditional Euclidean Clustering class
    std::cerr << "Segmenting to clusters...\n", tt.tic ();
    pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (100.0);
    cec.setMinClusterSize (cloud_with_normals->points.size () / 1000);
    cec.setMaxClusterSize (cloud_with_normals->points.size () / 4);
    cec.segment (*clusters);
    cec.getRemovedClusters (small_clusters, large_clusters);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    std::cout << " Small clusters: " << small_clusters->size() << " big clusters: " << large_clusters->size() << " normal clusters: " << clusters->size() << std::endl;


    // Visualizing options

    for(unsigned int i=0;i<clusters->size(); ++i){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (unsigned int j = 0; j < (*clusters)[i].indices.size (); ++j) {
            pcl::PointXYZRGB basic_point;
            basic_point.x = cloud_out->points[(*clusters)[i].indices[j]].x;
            basic_point.y = cloud_out->points[(*clusters)[i].indices[j]].y;
            basic_point.z = cloud_out->points[(*clusters)[i].indices[j]].z;
            basic_point.r = buffercloud->points[(*clusters)[i].indices[j]].r;
            basic_point.g = buffercloud->points[(*clusters)[i].indices[j]].g;
            basic_point.b = buffercloud->points[(*clusters)[i].indices[j]].b;
            basic_cloud_ptr->points.push_back(basic_point);
        }
        basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
        basic_cloud_ptr->height = 1;
        std::string name = "sample cloud";
        std::string UID   = boost::lexical_cast<std::string>(i);
        name = name + UID;

        viewer->addPointCloud<pcl::PointXYZRGB> (basic_cloud_ptr, name);

        viewer->spin();
    }
    std::cout << " small clusters" <<std::endl;
    for(unsigned int i=0;i<small_clusters->size(); ++i){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (unsigned int j = 0; j < (*small_clusters)[i].indices.size (); ++j) {
            pcl::PointXYZRGB basic_point;
            basic_point.x = cloud_out->points[(*small_clusters)[i].indices[j]].x;
            basic_point.y = cloud_out->points[(*small_clusters)[i].indices[j]].y;
            basic_point.z = cloud_out->points[(*small_clusters)[i].indices[j]].z;
            basic_point.r = buffercloud->points[(*small_clusters)[i].indices[j]].r;
            basic_point.g = buffercloud->points[(*small_clusters)[i].indices[j]].g;
            basic_point.b = buffercloud->points[(*small_clusters)[i].indices[j]].b;
            basic_cloud_ptr->points.push_back(basic_point);
        }
        basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
        basic_cloud_ptr->height = 1;
        std::string name = "sample cloud";
        std::string UID   = boost::lexical_cast<std::string>(i);
        name = name + UID;

        viewer->addPointCloud<pcl::PointXYZRGB> (basic_cloud_ptr, name);

        viewer->spin();
    }
    std::cout <<" large clusters"  << std::endl;
    for(unsigned int i=0;i<large_clusters->size(); ++i){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (unsigned int j = 0; j < (*large_clusters)[i].indices.size (); ++j) {
            pcl::PointXYZRGB basic_point;
            basic_point.x = cloud_out->points[(*large_clusters)[i].indices[j]].x;
            basic_point.y = cloud_out->points[(*large_clusters)[i].indices[j]].y;
            basic_point.z = cloud_out->points[(*large_clusters)[i].indices[j]].z;
            basic_point.r = buffercloud->points[(*large_clusters)[i].indices[j]].r;
            basic_point.g = buffercloud->points[(*large_clusters)[i].indices[j]].g;
            basic_point.b = buffercloud->points[(*large_clusters)[i].indices[j]].b;
            basic_cloud_ptr->points.push_back(basic_point);
        }
        basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
        basic_cloud_ptr->height = 1;
        std::string name = "sample cloud";
        std::string UID   = boost::lexical_cast<std::string>(i);
        name = name + UID;

        viewer->addPointCloud<pcl::PointXYZRGB> (basic_cloud_ptr, name);

        viewer->spin();
    }





    // Save the output point cloud
    std::cerr << "Saving...\n", tt.tic ();
    pcl::io::savePCDFile ("../resources/output.pcd", *cloud_out);
    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    return (0);
}

