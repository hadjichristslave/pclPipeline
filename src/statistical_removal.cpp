#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
    int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

   // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file


    std::string inputFile = argv[1];
    inputFile = "../resources/" + inputFile + ".pcd";
    std::cout << "input file is :" << inputFile << std::endl;

//    reader.read<pcl::PointXYZ> (inputFile, *cloud);
    pcl::io::loadPLYFile ("/home/panos/Desktop/pcl/resources/kinect.ply", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0.5);
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;

    std::string output = argv[1];
    output = "../resources/inliers_" + output + ".pcd";

    writer.write<pcl::PointXYZI> (output, *cloud_filtered, false);

    sor.setNegative (true);
    sor.filter (*cloud_filtered);

    output = argv[1];
    output = "../resources/outliers_" + output + ".pcd";
     

    writer.write<pcl::PointXYZI> (output, *cloud_filtered, false);

    return (0);
}
