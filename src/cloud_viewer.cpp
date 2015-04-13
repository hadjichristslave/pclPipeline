#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>

int user_data;

    void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    std::cout << "i only run once" << std::endl;
}

    void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int main (int argc, char *argv[])
{

    std::string inputFile  = argv[1];
    std::string ply        = argv[2];
    std::string subgraph   = argv[3];

    std::cout << "input file is :../resources/" << inputFile << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if(ply!="ply"){
        inputFile = "../resources/" + inputFile + ".pcd";
        pcl::io::loadPCDFile ( inputFile , *cloud);
    }else{
        inputFile = "../resources/" + inputFile + ".ply";
        pcl::io::loadPLYFile (inputFile, *cloud);
    }


    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered
    //    viewer.showCloud(cloud );

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    if(subgraph == "yes"){
        Eigen::Vector4f minPoint;

        minPoint[0]= atof(argv[4]);  // define minimum point x
        minPoint[1]= atof(argv[5]);  // define minimum point y
        minPoint[2]= atof(argv[6]);  // define minimum point z 

        Eigen::Vector4f maxPoint;
        maxPoint[0]=atof(argv[7]);  // define max point x
        maxPoint[1]=atof(argv[8]);  // define max point y
        maxPoint[2]=atof(argv[9]);  // define max point z 

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB>); 

        pcl::CropBox<pcl::PointXYZRGB> cropFilter;
        cropFilter.setInputCloud (cloud);
        cropFilter.setMin(minPoint);
        cropFilter.setMax(maxPoint);
        cropFilter.filter (*cloudOut); 

        viewer.showCloud(cloudOut);
        //This will only get called once
        viewer.runOnVisualizationThreadOnce (viewerOneOff);
        //This will get called once per visualization iteration
        viewer.runOnVisualizationThread (viewerPsycho);
    }else{
        viewer.showCloud(cloud);
        //This will only get called once
        viewer.runOnVisualizationThreadOnce (viewerOneOff);
        //This will get called once per visualization iteration
        viewer.runOnVisualizationThread (viewerPsycho);        
    }
    while (!viewer.wasStopped ())
    {
        //you can also do cool processing here
        //FIXME: Note that this is running in a separate thread from viewerPsycho
        //and you should guard against race conditions yourself...
        user_data++;
    }
    return 0;
}
