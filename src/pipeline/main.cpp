#include <string> 
#include <sstream> 
#include <iostream> 
#include <vector> 
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h> 
#include <pcl/io/io.h> 
#include "include/pipeline.hpp"


// Main  function is the core of the pipeline.
// Every wrapper is used here more or less
// Program is given the name of the cloud and performs the following steps.

// Pipeline is the following 
//---------------------------------------------------//
// Cloud loading  --> statistical outlier removal    // 
// --Downsampling?(not sure if needed)               //
// --> ransac(plane estimation) --> structures taken //
// --> ICP on all the clouds -- > xyz transformation //
//---------------------------------------------------------//
// --> fpfh acquisition --> colour info acquisition  //
// --> extract Point clouds data                     //
//---------------------------------------------------//


//using namespace pcl;
//using namespace std;
int main(int argc, char* argv[]){

    //Variable decleration
    Pipeline pip;

    vector< PointCloud< PointXYZRGB > > clouds;
    vector< FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33> > histograms;

    PointCloud< PointXYZRGB >::Ptr  firstElem ( new PointCloud< PointXYZRGB > ) ;

    // Cloud loading
    for(int i=0;i<5;i++){
        PointCloud< PointXYZRGB >::Ptr cloud ( new PointCloud<PointXYZRGB> );
        string inputFile = "../resources/pipeline/clouds/cloudsmall";
        std::ostringstream ss;
        ss << i;
        inputFile       += ss.str();
        inputFile       += ".ply";

        //Load the cloud named inputFile
        io::loadPLYFile ( inputFile , *cloud );

        //removeStatisticalOutliers
        pip.removeStatisticalOutliers( cloud );

        //Plane estimation, keep surfaces that are not plane(plain?)
        pip.planeEstimation( cloud );


        //ICP with cloud transformation. The ICP is performed on every cloud after the first compared to the first
        if(i>0){
            pip.ICPTransform( cloud, firstElem );
            clouds.push_back( * cloud );
        }else{
            clouds.push_back( * cloud );
            copyPointCloud( * cloud, * firstElem);
        }

        //fpfh acquisition
        histograms.push_back( pip.fpfhEst( cloud ) );
        cout << histograms.size() << " size of hisogram brah   " << endl;
    }   

}
