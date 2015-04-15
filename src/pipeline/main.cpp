#include <string> 
#include <sstream> 
#include <iostream> 
#include <vector> 
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h> 
#include <pcl/io/io.h> 
#include <libconfig.h>
#include "include/pipeline.hpp"
#include <stdlib.h>     /* atof */


// Main  function is the core of the pipeline.
// Every wrapper is used here more or less
// Program is given the name of the cloud and performs the following steps.

// Pipeline is the following 
//---------------------------------------------------//
// Cloud loading  --> statistical outlier removal    // 
// --Downsampling?(not sure if needed)               //
// --> ransac(plane estimation) --> structures taken //
// --> ICP on all the clouds -- > xyz transformation //
// --> fpfh acquisition --> colour info acquisition  //
// --> extract Point clouds data                     //
//---------------------------------------------------//

typedef vector < vector < vector < int > > > colourCounts;

int main(int argc, char* argv[]){

    // Read the configuration file
    config_t cfg, *cf;
    const char *base = NULL;
    cf = &cfg;
    config_init(cf);

    if (!config_read_file(cf, "../src/pipeline/config/config.cfg")) {
        fprintf(stderr, "%s:%d - %s\n",
                config_error_file(cf),
                config_error_line(cf),
                config_error_text(cf));
        config_destroy(cf);
        return(EXIT_FAILURE);
    }
    config_lookup_string(cf, "leafSize", &base);
    double leafSize;
    leafSize  = atof(base);
    cout << " leaf size" << leafSize << endl;
    // End of config parsing

 

    //Variables decleration
    Pipeline  pip;
    vector< PointCloud< PointXYZRGB > > clouds;
    vector <vector< FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33> > > histograms;
    colourCounts  colCounts;
    PointCloud< PointXYZRGB >::Ptr  firstElem ( new PointCloud< PointXYZRGB > ) ;

    // Cloud loading
    for(int i=0;i<1;i++){
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

        cout << " downsampling with leaf size " << leafSize << endl;

        //Plane estimation, keep surfaces that are not plane(plain?)
        pip.planeEstimation( cloud );

        //Downsample the cloud
        pip.downsample(cloud, leafSize);


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


        // Colour information extraction
        // RGB colour spectrum will be discretized into N bins
        colCounts.push_back(pip.colourInformationExtractor(cloud));

    }   

}
