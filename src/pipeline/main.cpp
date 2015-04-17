#include <string> 
#include <sstream> 
#include <iostream> 
#include <fstream>
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

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

int main(int argc, char* argv[]){
    bool printColourBins, printDistances = true;
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
    //Variables decleration
    Pipeline  pip;
    vector< PointCloud< PointXYZRGB > > clouds;
    vector <vector< FPFHEstimation<PointXYZRGB, PointNormal, FPFHSignature33> > > histograms;
    vector< vector< vector< float > > > histogramValues;
    colourCounts  colCounts;
    PointCloud< PointXYZRGB >::Ptr  firstElem ( new PointCloud< PointXYZRGB > ) ;
    // Cloud loading
    for(int i=0;i<1;i++){
        PointCloud< PointXYZRGB >::Ptr cloud ( new PointCloud<PointXYZRGB> );
        string inputFile = "../resources/pipeline/clouds/cloudsmall" + SSTR(i) + ".ply";
        io::loadPLYFile ( inputFile , *cloud );
        pip.removeStatisticalOutliers( cloud );
        pip.planeEstimation( cloud );
        pip.downsample(cloud, leafSize);
        //ICP with cloud transformation. The ICP is performed on every cloud after the first compared to the first
        if(i>0){
            pip.ICPTransform( cloud, firstElem );
            clouds.push_back( * cloud );
        }else{
            clouds.push_back( * cloud );
            copyPointCloud( * cloud, * firstElem);
        }
        histogramValues.push_back( pip.fpfhEst( cloud ) );
        // RGB colour spectrum will be discretized into N bins
        colCounts.push_back(pip.colourInformationExtractor(cloud));
    }
    if(printColourBins){
        ofstream myfile;
        myfile.open ("/home/panos/Desktop/cloudData/colours.csv");
        for(int i = 0; i < 125; i++) myfile<< "Bin" <<i<<",";
        myfile<<"\n";
        for(int i = 0; i < colCounts.size(); i++)
            for(int j = 0; j < colCounts[i].size(); j ++){
                for(int k = 0 ; k < colCounts[i][j].size() ; k ++ ){
                    std::string stream = SSTR(colCounts[i][j][k]);
                    stream            += (k==colCounts[i][j].size()-1)?"":",";
                    myfile            << stream;
                }
                myfile<< "\n";
            }
        myfile.close();
    }
    if(printDistances){
        ofstream myfile;
        myfile.open ("/home/panos/Desktop/cloudData/angleDistances.csv");
        myfile<< " Kullback-Leibler, EMD, Hellinger \n";
        for(int i = 0; i < histogramValues.size(); i++)
            for(int j = 0; j < histogramValues[i].size(); j++)
                myfile<< (double)histogramValues[i][j][0] << "," << (double)histogramValues[i][j][1] << "," << (double)histogramValues[i][j][2]<<"\n";
        myfile.close();
    }
}        
