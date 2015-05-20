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
#include <pcl/visualization/cloud_viewer.h>


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
    bool colourAngleDistribution = true;
    float xp , yp, zp, rp, gp, bp;
    int lines = 0;



    std::ifstream infile("/home/panos/Desktop/new.txt");
    std::ifstream infilz("/home/panos/Desktop/new.txt");

    while (infile >> xp >> yp>> zp>> rp>> gp>> bp){lines++;}
    //cout << " Processing number of lines " << lines;
    // Read the configuration file
    config_t cfg, *cf;
    const char *base = NULL;
    cf = &cfg;
    int cloudSize, numOfBins;
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
    config_lookup_int(cf, "clouds" , &cloudSize);
    config_lookup_int(cf, "numOfBins" , &numOfBins);
    double leafSize;
    leafSize  = atof(base);
    //Variables decleration
    Pipeline  pip;
    vector< PointCloud< PointXYZRGB > > clouds;
    vector< vector< vector< float > > > histogramValues;
    colourCounts  colCounts;
    PointCloud< PointXYZRGB >::Ptr  firstElem ( new PointCloud< PointXYZRGB > ) ;
    // Cloud loading
    //    cout << " x,y,z,Kullback-Leibler,EMD,Hellinger," ;
    //    for(int i = 0; i < numOfBins; i++)
    //       cout << " Colour bin " << i << ",";
    //    cout << endl;
    for(int i=0;i<cloudSize;i++){
        PointCloud< PointXYZRGB >::Ptr cloud ( new PointCloud<PointXYZRGB> );
        string inputFile = "../resources/pipeline/clouds/cloudsmall" + SSTR(i) + ".ply";
        io::loadPLYFile ( inputFile , *cloud );
        cloud->resize(lines);
        lines    = 0 ;
        while(infilz >> xp >> yp>> zp>> rp>> gp>> bp){
            cloud->points[lines].x = xp;
            cloud->points[lines].y = yp;
            cloud->points[lines].z = zp;
            cloud->points[lines].r = rp;
            cloud->points[lines].g = gp;
            cloud->points[lines].b = bp;
            lines++;
        }
        pip.downsample(cloud, leafSize);
        pip.removeStatisticalOutliers( cloud );

        //pcl::visualization::CloudViewer viewer("Cloud Viewer");
//        viewer.showCloud(cloud);

        //use the following functions to get access to the underlying more advanced/powerful
        //PCLVisualizer

        //This will only get called once

        //This will get called once per visualization iteration
  //      while (!viewer.wasStopped ())
        //{
            //you can also do cool processing here
            //FIXME: Note that this is running in a separate thread from viewerPsycho
            //and you should guard against race conditions yourself...
//        }
        for(int k=0;k<cloud->points.size();k++)
            //cout <<  cloud->points[k].x  << "," <<  cloud->points[k].y  <<  "," << cloud->points[k].z << ","<< (int)cloud->points[k].r << "," << (int)cloud->points[k].g << "," << (int)cloud->points[k].b<< endl;
        cout << " cloud size before ransac " << cloud->size() <<endl;
        pip.planeEstimation( cloud );
        cout << " cloud size after ransac" << cloud->size()<< endl;
        //ICP with cloud transformation. The ICP is performed on every cloud after the first compared to the firstter
        if(i>0){
            pip.ICPTransform( cloud, firstElem );
            cout << " cloud size after ICP " << cloud->size() << endl; 
            clouds.push_back( * cloud );
          }else{
            clouds.push_back( * cloud );
            copyPointCloud( * cloud, * firstElem);
        }
        histogramValues.push_back( pip.fpfhEst( cloud ) );
        // RGB colour spectrum will be discretized into N bins
        colCounts.push_back(pip.colourInformationExtractor(cloud));
        // Data ouput pipeline
    }
    // Histogram output pipeline
    if( colourAngleDistribution){
        system("echo "" >> /home/panos/Desktop/cloudData/aggregated.csv");
        ofstream myfile;
        myfile.open("/home/panos/Desktop/cloudData/aggregated.csv");
        string header = "x,y,z,Kullback-leibler,EMD,Hellinger,"; 
        for(int i = 0 ; i < numOfBins; i++) header += "Bin" + SSTR(i)+ ",";
        myfile << header; 
        myfile << "\n";
        cout << " number of clouds " << cloudSize << endl;
        for(int ij = 0;  ij < cloudSize; ij++){
            cout << " current cloud size " << clouds[ij].size() << endl;
            for(int ik = 0 ; ik < clouds[ij].size(); ik++){
                string line =  SSTR(clouds[ij].points[ik].x) \
                               + "," + SSTR(clouds[ij].points[ik].y) + "," \
                               + SSTR(clouds[ij].points[ik].z) +"," \
                               + SSTR(histogramValues[ij][ik][0])+","\
                               + SSTR(histogramValues[ij][ik][1])+"," \
                               + SSTR(histogramValues[ij][ik][2])+ ",";
                double sumi = 0 ;
                for(int i = 0; i < numOfBins; i++)
                    sumi += colCounts[ij][ik][i]; 
                for(int i = 0; i < numOfBins; i++) line += SSTR(colCounts[ij][ik][i] /*/sumi*/ ) + ",";
                line += "\n";
                myfile << line;
            }
            myfile << "-----";
            myfile << "\n";
        }
        myfile.close();
        /*
           system("echo "" > /home/panos/Desktop/cloudData/angleDistances.csv");
           system("echo "" > /home/panos/Desktop/cloudData/colours.csv");
           myfile.open ("/home/panos/Desktop/cloudData/colours.csv");
           for(int i = 0; i < numOfBins; i++) myfile<< "Bin" <<i<<",";
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
           myfile.open ("/home/panos/Desktop/cloudData/angleDistances.csv");
           myfile<< " Kullback-Leibler, EMD, Hellinger \n";
           for(int i = 0; i < histogramValues.size(); i++)
           for(int j = 0; j < histogramValues[i].size(); j++)
           myfile<< (double)histogramValues[i][j][0] << "," << (double)histogramValues[i][j][1] << "," << (double)histogramValues[i][j][2]<<"\n";
           myfile.close(); */
    }
}        
