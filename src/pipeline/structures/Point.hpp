#include <iostream>
#include <vector>
#include <algorithm>


using namespace std;
class Point{
    
    public:
        int x , y , z , r , g , b , idx;        // Define x,y,z,r,g,b and point id
        vector<double> distancesA, distancesC;  // Define colour and angle distances
        inline Point();
        const inline void add(const double val);
        const inline void printDistances(const vector<double> & v);


    private:
        
    
    
};

inline Point::Point(){
    x = 0; y = 0 ; z = 0; r = 0; g = 0; b = 0;    
}
const inline void Point::add(const double val){
    distancesA.push_back(val);    
}
const inline void Point::printDistances(const vector<double>& v){
    std::for_each(v.begin(), v.end(), [](double v) { cout << v << "," <<endl; });
}
