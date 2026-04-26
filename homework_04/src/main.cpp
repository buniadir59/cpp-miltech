#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <cmath>

#define TEST_OUTPUT

    // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
    //
    // Parameters:
    //   ticks_per_revolution = 1024
    //   wheel_radius_m       = 0.3
    //   wheelbase_m          = 1.0
    //
    // Input:  text file with 5 whitespace-separated numbers per line:
    //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
    // Output: same tabular format on stdout, starting from the second sample:
    //         timestamp_ms x y theta

constexpr long ticks_per_revolution = 1024; //iмпульсiв на один оберт колеса
constexpr double wheel_radius_m = 0.3; //радiус колеса у метрах (дiаметр 60 см)
constexpr double wheelbase_m = 1.0; //вiдстань мiж лiвим i правим бортом, у метрах



struct position_t {
    double x=0, y=0, theta=0;
};

std::ostream& operator<<(std::ostream& os, const position_t& p) { //overloaded << for position_t
    return os << p.x << " " << p.y << " " << p.theta;
}
 
struct odo_t {
    long fl_ticks, fr_ticks, bl_ticks, br_ticks;
};

std::ostream& operator<<(std::ostream& os, const odo_t& o) { //overloaded << for tick_t
    return os <<  "F(" << o.fl_ticks << ", " << o.fr_ticks << ") B(" << o.bl_ticks << ", " << o.br_ticks << ")"<<std::endl;
}

/**** returns time and, in o, respective odometry data*/
int readTickData(odo_t& o, std::ifstream& in) { //TODO eo file?
    int t;
    in >> t >> o.fl_ticks >> o.fr_ticks >> o.bl_ticks >> o.br_ticks;
    return in.eof() ? -1 : t;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }
    std::string filename = argv[1];
    std::ifstream in(filename); //file for getting odometry  data 

    std::cout << "Input file: " << filename << std::endl;

    if (!in) {
        std::cerr << "Cannot open input file: " << filename << std::endl;
        return 1;
    }
    
    long tick = 0;
    odo_t now;
    position_t pos;

    while (tick >= 0) {
        tick = readTickData(now, in);
        if (tick < 0) break;//eo file

#ifdef TEST_OUTPUT       
        std::cout << tick << ' ' << now << std::endl;
#endif

        //Calculate position 
        // //TODO 

        //show position
        std::cout << pos << std::endl;
    }

#ifdef TEST_OUTPUT    
    std:: cout << "End of input data\n";
#endif

    return 0;
}


