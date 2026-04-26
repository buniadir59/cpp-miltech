#include <iostream>
#include <iomanip>
#include <fstream>
#include <ostream>
#include <string>
#include <cmath>

//#define TEST_OUTPUT

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

constexpr long kTicksPerRevolution = 1024;  //iмпульсiв на один оберт колеса
constexpr double kWheelRadiusM = 0.3;       //радiус колеса у метрах (дiаметр 60 см)
constexpr double kWheelbaseM = 1.0;         //вiдстань мiж лiвим i правим бортом, у метрах
constexpr double kDistancePerTick = 2.0 * M_PI * kWheelRadiusM / kTicksPerRevolution;

struct position_t {
    double x=0, y=0, theta=0;
};

std::ostream& operator<<(std::ostream& os, const position_t& p) { //overloaded << for position_t
    return os << std::fixed << std::setprecision(2) << p.x << " " << p.y << " " << p.theta;
}
 
struct odo_t {
    long fl_ticks, fr_ticks, bl_ticks, br_ticks;

    odo_t& operator-=(const odo_t& delta) {  //decrement
        this->fl_ticks -= delta.fl_ticks;
        this->fr_ticks -= delta.fr_ticks;
        this->bl_ticks -= delta.bl_ticks;
        this->br_ticks -= delta.br_ticks;
        return *this;
    }
};

odo_t operator-(odo_t a1, const odo_t& a2) { //diff
        a1 -= a2;
        return a1;
}

std::ostream& operator<<(std::ostream& os, const odo_t& o) { //overloaded << for tick_t
    return os << "F(" << o.fl_ticks << ", " << o.fr_ticks
              << ") B(" << o.bl_ticks << ", " << o.br_ticks << ")";
}

/**** returns time and, in o, respective odometry data*/
bool readTickData(std::istream& in, long& timestamp_ms, odo_t& odo) {
    return static_cast<bool>( //NB! if error, nullptr is returned
        in >> timestamp_ms
           >> odo.fl_ticks
           >> odo.fr_ticks
           >> odo.bl_ticks
           >> odo.br_ticks
    );
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
        return 2;
    }

    position_t pos{}; //to be calculated
    long timestamp_ms{}; 
    odo_t current{}, previous{};
    bool has_previous = false;

    while (readTickData(in, timestamp_ms, current)) {

#ifdef TEST_OUTPUT       
        std::cout << timestamp_ms << ' ' << current << std::endl;
#endif        
        
        if (!has_previous) { //initial data=first line
            previous = current;
            has_previous = true;
            continue;
        }

        // calculate delta from previous -> current
        odo_t delta = current - previous; 

        //Усереднити борти (передне i заднє колесо одного боку обертаються синхронно)&Перевести у метри:
        double dl = (kDistancePerTick / 2.0) * (delta.fl_ticks + delta.bl_ticks);
        double dr = (kDistancePerTick / 2.0) * (delta.fr_ticks + delta.br_ticks);

        //Скiльки пройшов центр робота i на скiльки повернувся:
        double dist  = (dl + dr) / 2.0; //d = (dL + dR) / 2 // пройдена вiдстань центру
        double dtheta = (dr - dl) / kWheelbaseM; //dtheta = (dR - dL) / wheelbase_m // змiна орiєнтацiї

        // update pos: midpoint integration (усереднений напрямок на кроцi)
        pos.x += dist * cos(pos.theta + dtheta / 2.0);
        pos.y += dist * sin(pos.theta + dtheta / 2.0);
        pos.theta += dtheta;

        //show position
        std::cout << timestamp_ms << ' ' << pos << '\n'; 

        previous = current;
    }


#ifdef TEST_OUTPUT    
    std:: cout << "End of input data\n";
#endif

    return 0;
}


