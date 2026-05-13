#pragma once

//#include "ballistics.hpp"
#include "point_math.hpp"
//#include <numbers>
#include <string>
#include <array>

/* **** drone.hpp / drone.cpp
  DroneConfig 
  TargetState::
    update target position
    predict target position
  Drone::
    update drone params //TODO   
    choose target //TODO
  */

namespace drone {
/* Input file description:
  Параметр	Тип	Опис
  xd, yd, zd	float	Координати дрона (zd — висота, м)
  initialDir	float	Початковий напрямок дрона (радіани, від осі X)
  attackSpeed	float	Швидкість атаки дрона (м/с)
  accPath	    float	Довжина розгону/гальмування (м)
  ammo_name	char[]	Назва боєприпасу (див. таблицю боєприпасів)
  arrayTimeStep	float	Крок часу масиву координат цілей (с)    (like 5s)
  simTimeStep	float	Крок часу симуляції (с)                     (like 0.1s)
  hitRadius	float	Радіус ураження — допустима похибка попадання (м)
  angularSpeed	float	Кутова швидкість повороту (рад/с)
  turnThreshold	float	Пороговий кут для зупинки (рад) */
    struct DroneConfig {
        pointmath::Point position{};
        double altitude = 0.0;
        pointmath::AngleRad initial_direction{};
        double attack_speed = 0.0;
        double acceleration_path = 0.0;
        std::string ammo_name = "";
        double hit_radius = 0.0;
        double angular_speed = 0.0;
        double turn_threshold = 0.0;
    };

    struct TargetState { //current information on target available to drone
     //   pointmath::Point previous{};
        pointmath::Point last_known{};
        double last_known_s = 0.0;
     //   double current_time_s = 0.0;

        bool has_previous = false;
        pointmath::Point velocity{}; //estimated velocity from two last known positions

        void update(pointmath::Point new_position, double time_s); //receive new "real" position                     
        pointmath::Point predictPositionAt(double future_t_s) const; //for lead targeting
    };
    
    enum DroneState { STOPPED=0, ACCELERATING, DECELERATING, TURNING, MOVING };

    struct Drone {
        // constant - obtained from input data: 
        double alt;             //altitude
        double accPath;         //acceler path
        double attSpeed;  //attack speed
        double hitRad;          //Радіус ураження — допустима похибка попадання (м)
        double angSpeed;        //Кутова швидкість повороту (рад/с)
        double turnThrld;       //Пороговий кут для зупинки (рад)
        std::string ammoName;   //Ammo ammo;            

        // changing during simulation:
        int currentTgtTag = -1;     //no mission set
        DroneState state = STOPPED; //assume initial state is full stop
        pointmath::Point coord{};              //initialized from input file
        pointmath::AngleRad dirRad{0};         //напрямок дрона (радіани, від осі X) //initialized from input file
        pointmath::Point dirXY{};              //direction by X and Y (as Point)  according to dirAngleRad
        double speed = 0.0;               //current speed, m/s
        std::array<drone::TargetState, 5> tgts{}; //known information about targets  to drone

        explicit Drone(const DroneConfig& config)
        :   alt{config.altitude},
            accPath{config.acceleration_path},
            attSpeed{config.attack_speed},
            hitRad{config.hit_radius},
            angSpeed{config.angular_speed},         
            turnThrld{config.turn_threshold},
            ammoName(config.ammo_name),
            coord{config.position},
            dirRad{config.initial_direction}    
        {}                                    
    };

    const std::string& getDroneStateStr(DroneState state);

} //drone
