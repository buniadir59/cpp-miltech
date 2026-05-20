#pragma once

#include "ballistics.hpp"
#include "point_math.hpp"

#include <limits>
#include <string>
#include <array>

/* **** drone.hpp / drone.cpp 
  TargetState::
    predict target position for use in lead targeting 

  Mission::
    steer drone along drop path

  DroneConfig:: for initial data from input file 
  Drone::
    update drone params 
    choose target 
  */

constexpr int kNtgts = 5;    //number of targets


namespace drone {
  constexpr double eps = std::numeric_limits<double>::epsilon(); 

  auto normalize(double value) -> double;
    
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
        double initial_direction{};
        double attack_speed = 0.0;
        double acceleration_path = 0.0;
        std::string ammo_name = "";
        double hit_radius = 0.0;
        double angular_speed = 0.0;
        double turn_threshold = 0.0;
    };

    struct TargetState { //current information on target available to drone
        bool has_previous = false;
        double last_known_s = 0.0;
        pointmath::Point last_known{};
        pointmath::Point velocity{}; //estimated velocity from two last known positions

        double time_accuracy=0.0;
        double time_total=0.0;
        double time_to_interim=0.0; //time to reach interim point 
        ballistics::DropSolution dropRoute{};
        
        auto getSpeed() const -> double { return pointmath::getLength(velocity); }

        void update(const pointmath::Point& new_position, double time_s) { //receive new "real"(interpolated) position from simulation  
          if (has_previous) {
              const double dt = time_s - last_known_s; 
              if (dt > eps) velocity = (new_position - last_known) / dt;
          } else { //velocity stays 0
              has_previous = true;
          }

          last_known = new_position;
          last_known_s = time_s;
        }       

        pointmath::Point getLeadPosition(double delta_t) const { return last_known + velocity * (delta_t); }; //projected position for lead targeting

        auto calculateBallisticSolutionAt(double delta_t, ballistics::BallisticsInput& input)-> void  {    
          pointmath::Point pos_at_time = getLeadPosition(delta_t);
          input.target_x = pos_at_time.x;
          input.target_y = pos_at_time.y;
          dropRoute = ballistics::compute_drop_solution(input);
        };      
    }; //eo TargetState
    
    enum DroneState { STOPPED=0, ACCELERATING, DECELERATING, TURNING, MOVING };

    enum MissionState {NONE, TO_INTERIMP, TO_FIREP, FIRED, FAILED, COMPLETED};

    struct Mission {
      MissionState state{NONE};
      int tgtTag = -1; 
      double timeToDestination = 0.0; //time to reach destination, calculated at start of mission
      double maxSpeed = 0.0; //max speed to reach interim point
      pointmath::Point decelerateAtPoint{0, 0}; //point where to start decceleration to reach interim point
      pointmath::AngleRad destAngle{0.0}; //direction to destination, calculated at start of mission
      pointmath::Point destPoint{0, 0};

      auto missionStateToStr() const -> std::string {
        switch (state) {
            case NONE: return "NONE";
            case TO_INTERIMP: return "TO_INTERIMP";
            case TO_FIREP: return "TO_FIREP";
            case FIRED: return "FIRED";
            case FAILED: return "FAILED";
            case COMPLETED: return "COMPLETED";
            default: return "UNKNOWN_STATE";
        }
      }
    };

    struct Drone {
        // constant - obtained from input data: 
        double alt;             //altitude
        double accPath;         //acceler path
        double attSpeed;  //attack speed
        double hitRad;          //Радіус ураження — допустима похибка попадання (м)
        double angSpeed;        //Кутова швидкість повороту (рад/с)
        double turnThrld;       //Пороговий кут для зупинки (рад)
        std::string ammoName;   //Ammo ammo;            

        // constant - calculated once and saved for future use
        double kAccTime;     //time to gain attack speed (acceler time), calculated from acceler path and attack speed    
	      double kAcc; //calculated from acceler time and attack speed
        double kAccuracy_m = 0.0; //distance to destination to decide it is reached

        // changing during simulation:    
        double speed = 0.0;         //current speed, m/s
        DroneState state = STOPPED; //assume initial state is full stop
        pointmath::Point coord{};              //initialized from input file
        pointmath::Point dirXY{};              //direction by X and Y (as Point)  according to dirAngleRad
        pointmath::AngleRad dirRad{0.0};       //напрямок дрона (радіани, від осі X) //initialized from input file                      
        Mission mission{};                     //current mission 
        std::array<drone::TargetState, 5> tgts{};   //known to drone information about targets 
        
        int countMaxRecalc = 0;    //max number of recalculated drop solutions

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
        {
            setDroneDirection(config.initial_direction); 
            kAcc = attSpeed * attSpeed / (2 * accPath);
	          kAccTime = 2 * accPath / attSpeed; //time to accelerete from STOP to ATTACKspeed
        }       
        
        auto isOnMission() const -> bool { return mission.tgtTag >= 0; }
        auto getAmmoFlyTime() const -> double;

        void setDroneDirection (double aR){ dirRad = aR; dirXY = pointmath::cossin(dirRad.value); };
        void moveDrone(double dt);
        auto getTurningTime(double angle) const -> double;
        auto getTimeToFlyToInterimPoint(double dist) const -> double;
        auto getTimeToFlyToFirePoint(double dist) const -> double;
        
        auto getBestTarget() -> int;
        auto calculateTimeForDropRoute(pointmath::Point start, TargetState& tgt) -> double; 
        
        auto breakMission()-> void  { mission.state = NONE; mission.tgtTag = -1; }  
        auto startNewMission(double time_step) -> int;       
        auto continueMission(double t_step) -> MissionState;
        auto getHitDistance(pointmath::Point tgt_pos_at_hit) -> double;
        
        auto isTargetHit(double hit_dist) const -> bool { return hit_dist <= hitRad; }

        auto droneStateToStr() const -> std::string  {
        switch (state) {
            case STOPPED: return "STOPPED";
            case ACCELERATING: return "ACCELERATING";
            case DECELERATING: return "DECELERATING";
            case TURNING: return "TURNING";
            case MOVING: return "MOVING";
            default: return "UNKNOWN_STATE";
        }
      }
      
    };

    

} //drone
