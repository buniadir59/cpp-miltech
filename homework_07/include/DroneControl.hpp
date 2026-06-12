#pragma once
#include "math/point_math.hpp"
#include "math/angle_math.hpp"
#include "dto/MissionConfig.hpp"

namespace core {

enum DrState { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING };

class DroneControl {
 
  double angSpeed;   // Кутова швидкість повороту (рад/с)
  anglemath::AngleRad destAngle; //set under mission control

public:
 // constant - obtained from input data:
  double alt;       // altitude
  double accPath;   // acceler path
  double attSpeed;  // attack speed
  double turnThrld;  // Пороговий кут для зупинки (рад)
  // constant - calculated once and saved for future use
  double kAcceleration;      // calculated from acceler time and attack speed
  // changing during simulation:
  pointmath::Point coord{};               // initialized from input file
  anglemath::AngleRad dirRad{0.0};        // напрямок дрона (радіани, від осі X) //initialized from input file
  pointmath::Point dirXY{};               // direction by X and Y (as Point)  according to dirAngleRad
  
  double speed = 0.0;          // current dr speed, m/s

  DrState state = STOPPED;  // assume initial state is full stop 

  //auto setDestinationAngle(anglemath::AngleRad destAngle)-> void;

  void setDroneDirection(double aR);

    DroneControl(const dto::MissionConfig& config)
    : angSpeed{config.angular_speed}
    , alt{config.altitude}
    , accPath{config.acceleration_path}
    , attSpeed{config.attack_speed}
    ,  turnThrld{config.turn_threshold}
    , coord{config.drone_position}
    , dirRad{config.initial_direction}
  {
    setDroneDirection(config.initial_direction);
    kAcceleration = attSpeed * attSpeed / (2 * accPath);  
  }

  auto getTimeToGainAttackSpeed() const -> double; 
  auto getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move, double time_step) const -> double;
  auto getTurnTime(anglemath::AngleRad delta) const -> double;
  auto getTimeToFlyToInterimPoint(double dist) const -> double;
  auto getTimeToFlyToFP(double dist_to_fp) const -> double;
  
  auto move(double dt) -> void;
  auto getAimPoint(double ammoHDist)-> pointmath::Point { return coord + dirXY * ammoHDist;};  
  auto droneStateToStr() const -> const char*;

};

} //eo core::