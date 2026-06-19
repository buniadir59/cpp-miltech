#pragma once

#include "math/angle_math.hpp"
#include "math/point_math.hpp"

namespace drone {

// position, speed, direction, direction at destination point, direction to target???
// drone config ?
struct DroneContext {
  // constant - obtained from input data:
  double alt;        // altitude
  double accPath;    // acceler path
  double attSpeed;   // attack speed
  double turnThrld;  // Пороговий кут для зупинки (рад)
  double angSpeed;   // Кутова швидкість повороту (рад/с)

  // constant - calculated once and saved for future use
  double kAcceleration;  // calculated from acceler time and attack speed

  // changing during simulation:
  double speed = 0.0;                  // current dr speed, m/s
  anglemath::AngleRad dirRad;          // напрямок дрона (радіани, від осі X) //initialized from input file
  pointmath::Point coord{};            // initialized from input file
  pointmath::Point dirXY{0.0, 0.0};    // direction by X and Y (as Point)  according to dirAngleRad
  anglemath::AngleRad destAngle{0.0};  // set under mission control

  DroneContext(double alt, pointmath::Point coord, double accPath, 
    double attSpeed, double turnThrld, 
    double angSpeed, double initDir)
    : alt(alt)
    , accPath(accPath)
    , attSpeed(attSpeed)
    , turnThrld(turnThrld)
    , angSpeed(angSpeed)
    , dirRad(initDir)
    , coord(coord)
  {
    // TODO calc other members: speed, 
    dirXY = pointmath::cossin(dirRad.value);
    kAcceleration = attSpeed * attSpeed / (2 * accPath);
  }

  
  void setDroneDirection(double aR) 
{
  dirRad = aR;
  dirXY = pointmath::cossin(dirRad.value);
};

}; //eo struct

}  // namespace dto