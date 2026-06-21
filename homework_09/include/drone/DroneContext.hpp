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

  double deltaTime = 0.0;  // for calculating time dependent values (turning, moving etc),set by mission processor

  // constant - calculated once and saved for future use
  double kAcceleration;  // calculated from acceler time and attack speed

  // changing during simulation:
  double speed = 0.0;                // current dr speed, m/s
  anglemath::AngleRad dirRad;        // напрямок дрона (радіани, від осі X) //initialized from input file
  pointmath::Point coord{};          // initialized from input file
  pointmath::Point dirXY{0.0, 0.0};  // direction by X and Y (as Point)  according to dirAngleRad
  // where and how to go - set under mission control
  anglemath::AngleRad destAngle{0.0};
  pointmath::Point destPoint;
  bool hasToTurn = false; //destination is an interim point, otherwise firepoint
  double maxSpeed = 0.0; //maximum speed before decelerating when moving to interim point
  double distToDest = 0.0;          
  
  anglemath::AngleRad deltaAngle{0.0};  // = anglemath::AngleRad(ctx.destAngle - ctx.dirRad);
  DroneContext(double alt,
               pointmath::Point coord,
               double accPath,
               double attSpeed,
               double turnThrld,
               double angSpeed,
               double initDir,
               double time_step)
    : alt(alt)
    , accPath(accPath)
    , attSpeed(attSpeed)
    , turnThrld(turnThrld)
    , angSpeed(angSpeed)
    , deltaTime(time_step)
    , dirRad(initDir)
    , coord(coord)
  {
    dirXY = pointmath::cossin(dirRad.value);
    kAcceleration = attSpeed * attSpeed / (2 * accPath);
    // drone is idle
    destAngle = dirRad;
    destPoint = coord;
  }

  void updateDestDistAndDeltaAngle()
  {
    distToDest = pointmath::getLength(destPoint - coord);
    deltaAngle = anglemath::AngleRad(destAngle - dirRad);
  }

  auto execMoving()-> bool;
  auto execAccelerating()-> bool; //return true if completed
  auto execDecelerating()-> bool; //return true if completed
  auto execTurning() -> bool;  

  void adjustDirection() { setDroneDirection(deltaAngle.value < 0.0 ? dirRad.value - turnThrld : dirRad.value + turnThrld); }

  void setDroneDirection(double aR)
  {
    dirRad = aR;
    dirXY = pointmath::cossin(dirRad.value);
  };

  auto getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double;
  auto getTurnTime(anglemath::AngleRad delta) const -> double;
  auto getTimeToGainAttackSpeed() const -> double { return speed == attSpeed ? 0.0 : (attSpeed - speed) / kAcceleration; }

  auto getTimeToFlyToFP(double dist_to_fp) const -> double;

  auto getTimeToFlyToInterimPoint(double dist) const -> double;

};  // eo struct

}  // namespace drone