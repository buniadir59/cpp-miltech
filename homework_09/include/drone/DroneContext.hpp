#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "dto/BallisticResult.hpp"
#include "math/angle_math.hpp"
#include "math/point_math.hpp"

namespace dto {
struct Ammo;
}

namespace drone {

struct DroneContext {
  IBallisticSolver* solver = nullptr;
  const dto::Ammo* ammo = nullptr;

  // constant - obtained from input data:
  double alt;              // altitude
  double accPath;          // acceler path
  double attSpeed;         // attack speed
  double turnThrld;        // Пороговий кут для зупинки (рад)
  double angSpeed;         // Кутова швидкість повороту (рад/с)
  double kTimeStep = 0.0;  // for calculating time dependent values (turning, moving etc),set by mission processor
  // constant - calculated once and saved for future use
  double kAcceleration;  // calculated from acceler time and attack speed
  double kAccTime;
  double ammoBaseFFTime = 0.0;  // based on mconf altitude & attack speed
  double ammoBaseHDist = 0.0;   // based on mconf alt and attack speed

  // changing during simulation:
  double speed = 0.0;  // current dr speed, m/s
  double timeToGainAttSpeed;
  double distToGainAttSpeed = accPath;
  double timeToStop = 0.0;
  double distToStop = 0.0;

  anglemath::AngleRad dirRad;        // напрямок дрона (радіани, від осі X) //initialized from input file
  pointmath::Point coord{};          // initialized from input file
  pointmath::Point dirXY{0.0, 0.0};  // direction by X and Y (as Point)  according to dirAngleRad

  // where and how to go - set under mission control
  anglemath::AngleRad angleToTLpos{0.0};
  pointmath::Point destPoint;
  bool hasToTurn = false;  // destination is an interim point, otherwise firepoint
  double maxSpeed = 0.0;   // maximum speed before decelerating when moving to interim point

  dto::BallisticResult ballResult;

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
    , kTimeStep(time_step)
    , dirRad(initDir)
    , coord(coord)
  {
    dirXY = pointmath::cossin(dirRad.value);

    kAccTime = accPath * 2.0 / attSpeed;
    kAcceleration = attSpeed / kAccTime;
    // drone is idle
    angleToTLpos = dirRad;
    destPoint = coord;
    timeToGainAttSpeed = kAccTime;
    distToGainAttSpeed = accPath;
    timeToStop = 0.0;
    distToStop = 0.0;
  }

  auto setDestToFP(pointmath::Point dest) -> void;

  auto updateBasicAmmoRes() -> bool;  // for set solver and set ammo

  auto _updateSpeedDependentCtx() -> void;  // at the end of acc & decel execution
  void _setDir(double aR);                  // for exec turning & adjust dir
  auto _adjustDir() -> void;                // for exec mov/decel/accel

  // all below return true if completed
  auto execMoving() -> bool;
  auto execAccelerating() -> bool;
  auto execDecelerating() -> bool;
  auto execTurning() -> bool;

  auto getMinTimeToTurn(anglemath::AngleRad delta_angle, double time_on_move) const -> double;
  auto getTurnTime(anglemath::AngleRad delta) const -> double;

};  // eo struct

}  // namespace drone