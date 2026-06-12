#include "Mission.hpp"
#include "dto/Target.hpp"
#include "TargetControl.hpp"
#include "DroneControl.hpp"
#include "math/point_math.hpp"
#include "math//angle_math.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "defines.hpp"

using Point = pointmath::Point;
using AngleRad = anglemath::AngleRad;

namespace core {

auto Mission::init(double time_st, DroneControl* drone_ptr, const dto::Ammo& ammo) -> void {
  drone = drone_ptr;
  time_step = time_st;
  kAccuracy_m = drone->attSpeed * time_st / 2.0;
  
  dropRoute = solver->solve(drone->coord, tgt_lead_pos, drone->alt, drone->attSpeed, drone->accPath, ammo); // get ballistic solution for the first time to be able to call short version
  ammoFlyTime = dropRoute.fall_time_s;
  ammoHorizDist = dropRoute.horizontal_fall_distance_m;
}


/****
 * Change drone state according to its direction and location re destination
 * turn on the move if needed
 * returns true  if fired
 */
auto Mission::continueMission() -> bool
{
  timer -= time_step;  //@cM
  if (timer < 0) { 
    timer = 0.0;
  }

  if (state == TO_INTERIMP) {
    if (calculateMission() == false) { //@cm
      breakMission();
      return false;
    };
  }

  dto::Target& target = currTgt->now;    

  switch (state) {
    case TO_FIREP:
      // reevaluate fire point and turn if needed, then check if we are at fire point to fire
      if ((missionResultCode = recalculateFPOntheRoute(target)) > 0) { //error
        breakMission();  //@cm // mission failed, we will reevaluate target and try again
        return false;
      }

      /*Дрон досяг fire point під час розгону/гальмування — не скидає бомбу, а перезапускає місію. Можливо
      надмірно суворо: можна було б чекати MOVING замість перезапуску.

      Або додати можливість скиду на довільній швидкості. Симулятор ДЗ3 це підтримує
      TODO! Але це нечесно! заряд не долетить - хммм, подивитись баллістичне рішення для іншої швидкості? */
      return missionResultCode == -1; //fired or continue
      break;

    case TO_INTERIMP: {
      if (drone->state == core::STOPPED) {  // at interim, turn to fire point
        state = TO_FIREP;
        destPoint = dropRoute.fire_p;
        destAngle = pointmath::getAngle(destPoint - drone->coord);
        drone->state = core::TURNING;  //@continue to fire point
        return false;
      }
      else if (drone->state == core::ACCELERATING) {   // check max speed
        if (maxSpeed < drone->attSpeed)  // otherwise continue acccelerating to interim until Moving
        {
          double next_speed = drone->speed + drone->kAcceleration * time_step;
          if ((drone->speed <= maxSpeed) && (next_speed > maxSpeed)) {
            // check if next step is better
            if ((maxSpeed - drone->speed) < (next_speed - maxSpeed)) {  // start decelerating
              drone->state = core::DECELERATING;
            }
          }
        }
      }
      else if (drone->state == core::MOVING) {  // check if we are at point to start decelerating
        double dist_to_dec = pointmath::getLength(decelerateAtPoint - drone->coord);
        if (dist_to_dec <= kAccuracy_m) {  // we just missed the interim point, or we are close enough,
          drone->state = core::DECELERATING;
        }
      }
      // if state == TURNING) - continue to interim until Accelerating at Move
      // if state == DECELERATING) - continue to interim  until Stopped
    } break;

    default:  
      throw std::runtime_error("ERR_TODO -mission state not implemented in continueMission: ");
  }

  return false;
}


// return -1 if fired, 0 if direct manuver to FP is to continue, or error code
auto Mission::recalculateFPOntheRoute(dto::Target& target) -> int
{
  int count = 0; double timeFPAccuracy = 0.0;
  while (++count < defines::kMaxRecalculations) {
    double timeToFP = recalculateTimeToFP(target);
    
    timeFPAccuracy = timeToFP - timer;
    timer = timeToFP;   
    if (std::abs(timeFPAccuracy) < currTgt->getAccuracyS(kAccuracy_m))
      break; //accuracy is ok
  }

  if (count == defines::kMaxRecalculations) { //mission impossible 
    DEBUG("MISSION IMPOSSIBLE! tt_fp=" << tt << " vs timer=" << timer);
    currTgt->state = core::UNREACHABLE;
    return 3;
  }

  double angle2tgt, dist2tgt;
  pointmath::trxPointToDistAngle(tgt_lead_pos - drone->coord, dist2tgt, angle2tgt);

  destAngle = angle2tgt;  // update destination according to lead targeting
  double delta2fp = dist2tgt - ammoHorizDist; 
    DEBUG("@RCFP: M>" 
    << missionStateToStr()  
    << "@ tmr=" << timer << "@ MdA>>" << destAngle 
    << "@MdP@" << destPoint << "@ FP@" << dropPoint  
    << "@LP@" << tgt_lead_pos 
    << "@c=" << count << "@delta2fp=" << delta2fp);

  if (std::abs(delta2fp) <= kAccuracy_m) {  // we just missed the fire point, or we are close enough,
    // so we can try to fire on the move
    if (drone->state != core::MOVING) {
     LOG("@at FP: not MOVING!@dist to FP=" << delta2fp << "@ tmr=" << timer);
      currTgt->state = core::UNREACHABLE;
      return 1;  // we will miss, because ammo ff time will be less than needed

    }
    else {  // fire! 
      drone->state = core::DECELERATING;  //@continue mission
      return -1; //fire

    }
  }
  else if (delta2fp < 0) {
    LOG("@Missed FP!@dist to FP=" << delta2fp);
    currTgt->state = core::UNREACHABLE;
    return 2; 
  }

  double time_fly_to_FP = drone->getTimeToFlyToFP(delta2fp);
  AngleRad deltaAngle = AngleRad(angle2tgt - drone->dirRad.value);
  double min_time_to_turn = drone->getMinTimeToTurn(deltaAngle, time_fly_to_FP, time_step);

  destPoint = drone->coord + pointmath::cossin(angle2tgt) * delta2fp;
  dropPoint = destPoint;

  if (min_time_to_turn > 0.0) {
    if (drone->speed > 0.0) {       // need to stop first, to turn
      drone->state = core::DECELERATING;  // TODO: calculate/test mode decelerate-accelerate to give more time to turn?
    }
    else {  // v=0, turn on the spot
      drone->state = core::TURNING;
    }
  }
  else {  // we can continue moving
    drone->state = drone->speed < drone->attSpeed ? core::ACCELERATING : core::MOVING;
  }

  if ((drone->state != core::TURNING) && (deltaAngle.value != 0.0)) {  // turn on the move if needed
    if (std::abs(deltaAngle.value) < drone->turnThrld) {
      drone->setDroneDirection(angle2tgt);  //@RCFP
    }
    else {
      drone->setDroneDirection((deltaAngle.value < 0.0) ? drone->dirRad.value - drone->turnThrld  //@RCFP
                          : drone->dirRad.value + drone->turnThrld);
    }
  }

/*   DEBUG("@RCFP: M>" 
    << missionStateToStr()  
    << "@ tt=" << timer << "@ MdA>>" << destAngle 
    << " MdP@" << destPoint << "@ FP@" << dropPoint  
    << "@ LP@" << tgt_lead_pos 
    << "@ c=" << count << "@delta2fp=" << delta2fp); */
  return 0; //we shall continue mission
}

auto Mission::solveDropRoute() -> void //wrapper for solve
{
    dropRoute = solver->solve(drone->coord, tgt_lead_pos);
    ammoFlyTime = dropRoute.fall_time_s;
    ammoHorizDist = dropRoute.horizontal_fall_distance_m;
}


auto Mission::recalculateTimeToFP(dto::Target& target) -> double { //used in cycle in recalculate FPOntheRoute
  tgt_lead_pos = getTargetLeadPosition(target, timer + ammoFlyTime);  //@rcfp
  double angle2tgt, dist2tgt;
  pointmath::trxPointToDistAngle(tgt_lead_pos - drone->coord, dist2tgt, angle2tgt);
  destAngle = angle2tgt;  // update destination according to lead targeting
  double delta2fp = dist2tgt - ammoHorizDist;
  double min_time_to_turn = 0.0;
  double time_fly_to_FP = delta2fp <= kAccuracy_m? 0.0 : drone->getTimeToFlyToFP(delta2fp);
  if (drone->state == core::TURNING) {
    min_time_to_turn = drone->getMinTimeToTurn(AngleRad(angle2tgt - drone->dirRad.value), time_fly_to_FP, time_step);
  }

  return  min_time_to_turn + time_fly_to_FP;
}


auto Mission::calculateTimeForDropRoute(const pointmath::Point& start) -> double
{
  const Point& pos_fire = dropRoute.fire_p;
  double total_time{};
  double time_to_interim{};

  if (dropRoute.has_intermediate_point) {
    double dist_to_interim = 0.0;
    double angle_to_interim = 0.0;
    pointmath::trxPointToDistAngle(dropRoute.interm_p - start, dist_to_interim, angle_to_interim);
  
    // check distance
    if (dist_to_interim <= kAccuracy_m) {  // we are practically at interim point, so proceed to FP from here
      dropRoute.has_intermediate_point = false;
    } else {
      time_to_interim = drone->getTurnTime(AngleRad(angle_to_interim - drone->dirRad.value));  // turn from current dr.dir to IP
      time_to_interim += drone->getTimeToFlyToInterimPoint(dist_to_interim);

      double dist_to_fire = 0.0;
      double angle_to_fire = 0.0;
      pointmath::trxPointToDistAngle(pos_fire - dropRoute.interm_p, dist_to_fire, angle_to_fire);

      AngleRad turn_between_legs = AngleRad(angle_to_fire - angle_to_interim);

      total_time = time_to_interim;
      total_time += drone->getTurnTime(turn_between_legs);
      total_time += drone->getTimeToFlyToFP(dist_to_fire);
    }
  }

  if (dropRoute.has_intermediate_point == false) {
    double dist, angle;
    pointmath::trxPointToDistAngle(pos_fire - start, dist, angle);
    total_time = drone->getTimeToFlyToFP(dist);
    double minTurnTime = drone->getMinTimeToTurn(AngleRad(angle - drone->dirRad.value), total_time, time_step);
    total_time += minTurnTime;  // turn from current dr.dir to FP
  }

  time_accuracy = total_time - time_total;
  time_total = total_time;
  return total_time;
}

//called at start mission
//solves ballistic task => drop route
//returns false if mission is impossible
auto Mission::calculateMissionDropeRoute(const dto::Target&  target) -> bool
{ 
  constexpr double kMinSpeedRatio = 0.8;

 // const double tgtSpeed = pointmath::getLength(target.velocity);    //@cmdr
  const double angleToTarget = pointmath::getAngle(target.position - drone->coord);
  const double projectedTgtSpeed = currTgt->speed * std::cos(angleToTarget);

  if (drone->attSpeed * kMinSpeedRatio < projectedTgtSpeed) {
    return false; 
  }

  tgt_lead_pos = getTargetLeadPosition(target, timer + ammoFlyTime); //@sm
  solveDropRoute(); // get ballistic solution
  int count = 0;
  double total_time = calculateTimeForDropRoute(drone->coord); //@cmdr
  double accuracy_s = currTgt->getAccuracyS(kAccuracy_m);
  if (currTgt->speed > defines::eps) {  // we shall not calculate time accuracy for still target
    // check time accuracy and maybe repeat calculations
    while (std::abs(time_accuracy) >accuracy_s  &&
             count < defines::kMaxRecalculations) {  // we are not accurate enough, so we can try to recalculate with new time estimation
        count++;
        
        tgt_lead_pos = getTargetLeadPosition(target, total_time + ammoFlyTime);
        solveDropRoute(); // @ cmdr 
        total_time = calculateTimeForDropRoute(drone->coord); //@cmdr
    }

    if (count > countMaxRecalc)
        countMaxRecalc = count;
  }

  DEBUG("@CMDR@ count=" << count << ' ' << missionStateToStr() 
         << "@ TPos=@" << target.position 
         << "@ Tv=" << currTgt->speed 
         << "@ ttfp=" << tt_fp << "@ vs tt=" << total_time); 

  return count < defines::kMaxRecalculations;
}


// cosine theorema for Vt*T, hitDist(H) alphT(A) and distDT(D): 
// distance to hit point: H = ammoHorDist +Vd*(T - ammFT)
//  sqrT*sqrVt - 2*Vt*T*D*cos(180-A) + sqrD = [Ahd +Vd*(T - Aft)]*[Ahd +Vd*(T - Aft)]
//                                          = sqrAhd + 2*Ahd*Vd*(T-Aft) + sqrVd(sqrT + sqrAft - 2*Aft*T) 
//                                          = sqrAhd - 2*Ahd*Vd*Aft + sqrVd*sqrAft   + 2*Ahd*Vd*T- 2*sqrVd*Aft*T + sqrVd*sqrT 
//                                          = 
// 0 = (sqrAhd - 2*Ahd*Vd*Aft + sqrVd*sqrAft - sqrDist) + T*2*(Ahd*Vd - sqrVd*Aft + Vt*Dist*cosA) + sqrT*(sqrVd-sqrVt)
// c = (sqrAhd - 2*Ahd*Vd*Aft + sqrVd*sqrAft - sqrDist)
// b = 2*(Ahd*Vd - sqrVd*Aft + Vt*Dist*cosA) 
// halfB = Ahd*Vd + Vt*Dist*cosA - sqrVd*Aft
// a = (sqrVd-sqrVt)
// D = sqr(b) - 4ac: => D = 4*(sqrHalfB - a*c)-> check if 4D>= 0, otherwize no catch
// quarterD = (sqrHalfB - a*c)
// if D = 0 => T = -halfB/a  NB! a & b should be of different sign
// T = -halfB/a  +- sqrt(quarterD)/a  
//    NB! assumed drone moves with attack speed
//  Return 0.0 if mission impossible  
auto Mission::calcTimeToFP() -> double   { //based on cosine theorema //TODO not used at the current implementation, option for the future
  double dist, angleVal;
  pointmath::trxPointToDistAngle(tgt_lead_pos - drone->coord, dist, angleVal);
  double Vt = pointmath::getLength(currTgt->now.velocity);
  double VdByAft = drone->attSpeed * ammoFlyTime;
  double a = drone->attSpeed * drone->attSpeed - Vt * Vt;
  double cosAByVt = Vt * std::cos(angleVal);
  double halfB = drone->attSpeed * (ammoHorizDist - VdByAft) + dist * cosAByVt;
  double c = ammoHorizDist * (ammoHorizDist - 2 * VdByAft) + VdByAft * VdByAft - dist * dist;
  double quarterD = halfB * halfB - a * c;
  if ((quarterD < 0) || (std::abs(a) < defines::eps)) {
    return 0.0; //mission impossible
  }
    
  double sqrtQD = std::sqrt(quarterD);
  double t1 = (sqrtQD - halfB) / a;
  double t2 = -(sqrtQD + halfB) / a;
  double t = fmax(t1, t2) - ammoFlyTime; // > t2 ? t1 : t2;

  if (cosAByVt >= 10.0) {
    DEBUG("@cosine theorema* tt="<< t <<" t1=" << t1 << " t2=" << t2 << "  Vt=" << Vt << " vt_proj=@" << cosAByVt);  
  }

  return t; //returns time for leat targeting
}

auto Mission::calculateMission() -> bool {

  const dto::Target& target = currTgt->now; 

  if (calculateMissionDropeRoute(target) == false)  {
    currTgt->state = core::UNREACHABLE;
    breakMission();
    return false;
  }       

  state = TO_FIREP; //initial
  dropPoint = dropRoute.fire_p; 

  if (dropRoute.has_intermediate_point == true) { // there is interim point, discard mission 
      currTgt->state = core::UNREACHABLE;     
      breakMission();
      return false;  
  } 

 //to  TO_FIREP
    destPoint = dropRoute.fire_p;
    destAngle = pointmath::getAngle(destPoint - drone->coord);
    timer = time_total - time_to_interim; 
    tgt_lead_pos = getTargetLeadPosition(target, timer + ammoFlyTime); //@snm
    if (recalculateFPOntheRoute(currTgt->now)) { //@snm
      currTgt->state = core::UNREACHABLE;
            breakMission();
      return false;
    }

    double ttt = drone->getMinTimeToTurn(destAngle, timer, time_step);

    if (ttt > 0.0) {
      if (!((drone->state == core::STOPPED) || (drone->state == core::TURNING))) {  
        drone->state = core::DECELERATING;                             // @snm
      } 
    } else {
      if (destAngle.value != 0.0) {
        drone->setDroneDirection((destAngle.value < 0.0) ? drone->dirRad.value - drone->turnThrld 
                          : drone->dirRad.value + drone->turnThrld);
      }  
      drone->state = drone->speed == drone->attSpeed ? core::MOVING : core::ACCELERATING; 
    }

    DEBUG( "@SNM " << missionStateToStr() << "@tmr=" << timer << "@MdA" << destAngle 
        << "@MdP@" << destPoint << "@FP@" << dropPoint
        << "@LP@" << tgt_lead_pos 
         );
    return true;

/* 
  //continue, there is interim point 
  anglemath::AngleRad delta_angle_to_dest = angleToInterim - drone->dirRad.value;
  destAngle = angleToInterim; //pointmath::getAngle(destPoint - drone->coord);

//  const double dist_to_interim = pointmath::getLength(destPoint - drone->coord);
  const Point dir_to_dest = pointmath::cossin(destAngle.value);
  
  if (std::abs(delta_angle_to_dest.value) <= drone->turnThrld) {  // start new mission
    drone->setDroneDirection(destAngle.value); // turn small angle
    drone->state = ACCELERATING;               // both, to interim or to fire point
  } else if (drone->state == STOPPED) {
    drone->state = TURNING;
  } else {
    drone->setDroneDirection((destAngle.value < 0.0) ? drone->dirRad.value - drone->turnThrld 
                          : drone->dirRad.value + drone->turnThrld);  // turn small angle
  }

  double ttt = drone->getMinTimeToTurn(delta_angle_to_dest, distToInterim / drone->attSpeed, time_step);
  drone->state = (ttt > 0.0) ? DECELERATING : ACCELERATING;

    if (distToInterim > 2.0 * drone->accPath) {
      maxSpeed = drone->attSpeed;
      decelerateAtPoint = destPoint - dir_to_dest * drone->accPath;
    }
    else {
      maxSpeed = std::sqrt(distToInterim * drone->kAcceleration);
      decelerateAtPoint = drone->coord + dir_to_dest * (distToInterim / 2.0);
    }
  
  timer = time_total;
  tgt_lead_pos = getTargetLeadPosition(target, timer + ammoFlyTime); //@sm

  DEBUG("@SNM I@Mtmr=" << timer << "@MdA" << destAngle 
        << "@MdP@" << destPoint << " FP@" << dropPoint <<"@LP@" << tgt_lead_pos 
       );
  return true; */

}

//target is selected (=current target tag)
//return false, if impossible to reach target
auto Mission::startNewMission(TargetControl& tgt) -> bool
{
  currTgt = &tgt;
  timer = 0; 
  missionResultCode = 0;
  return calculateMission(); //@snm

};




auto Mission::setSolver(IBallisticSolver* s)-> void {
    solver = s;
    solveDropRoute(); //to get ammo FF time an dist
}

} //eo namespace core