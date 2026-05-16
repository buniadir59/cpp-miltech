#include "drone.hpp"
#include "point_math.hpp"
#include "ballistics.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

/* **** drone.hpp / drone.cpp 
  TargetState::
    predict target position for use in lead targeting 

  Mission::
    steer drone along drop path

  DroneConfig:: for initial data from input file 
  Drone::
    update drone params //TODO   
    choose target //TODO
  */

namespace drone {

namespace {  //for helpers ##########################################################

    pointmath::Point getDestination(const ballistics::DropSolution& drop_route) {
        if (drop_route.has_intermediate_point) {
            return {drop_route.intermediate_x, drop_route.intermediate_y};
        } else {
            return {drop_route.fire_x, drop_route.fire_y};
        }
    }


} //eo namespace for helpers #####################

// TODO class private funcs ############
    auto Drone::getAmmoFlyTime() -> double { //TODO not used ?
        return  currentTgt.dropRoute.fall_time_s;
    };

    /**** returns time to turn from  current drone direction to angle
    * returns 0 if absolute turn value is less than threshold  //
    */  
    auto Drone::getTurningTime(double angle) {
        pointmath::AngleRad delta = angle - dirRad; //it shall normalize the diff
        double absvalue = std::abs(delta.value);
        return absvalue < turnThrld ? 0.0  : absvalue / angSpeed;
    }
    
    auto Drone::getTimeToFlyToInterimPoint(double dist) { 
        //we assume, starting and final drone states are Stopped 
        double cruize_dist = dist - 2.0 * accPath;
    
        if (cruize_dist > eps) { 
            double cruizeT = cruize_dist / attSpeed;
            return cruizeT + 2.0 * kAccTime;
        } else {
            return 2.0 * std::sqrt(dist / kAcc);
        }
    }

    auto Drone::getTimeToFlyToFirePoint(double dist) {
        // we assume, starting drone state is Stopped, and and final - Moving
        const double cruizeDist = dist - accPath;
        if (cruizeDist > eps) {
            return cruizeDist / attSpeed + kAccTime;
        }
        return kAccTime;
    }

    auto Drone::getTotalTimeForDropRoute(pointmath::Point start, ballistics::DropSolution drop_route) {
        pointmath::Point pos_fire = {drop_route.fire_x, drop_route.fire_y};
        double total_time{};
        double time_to_interim{};

        if (drop_route.has_intermediate_point) {
                pointmath::Point pos_int = {drop_route.intermediate_x, drop_route.intermediate_y};
                double dist, angle;
                pointmath::trxPointToDistAngle(pos_int - start, dist, angle);
                time_to_interim = getTurningTime(angle);
                time_to_interim += getTimeToFlyToInterimPoint(dist);
                total_time = time_to_interim;
                pointmath::trxPointToDistAngle(pos_fire - pos_int, dist, angle);
                total_time += getTimeToFlyToFirePoint(dist);
                total_time += getTurningTime(angle);
        } else {
                double dist, angle;
                pointmath::trxPointToDistAngle(pos_fire  - start, dist, angle);
                total_time = getTimeToFlyToFirePoint(dist);
                total_time += getTurningTime(angle);
        }

        return total_time;
    }

    // **** calculate best target tag, assuming drone is stopped  
    auto Drone::getBestTargetAt(double delta_t)  -> int {     
        int bestTag{-1}; //there will be tag of the target with the least time to hit
        double bestTT{std::numeric_limits<double>::max()};
        ballistics::BallisticsInput input = {coord.x, coord.y, alt, 0, 0,
        attSpeed, accPath, ammoName};

        for (auto i=0; i < kNtgts; ++i) {            
            tgts[i].calculateBallisticSolutionAt(delta_t, input); //time_s = current time or extended with leading time if we have an estimate
          
            const double total_time = getTotalTimeForDropRoute(coord, tgts[i].dropRoute);
            tgts[i].time_total = total_time;
            if (total_time < bestTT) {
                bestTT = total_time;
                bestTag = i;
            }
        }

        return bestTag;
    }  

// TODO class public funcs ############
    auto Drone::startNewMission(double time_step) -> bool {
        kAccuracy_m = attSpeed * time_step / 2.0;

        int tgt_tag = getBestTargetAt(0.0);
        ballistics::DropSolution& drop_route = tgts[tgt_tag].dropRoute;   //currentTgt = ;
        pointmath::Point dest = getDestination(drop_route);
        

        //TODO if no turn needed Stop shall be eliminated
        double angle = pointmath::getAngle(dest - coord);    

        if (std::abs(angle) <= turnThrld)  { //start new mission
            if (angle != 0.0) setDroneDirection(angle); //turn small angle
            state = ACCELERATING;

        } else  { //need to stop first => check state

            if (!((state == STOPPED) || (state == TURNING))) { //wait till Stopped
                state = DECELERATING;
                return false;
            }   

            state = TURNING;
            turnClockwise = angle < 0;
        } 
          
        mission.tgtTag = tgt_tag;        
        mission.state = drop_route.has_intermediate_point ? TO_INTERIMP : TO_FIREP;
        mission.destPoint = dest;

        return true;
    };

    /****
    * Change drone state according to its direction and location re destination
    * turn on the move if needed
    * returns mission status
    */
    auto Drone::continueMission() -> MissionState {  // TODO   
        //NONE, TO_INTERIMP, TO_FIREP, FIRED, FAILED, COMPLETED
        switch (mission.state) {
        case TO_FIREP: {
            double angle = pointmath::getAngle(currentTgt.last_known);
            double dist = pointmath::getLength(mission.destPoint - coord);

            if (std::abs(angle - dirRad.value) < turnThrld) { //steer at target on the move
                setDroneDirection(angle);
                if (state == TURNING) state = ACCELERATING; //???
            }  /* TODO else { //current mission becomes invalid  if time to turn < m. total time, we still ok
            if (state != TURNING) { //have to stop and reevaluate targets 
                currentTgtTag = -1;
                if (state != STOPPED) state = DECELERATING; 
                return false;            
            }
            }*/

            switch (state) { //the only viable cases are Turning/Accelerating/Moving
                case TURNING:
                    if (angle == dirRad.value) state = ACCELERATING; //????
                break;
                case ACCELERATING:           //just continue Accelerating          
                break;
                case MOVING:  //check if we are at firepoint
                    if (dist < kAccuracy_m) {  //fire!
                        mission.state = FIRED;
                        state = DECELERATING;
                        return FIRED;
                    }
                    //if we are not yet in FP, continue    
                break; 
                default:;
            }
                       
            }
            break;
        case FIRED:
            //TODO if time to complete => //if yes, check if we are going to hit the target      if (getAccuracyInM() < hitRad) { } else { //if no hit expected, new mission would be needed 
            
            break;
        case TO_INTERIMP: //TODO
            break;

        case NONE: 
        case FAILED:
        case COMPLETED:
            throw std::runtime_error("ERR_TODO");
            break; //TODO
        }

        return mission.state;
    }

    auto Drone::isMissionSuccessful(pointmath::Point tgt_pos_at_hit) -> bool {
        pointmath::Point hit_coord = coord + dirXY * 
                                        currentTgt.dropRoute.horizontal_fall_distance_m;  
        double hit_dist = pointmath::getLength(tgt_pos_at_hit - hit_coord);
        return hit_dist <= hitRad;
    }

 /****
    * Оновити координати, швидкість та стан дрона відповідно до поточної кроку
    * NB! drone state is only changes when Accelerating or Decelerating on reaching attack speed or 0-speed accordingly
    */
    void Drone::moveDrone(double  dt) { 
        switch (state) {
        case STOPPED: //should not be the case after steering drone
            throw std::runtime_error("TODO Stopped at move");
            break; 
        case TURNING: { 
                double aval = angSpeed * dt;
                aval = turnClockwise ? dirRad.value - aval : dirRad.value + aval;
                setDroneDirection(aval);              
            } 
            break;
        case MOVING: //one step in the same direction 
            coord += dirXY * (attSpeed * dt);
            break;
        case ACCELERATING: { //increase speed. if attack speed, change state to Moving
                const double time_to_att_speed = (attSpeed - speed) / kAcc;
                const double acc_dt = std::min(dt, time_to_att_speed);

                double dist = (speed + kAcc * acc_dt / 2.0) * acc_dt;
                speed += kAcc * acc_dt;
                
                if (time_to_att_speed <= dt) {  //it is last accelerating step
                    speed = attSpeed;
                    state = MOVING;
                    dist += (dt - acc_dt) * attSpeed;
                }

                coord += dirXY * dist;
                break;
            }
        case DECELERATING: { //decrease speed. if 0, change state to Stopped
                const double time_to_stop = speed / kAcc;
                const double step_dt = std::min(dt, time_to_stop);
             
                const double dist = (speed - kAcc * step_dt / 2.0) * step_dt;
                coord += dirXY * dist;
                speed -= kAcc * step_dt;
                
                if (time_to_stop <= dt) {
                    speed = 0.0;
                    state = STOPPED;
                }               
                break;    
            }    
        }
    }

        //##############################################################################################
        const std::string& Drone::getDroneStateStr() const {
                static const std::string arrStateStr[] = { "STOPPED", "ACCELERATING", "DECELERATING", "TURNING", "MOVING",  "Unknown"};
                
                if (int(state) >= 5) {
                    return arrStateStr[5];
                } else {
                    return arrStateStr[state];
                }
        }     

    } // drone



