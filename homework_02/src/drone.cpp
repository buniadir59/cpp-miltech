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

    pointmath::Point getFP(const ballistics::DropSolution& drop_route) {
            return {drop_route.fire_x, drop_route.fire_y};
    }

} //eo namespace for helpers #####################

// TODO class private funcs ############
    auto Drone::getAmmoFlyTime() -> double { //TODO not used ?
        return  tgts[0].dropRoute.fall_time_s; //it does not depend on target, so we can take any
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

    auto Drone::calculateTimeForDropRoute(pointmath::Point start, TargetState& tgt)-> double {
        ballistics::DropSolution& drop_route = tgt.dropRoute;
        pointmath::Point pos_fire = {drop_route.fire_x, drop_route.fire_y};
        double total_time{};
        double time_to_interim{};

        if (tgt.dropRoute.has_intermediate_point) {
                pointmath::Point pos_int = {drop_route.intermediate_x, drop_route.intermediate_y};
                double dist, angle;
                pointmath::trxPointToDistAngle(pos_int - start, dist, angle);
                time_to_interim = getTurningTime(angle);
                time_to_interim += getTimeToFlyToInterimPoint(dist);
                total_time = time_to_interim;
                pointmath::trxPointToDistAngle(pos_fire - pos_int, dist, angle);
                total_time += getTimeToFlyToFirePoint(dist);
                total_time += getTurningTime(angle);
                tgt.time_to_interim = time_to_interim;
        } else {
                double dist, angle;
                pointmath::trxPointToDistAngle(pos_fire  - start, dist, angle);
                total_time = getTimeToFlyToFirePoint(dist);
                total_time += getTurningTime(angle);
        }
        tgt.time_accuracy = total_time - tgt.time_total; 
        tgt.time_total = total_time;
        return total_time;
    }

    // **** calculate best target tag based on previous total time astimation
    auto Drone::getBestTarget()  -> int {     
        int bestTag{-1}; //there will be tag of the target with the least time to hit
        double bestTT{std::numeric_limits<double>::max()};
        ballistics::BallisticsInput input = {coord.x, coord.y, alt, 0, 0,
        attSpeed, accPath, ammoName};
        double ammo_fall_time = getAmmoFlyTime();

        for (auto i=0; i < kNtgts; ++i) {     
            TargetState& tgt = tgts[i];       
            tgt.calculateBallisticSolutionAt(tgt.time_total + ammo_fall_time, input); 
          
            const double total_time = calculateTimeForDropRoute(coord, tgt);

            //TODO check time accuracy and maybe repeat calculations ???
            if (total_time < bestTT) {
                bestTT = total_time;
                bestTag = i;
            }
        }

        return bestTag;
    }  

// TODO class public funcs ############
    auto Drone::startNewMission(double time_step) -> int {
        kAccuracy_m = attSpeed * time_step / 2.0;

        int tgt_tag = getBestTarget();
        ballistics::DropSolution& drop_route = tgts[tgt_tag].dropRoute;  
        pointmath::Point dest = getDestination(drop_route);
        
        //TODO if no turn needed Stop shall be eliminated
        double angle_to_target = pointmath::getAngle(dest - coord);
        double delta_angle_to_tgt = angle_to_target - dirRad.value; //TODO we can have better target after turn, but for now we will not check it until we are on the way to destination
        delta_angle_to_tgt = pointmath::normalizeAngle(delta_angle_to_tgt); //normalize to [-pi, pi]  
        
        if (std::abs(delta_angle_to_tgt) <= turnThrld)  { //start new mission
            if (delta_angle_to_tgt > eps) setDroneDirection(dirRad.value + delta_angle_to_tgt); //turn small angle
            state = ACCELERATING;

        } else  { //need to stop first => check state

            if (!((state == STOPPED) || (state == TURNING))) { //wait till Stopped
                state = DECELERATING; // @start new mission
                mission.state = NONE;
                tgt_tag = -1; //no target, we will reevaluate after stop
                return tgt_tag;
            }   

            state = TURNING;
            mission.destAngle = angle_to_target; //TODO we can have better target after turn, but for now we will not check it until we are on the way to destination
        }
          
        if (drop_route.has_intermediate_point) {
            mission.state = TO_INTERIMP;
            mission.time_to_destination = tgts[tgt_tag].time_to_interim; 
        } else {
            mission.state = TO_FIREP;
            mission.time_to_destination = tgts[tgt_tag].time_total; 
        }
        mission.destPoint = dest;
        
        return tgt_tag;
    };

    /****
    * Change drone state according to its direction and location re destination
    * turn on the move if needed
    * returns mission status
    */
    auto Drone::continueMission(double t_step) -> MissionState {  // TODO     //NONE, TO_INTERIMP, TO_FIREP, FIRED, FAILED, COMPLETED
        mission.time_to_destination -= t_step; 

        switch (mission.state) {
        case TO_FIREP: {
            //reevaluate fire point and turn if needed, then check if we are at fire point to fire
            TargetState& currentTgt = tgts[mission.tgtTag];
            double ammo_f_dist = currentTgt.dropRoute.horizontal_fall_distance_m;
            double ammo_f_time = currentTgt.dropRoute.fall_time_s;
            double angle2tgt, dist2tgt;
            
            pointmath::Point tgt_lead_pos = currentTgt.getLeadPosition(mission.time_to_destination + ammo_f_time); 
            pointmath::Point dr2tgt = tgt_lead_pos - coord;
            pointmath::trxPointToDistAngle(dr2tgt, dist2tgt, angle2tgt);
            double delta2fp = dist2tgt - ammo_f_dist;

            if (delta2fp <= kAccuracy_m) { //we just missed the fire point, or we are close enough, 
                // so we can try to fire on the move  
                if (state == MOVING) { //(std::abs(delta2fp) < hitRad) { //fire!
                    mission.state = FIRED;
                    state = DECELERATING; //@continue mission
                    return FIRED;

                } else {
                    breakMission(); //mission failed, we will reevaluate target and try again
                    if (speed != 0.0) {
                        state = DECELERATING;  //@continue mission
                    } else state = STOPPED;
                    return NONE; //not hit, simulation is over TODO ???
                }
                
            } 
            //if we are not yet in FP, continue   
            mission.destAngle = angle2tgt; //update destination according to lead targeting
            mission.destPoint = coord + pointmath::cossin(angle2tgt) * delta2fp;  
            mission.time_to_destination = delta2fp /attSpeed; //update time to destination according to new distance to fire point and time to fly there
            if (std::abs(angle2tgt - dirRad.value) < turnThrld) { //steer at target on the move
                setDroneDirection(angle2tgt);
                if (state == TURNING) state = ACCELERATING; //???
            } else if (state == STOPPED) {
                state = TURNING;
            } else if (state != TURNING) { //we are moving, but need to turn, so stop and reevaluate
                if (state != STOPPED) state = DECELERATING;  //@continue mission
                breakMission(); //mission.state = NONE; mission.tgtTag = -1; //mission failed, we will reevaluate target and try again
                return NONE; 
            } 
                       
            }
            break;

        case TO_INTERIMP: breakMission();//TODO @continue mission
            break;

   default: //COMPLETED FAILED NONE
            throw std::runtime_error("ERR_TODO -mission state not implemented in continueMission: " 
                + mission.missionStateToStr()); //missionStateToStr(mission.state));
            break; //TODO
        }

        return mission.state;
    }
    

    auto Drone::getHitDistance(pointmath::Point tgt_pos_at_hit) -> double {
        pointmath::Point hit_coord = coord + dirXY * 
                                        tgts[mission.tgtTag].dropRoute.horizontal_fall_distance_m;  
        double hit_dist = pointmath::getLength(tgt_pos_at_hit - hit_coord);
        bool is_hit = hit_dist <= hitRad;
        if (is_hit) {
            mission.state = COMPLETED;
        } else {
           breakMission();
        }
        return hit_dist;
    }

 /****
    * Оновити координати, швидкість та стан дрона відповідно до поточної кроку
    * NB! if Turning, check if angle to destination is reached, if yes, change state to Accelerating,
    * NB! drone state is only changes when Accelerating or Decelerating on reaching attack speed or 0-speed accordingly
    */
    void Drone::moveDrone(double  dt) { 
        switch (state) {
        case STOPPED: //waiting the results of previous mission, no move, no state change
            break; 
        case TURNING: { 
                double dval = angSpeed * dt;
                pointmath::AngleRad dturn = mission.destAngle - dirRad;
                double delta_angle = std::abs(dturn.value);
                dval = std::min(dval, delta_angle); //do not turn more than needed  
                if ((delta_angle <= turnThrld) || (delta_angle <= dval)) { //turn is completed
                    setDroneDirection(mission.destAngle.value);
                    state = ACCELERATING;
                    /*if (mission.state == TO_INTERIMP) { !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        mission.state = TO_FIREP; //just completed turn to FP
                        mission.destPoint = getFP(tgts[mission.tgtTag].dropRoute);
                        mission.destAngle = 
                    } else if (mission.state == TO_FIREP) {
                         //start move to fire point
                    }*/
                    break;
                }
            
                setDroneDirection((dturn.value < 0.0) ? dirRad.value - dval : dirRad.value + dval);                            
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

    } // drone



