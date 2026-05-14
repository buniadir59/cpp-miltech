#include "drone.hpp"
#include "point_math.hpp"
#include "ballistics.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

/* **** drone.hpp / drone.cpp 
  TargetState::
    update target position
    predict target position
  DroneConfig 
  Drone::
    update drone params //TODO   
    choose target //TODO
  */

namespace drone {

namespace {  //for helpers
    constexpr int kNtgts = 5;    //number of targets //TODO duplicates the same in simulation (((
    constexpr double eps = std::numeric_limits<double>::epsilon(); 

} //namespace for helpers


    void TargetState::update(pointmath::Point new_position, double time_s) { //update state with recent information about target location
        if (has_previous) {
            const double dt = time_s - last_known_s; //previous_time_s;
            velocity = (new_position - last_known) / dt;
        } else { //velocity stays 0
             has_previous = true;
        }

        last_known = new_position;
        last_known_s = time_s;
    }

    pointmath::Point TargetState::predictPositionAt(double future_t_s) const {
        return last_known + velocity * (future_t_s - last_known_s);
    }

    auto TargetState::calculateBallisticSolutionAt(double time_s, ballistics::BallisticsInput& input) {    
        pointmath::Point pos_at_time = predictPositionAt(time_s);
        input.target_x = pos_at_time.x;
        input.target_y = pos_at_time.y;
        dropRoute = ballistics::compute_drop_solution(input);
    }

    //set both simulteneously - dir & dirRad 
    void Drone::setDroneDirection (double aR) {
        dirRad = aR;
        dirXY = pointmath::cossin(dirRad.value);
    }

    /****
    * Оновити координати, швидкість та стан дрона відповідно до поточної кроку
    * NB! drone state is only changes when Accelerating or Decelerating on reaching attack speed or 0-speed accordingly
    */
    void Drone::moveDrone(double  dt) { 
        switch (state) {
        case STOPPED: //should not be the case after steering drone
          //  throw std::runtime_error("TODO");
            break; 
        case TURNING: { 
                double aval = angSpeed * dt;
                aval = turnClockwise ? dirRad.value - aval : dirRad.value + aval;
                setDroneDirection(aval);
                break;
            }
        case MOVING: //one step in the same direction 
            coord += dirXY * (attSpeed * dt);
            break;

        case ACCELERATING: { //increase speed. if attack speed, change state to Moving
                const double time_to_attack_speed = (attSpeed - speed) / kAcc;
                const double accel_dt = std::min(dt, time_to_attack_speed);
                const double remaining_dt = std::max(0.0, dt - accel_dt);

                double dist = (speed + kAcc * accel_dt / 2.0) * accel_dt;
                speed += kAcc * accel_dt;

                if (remaining_dt > eps || attSpeed - speed <= eps) {
                    speed = attSpeed;
                    state = MOVING;
                    dist += remaining_dt * attSpeed;
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
                
                if (step_dt < dt || speed <= eps) {
                    speed = 0.0;
                    state = STOPPED;
                }               
                break;    
            }    
        }
    }

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
    auto Drone::getBestTargetAt(double time_s)  -> int {     
        int bestTag{-1}; //there will be tag of the target with the least time to hit
        double bestTT{std::numeric_limits<double>::max()};
        ballistics::BallisticsInput input = {coord.x, coord.y, alt, 0, 0,
        attSpeed, accPath, ammoName};

        for (auto i=0; i < kNtgts; ++i) {            
            tgts[i].calculateBallisticSolutionAt(time_s, input); //time_s = current time or extended with leading time if we have an estimate
          
            const double total_time = getTotalTimeForDropRoute(coord, tgts[i].dropRoute);
            tgts[i].time_total = total_time;
            if (total_time < bestTT) {
                bestTT = total_time;
                bestTag = i;
            }
        }

        return bestTag;
    }  


    auto Drone::startMission(double accuracy_m) -> void {    // UNDER_CONSTR TODO
        if (state != STOPPED) {
            throw std::runtime_error("ERR_TODO");
        }

        ballistics::DropSolution& drop_route = tgts[currentTgtTag].dropRoute;
            
        if (drop_route.has_intermediate_point) {
            throw std::runtime_error("ERR_TODO");
        }

        destPoint = {drop_route.fire_x, drop_route.fire_y};
        destState = MOVING;

        double dist; 
        double angle;    
        trxPointToDistAngle(destPoint - coord, dist, angle); 

        if (std::abs(angle) > turnThrld) { //if angle over threshold => Turning, otherwise Accelerating
            state = TURNING;
            turnClockwise = angle < 0;
        } else {
            if (angle != 0.0) setDroneDirection(angle); //turn small angle
            state = ACCELERATING;
        }
        kAccuracy_m = accuracy_m; //TODO use time
        return;
    };

    /*****
    * returns distance in m between the point where ammo will hit the ground 
    * and target projected position at that time
    
    auto Drone::getAccuracyInM(double time_now) -> double {
        double hit_t_s = tgts[currentTgtTag].dropRoute.fall_time_s + time_now;
        pointmath::Point tgt_interpolated_coord = tgts[currentTgtTag].predictPositionAt(hit_t_s);
        pointmath::Point hit_coord = coord + dirXY * hitRad;
        double d = getLength(hit_coord - tgt_predicted_coord); //hypot
        return d; 
    }*/

    /****
    * Change drone state according to its direction and location re destination
    * turn on the move if needed
    * returns true  if fired
    */
    auto Drone::steerDrone(double time_now) -> bool {  // TODO   
        if (currentTgtTag == -1) { 
            if (state == STOPPED) {
                throw std::runtime_error(" Time to select new target - TODO!!!!!!"); 
            } else if (state == DECELERATING)  { //continue decelerating
                return false; //can happen if we missed the current target
            } else {
                throw std::runtime_error("ERR!!!!! No way it's happened!"); 
            }   
        }

        bool to_interim_point = destState != MOVING;
        if (to_interim_point) { //works yet to FP only
            throw std::runtime_error("ERR_TODO");
        } 

        double angle = pointmath::getAngle(tgts[currentTgtTag].predictPositionAt(time_now));
        double dist;
        trxPointToDistAngle(destPoint - coord, dist, angle);
    
        //if angle to target > 0 and < threshold steer at target on the move
        if (std::abs(angle - dirRad.value) < turnThrld) {
            setDroneDirection(angle);
        }  else { //current mission becomes invalid  if time to turn < m. total time, we still ok
            if (state != TURNING) { //have to stop and reevaluate targets 
                currentTgtTag = -1;
                if (state != STOPPED) state = DECELERATING; 
                return false;            
            }
        } 

        switch (state) {
        case STOPPED: //do nothing // ??? change to Turning as we are in the Interim P
            break; 

        case TURNING:    //if drone direction < threshold, change state to Accelerating
            if (angle == dirRad.value) state = ACCELERATING;
            break;
            
        case MOVING: { 
            // we are heaing to IPif (destState != MOVING) { TODO }

            //check if we are at firepoint
            if (dist < kAccuracy_m) { 
                return true; //fire!
          ////if yes, check if we are going to hit the target      if (getAccuracyInM() < hitRad) { } else { //if no hit expected, new mission would be needed 
            }
            //if we are not yet in FP, continue        
            break;
        }
        case ACCELERATING: // to FP or to IP
            //to IP  TODO   if (destState != MOVING) { }  //to FP
                
            //just continue Accelerating          
            break; 
        case DECELERATING:  // to IP  - TODO 
            break;       
        }
        return false;
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



