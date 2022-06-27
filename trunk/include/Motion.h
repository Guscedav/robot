/*
 * Motion.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.01.2016
 *      Author: Marcel Honegger
 */

#ifndef MOTION_H_
#define MOTION_H_

#include <cstdlib>

/**
 * This class keeps the motion values <code>position</code>, <code>velocity</code> and
 * <code>acceleration</code>, and offers methods to increment these values towards a
 * desired target position or velocity.
 * <br/>
 * To increment the current motion values, this class uses a simple 2nd order motion planner.
 * This planner calculates the motion to the target position or velocity with the various motion
 * phases, based on given limits for the profile velocity, acceleration and deceleration.
 * <br/>
 * The following figures show the different profile types used for planning a trajectory from
 * the current actual velocity to a given target velocity.
 * <img src="motion1.png" width="570" style="text-align:center"/>
 * <br/>
 * <div style="text-align:center"><b>The profile types used for planning a trajectory to a velocity</b></div>
 * The following figures show the different profile types used for planning a trajectory from
 * the current actual position and velocity to a given target position.
 * <img src="motion2.png" width="725" style="text-align:center"/>
 * <br/>
 * <div style="text-align:center"><b>The profile types used for planning a trajectory to a position</b></div>
 * <br/>
 * Note that the trajectory is calculated every time the motion state is incremented.
 * This allows to change the target position or velocity, as well as the limits for profile
 * velocity, acceleration and deceleration at any time.
 */
class Motion {
    
    public:
        
        double      position;       /**< The position value of this motion, given in [m] or [rad]. */
        float       velocity;       /**< The velocity value of this motion, given in [m/s] or [rad/s]. */
        float		acceleration;	/**< The acceleration value of this motion, given in [m/s&sup2;] or [rad/s&sup2;]. */
        
                    Motion();
                    Motion(double position, float velocity);
                    Motion(double position, float velocity, float acceleration);
                    Motion(const Motion& motion);
        virtual     ~Motion();
        void        set(double position, float velocity);
        void        set(double position, float velocity, float acceleration);
        void        set(const Motion& motion);
        void        setPosition(double position);
        double      getPosition();
        void        setVelocity(float velocity);
        float       getVelocity();
        void        setAcceleration(float acceleration);
        float       getAcceleration();
        void        setProfileVelocity(float profileVelocity);
        void        setProfileAcceleration(float profileAcceleration);
        void        setProfileDeceleration(float profileDeceleration);
        void        setProfileJerk(float profileJerk);
        void        setLimits(float profileVelocity, float profileAcceleration, float profileDeceleration);
        void        setLimits(float profileVelocity, float profileAcceleration, float profileDeceleration, float profileJerk);
        float       getTimeToPosition(double targetPosition);
        double      getDistanceToStop();
        void        incrementToVelocity(float targetVelocity, float period);
        void        incrementToPosition(double targetPosition, float period);
        
    private:
        
        static const float  DEFAULT_LIMIT;  // default value for limits
        static const float  MINIMUM_LIMIT;  // smallest value allowed for limits
        
        float       profileVelocity;
        float       profileAcceleration;
        float       profileDeceleration;
        float       profileJerk;
};

#endif /* MOTION_H_ */
