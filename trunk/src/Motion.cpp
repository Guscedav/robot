/*
 * Motion.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 13.01.2016
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <algorithm>
#include "Motion.h"

using namespace std;

const float Motion::DEFAULT_LIMIT = 1.0f;       // default value for limits
const float Motion::MINIMUM_LIMIT = 1.0e-6f;    // smallest value allowed for limits

/**
 * Creates a <code>Motion</code> object.
 * The values for position, velocity and acceleration are set to 0.
 */
Motion::Motion() {
    
    position = 0.0;
    velocity = 0.0f;
    acceleration = 0.0f;
    
    profileVelocity = DEFAULT_LIMIT;
    profileAcceleration = DEFAULT_LIMIT;
    profileDeceleration = DEFAULT_LIMIT;
    profileJerk = DEFAULT_LIMIT;
}

/**
 * Creates a <code>Motion</code> object with given values for position and velocity.
 * @param position the initial position value of this motion, given in [m] or [rad].
 * @param velocity the initial velocity value of this motion, given in [m/s] or [rad/s].
 */
Motion::Motion(double position, float velocity) {
    
    this->position = position;
    this->velocity = velocity;
    this->acceleration = 0.0f;
    
    profileVelocity = DEFAULT_LIMIT;
    profileAcceleration = DEFAULT_LIMIT;
    profileDeceleration = DEFAULT_LIMIT;
    profileJerk = DEFAULT_LIMIT;
}

/**
 * Creates a <code>Motion</code> object with given values for position, velocity and acceleration.
 * @param position the initial position value of this motion, given in [m] or [rad].
 * @param velocity the initial velocity value of this motion, given in [m/s] or [rad/s].
 * @param acceleration the initial acceleration value of this motion, given in [m/s&sup2;] or [rad/s&sup2;].
 */
Motion::Motion(double position, float velocity, float acceleration) {
    
    this->position = position;
    this->velocity = velocity;
    this->acceleration = acceleration;
    
    profileVelocity = DEFAULT_LIMIT;
    profileAcceleration = DEFAULT_LIMIT;
    profileDeceleration = DEFAULT_LIMIT;
    profileJerk = DEFAULT_LIMIT;
}

/**
 * Creates a <code>Motion</code> object with given values for position and velocity.
 * @param motion another <code>Motion</code> object to copy the values from.
 */
Motion::Motion(const Motion& motion) {
    
    position = motion.position;
    velocity = motion.velocity;
    acceleration = motion.acceleration;
    
    profileVelocity = motion.profileVelocity;
    profileAcceleration = motion.profileAcceleration;
    profileDeceleration = motion.profileDeceleration;
    profileJerk = motion.profileJerk;
}

Motion::~Motion() {}

/**
 * Sets the values for position and velocity.
 * @param position the desired position value of this motion, given in [m] or [rad].
 * @param velocity the desired velocity value of this motion, given in [m/s] or [rad/s].
 */
void Motion::set(double position, float velocity) {
    
    this->position = position;
    this->velocity = velocity;
}

/**
 * Sets the values for position, velocity and acceleration.
 * @param position the desired position value of this motion, given in [m] or [rad].
 * @param velocity the desired velocity value of this motion, given in [m/s] or [rad/s].
 * @param acceleration the desired acceleration value of this motion, given in [m/s&sup2;] or [rad/s&sup2;].
 */
void Motion::set(double position, float velocity, float acceleration) {
    
    this->position = position;
    this->velocity = velocity;
    this->acceleration = acceleration;
}

/**
 * Sets the values for position, velocity and acceleration.
 * @param motion another <code>Motion</code> object to copy the values from.
 */
void Motion::set(const Motion& motion) {
    
    position = motion.position;
    velocity = motion.velocity;
    acceleration = motion.acceleration;
}

/**
 * Sets the position value.
 * @param position the desired position value of this motion, given in [m] or [rad].
 */
void Motion::setPosition(double position) {
    
    this->position = position;
}

/**
 * Gets the position value.
 * @return the position value of this motion, given in [m] or [rad].
 */
double Motion::getPosition() {
    
    return position;
}

/**
 * Sets the velocity value.
 * @param velocity the desired velocity value of this motion, given in [m/s] or [rad/s].
 */
void Motion::setVelocity(float velocity) {
    
    this->velocity = velocity;
}

/**
 * Gets the velocity value.
 * @return the velocity value of this motion, given in [m/s] or [rad/s].
 */
float Motion::getVelocity() {
    
    return velocity;
}

/**
 * Sets the acceleration value.
 * @param acceleration the desired acceleration value of this motion, given in [m/s&sup2;] or [rad/s&sup2;].
 */
void Motion::setAcceleration(float acceleration) {
    
    this->acceleration = acceleration;
}

/**
 * Gets the acceleration value.
 * @return the acceleration value of this motion, given in [m/s&sup2;] or [rad/s&sup2;].
 */
float Motion::getAcceleration() {
    
    return acceleration;
}

/**
 * Sets the limit for the velocity value.
 * @param profileVelocity the limit of the velocity.
 */
void Motion::setProfileVelocity(float profileVelocity) {
    
    if (profileVelocity > MINIMUM_LIMIT) this->profileVelocity = profileVelocity; else this->profileVelocity = MINIMUM_LIMIT;
}

/**
 * Sets the limit for the acceleration value.
 * @param profileAcceleration the limit of the acceleration.
 */
void Motion::setProfileAcceleration(float profileAcceleration) {
    
    if (profileAcceleration > MINIMUM_LIMIT) this->profileAcceleration = profileAcceleration; else this->profileAcceleration = MINIMUM_LIMIT;
}

/**
 * Sets the limit for the deceleration value.
 * @param profileDeceleration the limit of the deceleration.
 */
void Motion::setProfileDeceleration(float profileDeceleration) {
    
    if (profileDeceleration > MINIMUM_LIMIT) this->profileDeceleration = profileDeceleration; else this->profileDeceleration = MINIMUM_LIMIT;
}

/**
 * Sets the limit for the jerk value.
 * @param profileJerk the limit of the jerk.
 */
void Motion::setProfileJerk(float profileJerk) {
    
    if (profileJerk > MINIMUM_LIMIT) this->profileJerk = profileJerk; else this->profileJerk = MINIMUM_LIMIT;
}

/**
 * Sets the limits for velocity, acceleration and deceleration values.
 * @param profileVelocity the limit of the velocity.
 * @param profileAcceleration the limit of the acceleration.
 * @param profileDeceleration the limit of the deceleration.
 */
void Motion::setLimits(float profileVelocity, float profileAcceleration, float profileDeceleration) {
    
    if (profileVelocity > MINIMUM_LIMIT) this->profileVelocity = profileVelocity; else this->profileVelocity = MINIMUM_LIMIT;
    if (profileAcceleration > MINIMUM_LIMIT) this->profileAcceleration = profileAcceleration; else this->profileAcceleration = MINIMUM_LIMIT;
    if (profileDeceleration > MINIMUM_LIMIT) this->profileDeceleration = profileDeceleration; else this->profileDeceleration = MINIMUM_LIMIT;
}

/**
 * Sets the limits for velocity, acceleration, deceleration and jerk values.
 * @param profileVelocity the limit of the velocity.
 * @param profileAcceleration the limit of the acceleration.
 * @param profileDeceleration the limit of the deceleration.
 * @param profileJerk the limit of the jerk.
 */
void Motion::setLimits(float profileVelocity, float profileAcceleration, float profileDeceleration, float profileJerk) {
    
    if (profileVelocity > MINIMUM_LIMIT) this->profileVelocity = profileVelocity; else this->profileVelocity = MINIMUM_LIMIT;
    if (profileAcceleration > MINIMUM_LIMIT) this->profileAcceleration = profileAcceleration; else this->profileAcceleration = MINIMUM_LIMIT;
    if (profileDeceleration > MINIMUM_LIMIT) this->profileDeceleration = profileDeceleration; else this->profileDeceleration = MINIMUM_LIMIT;
    if (profileJerk > MINIMUM_LIMIT) this->profileJerk = profileJerk; else this->profileJerk = MINIMUM_LIMIT;
}

/**
 * Gets the time needed to move to a given target position.
 * @param targetPosition the desired target position given in [m] or [rad].
 * @return the time to move to the target position, given in [s].
 */
float Motion::getTimeToPosition(double targetPosition) {
    
    // calculate position, when velocity is reduced to zero
    
    double stopPosition = (velocity > 0.0f) ? position+static_cast<double>(velocity*velocity/profileDeceleration*0.5f) : position-static_cast<double>(velocity*velocity/profileDeceleration*0.5f);
    
    if (targetPosition > stopPosition) { // positive velocity required
        
        if (velocity > profileVelocity) { // slow down to profile velocity first
            
            float t1 = (velocity-profileVelocity)/profileDeceleration;
            float t2 = static_cast<float>(targetPosition-stopPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            return t1+t2+t3;
            
        } else if (velocity > 0.0f) { // speed up to profile velocity
            
            float t1 = (profileVelocity-velocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = (static_cast<float>(targetPosition-position)-(velocity+profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float maxVelocity = sqrt((2.0f*static_cast<float>(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (maxVelocity-velocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = maxVelocity/profileDeceleration;
            }
            
            return t1+t2+t3;
            
        } else { // slow down to zero first, and then speed up to profile velocity
            
            float t1 = -velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = (static_cast<float>(targetPosition-position)-velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float maxVelocity = sqrt((2.0f*static_cast<float>(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = maxVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = maxVelocity/profileDeceleration;
            }
            
            return t1+t2+t3+t4;
        }
        
    } else { // negative velocity required
        
        if (velocity < -profileVelocity) { // slow down to (negative) profile velocity first
            
            float t1 = (-profileVelocity-velocity)/profileDeceleration;
            float t2 = static_cast<float>(stopPosition-targetPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            return t1+t2+t3;
            
        } else if (velocity < 0.0f) { // speed up to (negative) profile velocity
            
            float t1 = (velocity+profileVelocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = (static_cast<float>(position-targetPosition)+(velocity-profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*static_cast<float>(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (velocity-minVelocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = -minVelocity/profileDeceleration;
            }
            
            return t1+t2+t3;
            
        } else { // slow down to zero first, and then speed up to (negative) profile velocity
            
            float t1 = velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = (-static_cast<float>(targetPosition-position)+velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*static_cast<float>(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = -minVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = -minVelocity/profileDeceleration;
            }
            
            return t1+t2+t3+t4;
        }
    }
}

/**
 * Gets the distance moved until the velocity reaches zero.
 * @return the distance to the stop position.
 */
double Motion::getDistanceToStop() {
    
    return static_cast<double>(velocity*velocity/profileDeceleration*0.5f);
}

/**
 * Increments the current motion towards a given target velocity.
 * @param targetVelocity the desired target velocity given in [m/s] or [rad/s].
 * @param period the time period to increment the motion values for, given in [s].
 */
void Motion::incrementToVelocity(float targetVelocity, float period) {
    
    if (targetVelocity < -profileVelocity) targetVelocity = -profileVelocity;
    else if (targetVelocity > profileVelocity) targetVelocity = profileVelocity;
    
    if (targetVelocity > 0.0f) {
        
        if (velocity > targetVelocity) { // slow down to target velocity
            
            float t1 = (velocity-targetVelocity)/profileDeceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
                acceleration = -profileDeceleration;
            } else {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            }
            
        } else if (velocity > 0.0f) { // speed up to target velocity
            
            float t1 = (targetVelocity-velocity)/profileAcceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity+profileAcceleration*0.5f*period)*period);
                velocity += profileAcceleration*period;
                acceleration = profileAcceleration;
            } else {
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            }
            
        } else { // slow down to zero first, and then speed up to target velocity
            
            float t1 = -velocity/profileDeceleration;
            float t2 = targetVelocity/profileAcceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
                acceleration = profileDeceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>((velocity+profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += profileAcceleration*(period-t1);
                acceleration = profileAcceleration;
            } else {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += static_cast<double>(velocity*(period-t1-t2));
                acceleration = 0.0f;
            }
        }
        
    } else {
        
        if (velocity < targetVelocity) { // slow down to (negative) target velocity
            
            float t1 = (targetVelocity-velocity)/profileDeceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
                acceleration = profileDeceleration;
            } else {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            }
            
        } else if (velocity < 0.0f) { // speed up to (negative) target velocity
            
            float t1 = (velocity-targetVelocity)/profileAcceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity-profileAcceleration*0.5f*period)*period);
                velocity += -profileAcceleration*period;
                acceleration = -profileAcceleration;
            } else {
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            }
            
        } else { // slow down to zero first, and then speed up to (negative) target velocity
            
            float t1 = velocity/profileDeceleration;
            float t2 = -targetVelocity/profileAcceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
                acceleration = -profileDeceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>((velocity-profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += -profileAcceleration*(period-t1);
                acceleration = -profileAcceleration;
            } else {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += static_cast<double>(velocity*(period-t1-t2));
                acceleration = 0.0f;
            }
        }
    }
}

/**
 * Increments the current motion towards a given target position.
 * @param targetPosition the desired target position given in [m] or [rad].
 * @param period the time period to increment the motion values for, given in [s].
 */
void Motion::incrementToPosition(double targetPosition, float period) {
    
    // calculate position, when velocity is reduced to zero
    
    double stopPosition = (velocity > 0.0f) ? position+static_cast<double>(velocity*velocity/profileDeceleration*0.5f) : position-static_cast<double>(velocity*velocity/profileDeceleration*0.5f);
    
    if (targetPosition > stopPosition) { // positive velocity required
        
        if (velocity > profileVelocity) { // slow down to profile velocity first
            
            float t1 = (velocity-profileVelocity)/profileDeceleration;
            float t2 = static_cast<float>(targetPosition-stopPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
                acceleration = -profileDeceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            } else if (t1+t2+t3 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity-profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += -profileDeceleration*(period-t1-t2);
                acceleration = -profileDeceleration;
            } else {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t3)*t3);
                velocity += -profileDeceleration*t3;
                acceleration = 0.0f;
            }
            
        } else if (velocity > 0.0f) { // speed up to profile velocity
            
            float t1 = (profileVelocity-velocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = (static_cast<float>(targetPosition-position)-(velocity+profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float maxVelocity = sqrt((2.0f*static_cast<float>(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (maxVelocity-velocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = maxVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += static_cast<double>((velocity+profileAcceleration*0.5f*period)*period);
                velocity += profileAcceleration*period;
                acceleration = profileAcceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            } else if (t1+t2+t3 > period) {
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity-profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += -profileDeceleration*(period-t1-t2);
                acceleration = -profileDeceleration;
            } else {
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t1)*t1);
                velocity += profileAcceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t3)*t3);
                velocity += -profileDeceleration*t3;
                acceleration = 0.0f;
            }
            
        } else { // slow down to zero first, and then speed up to profile velocity
            
            float t1 = -velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = (static_cast<float>(targetPosition-position)-velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float maxVelocity = sqrt((2.0f*static_cast<float>(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = maxVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = maxVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
                acceleration = profileDeceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>((velocity+profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += profileAcceleration*(period-t1);
                acceleration = profileAcceleration;
            } else if (t1+t2+t3 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += static_cast<double>(velocity*(period-t1-t2));
                acceleration = 0.0f;
            } else if (t1+t2+t3+t4 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += static_cast<double>(velocity*t3);
                position += static_cast<double>((velocity-profileDeceleration*0.5f*(period-t1-t2-t3))*(period-t1-t2-t3));
                velocity += -profileDeceleration*(period-t1-t2-t3);
                acceleration = -profileDeceleration;
            } else {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>((velocity+profileAcceleration*0.5f*t2)*t2);
                velocity += profileAcceleration*t2;
                position += static_cast<double>(velocity*t3);
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t4)*t4);
                velocity += -profileDeceleration*t4;
                acceleration = 0.0f;
            }
        }
        
    } else { // negative velocity required
        
        if (velocity < -profileVelocity) { // slow down to (negative) profile velocity first
            
            float t1 = (-profileVelocity-velocity)/profileDeceleration;
            float t2 = static_cast<float>(stopPosition-targetPosition)/profileVelocity;
            float t3 = profileVelocity/profileDeceleration;
            
            if (t1 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*period)*period);
                velocity += profileDeceleration*period;
                acceleration = profileDeceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            } else if (t1+t2+t3 > period) {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity+profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += profileDeceleration*(period-t1-t2);
                acceleration = profileDeceleration;
            } else {
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t1)*t1);
                velocity += profileDeceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t3)*t3);
                velocity += profileDeceleration*t3;
                acceleration = 0.0f;
            }
            
        } else if (velocity < 0.0f) { // speed up to (negative) profile velocity
            
            float t1 = (velocity+profileVelocity)/profileAcceleration;
            float t3 = profileVelocity/profileDeceleration;
            float t2 = (static_cast<float>(position-targetPosition)+(velocity-profileVelocity)*0.5f*t1)/profileVelocity-0.5f*t3;
            
            if (t2 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*static_cast<float>(targetPosition-position)*profileAcceleration+velocity*velocity)*profileDeceleration/(profileAcceleration+profileDeceleration));
                t1 = (velocity-minVelocity)/profileAcceleration;
                t2 = 0.0f;
                t3 = -minVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += static_cast<double>((velocity-profileAcceleration*0.5f*period)*period);
                velocity += -profileAcceleration*period;
                acceleration = -profileAcceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += static_cast<double>(velocity*(period-t1));
                acceleration = 0.0f;
            } else if (t1+t2+t3 > period) {
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity+profileDeceleration*0.5f*(period-t1-t2))*(period-t1-t2));
                velocity += profileDeceleration*(period-t1-t2);
                acceleration = profileDeceleration;
            } else {
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t1)*t1);
                velocity += -profileAcceleration*t1;
                position += static_cast<double>(velocity*t2);
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t3)*t3);
                velocity += profileDeceleration*t3;
                acceleration = 0.0f;
            }
            
        } else { // slow down to zero first, and then speed up to (negative) profile velocity
            
            float t1 = velocity/profileDeceleration;
            float t2 = profileVelocity/profileAcceleration;
            float t4 = profileVelocity/profileDeceleration;
            float t3 = (-static_cast<float>(targetPosition-position)+velocity*0.5f*t1)/profileVelocity-0.5f*(t2+t4);
            
            if (t3 < 0.0f) {
                float minVelocity = -sqrt((-2.0f*static_cast<float>(targetPosition-position)*profileDeceleration+velocity*velocity)*profileAcceleration/(profileAcceleration+profileDeceleration));
                t2 = -minVelocity/profileAcceleration;
                t3 = 0.0f;
                t4 = -minVelocity/profileDeceleration;
            }
            
            if (t1 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*period)*period);
                velocity += -profileDeceleration*period;
                acceleration = -profileDeceleration;
            } else if (t1+t2 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>((velocity-profileAcceleration*0.5f*(period-t1))*(period-t1));
                velocity += -profileAcceleration*(period-t1);
                acceleration = -profileAcceleration;
            } else if (t1+t2+t3 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += static_cast<double>(velocity*(period-t1-t2));
                acceleration = 0.0f;
            } else if (t1+t2+t3+t4 > period) {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += static_cast<double>(velocity*t3);
                position += static_cast<double>((velocity+profileDeceleration*0.5f*(period-t1-t2-t3))*(period-t1-t2-t3));
                velocity += profileDeceleration*(period-t1-t2-t3);
                acceleration = profileDeceleration;
            } else {
                position += static_cast<double>((velocity-profileDeceleration*0.5f*t1)*t1);
                velocity += -profileDeceleration*t1;
                position += static_cast<double>((velocity-profileAcceleration*0.5f*t2)*t2);
                velocity += -profileAcceleration*t2;
                position += static_cast<double>(velocity*t3);
                position += static_cast<double>((velocity+profileDeceleration*0.5f*t4)*t4);
                velocity += profileDeceleration*t4;
                acceleration = 0.0f;
            }
        }
    }
}
