/*
 * TaskStop.cpp
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 08.09.2017
 *      Author: Marcel Honegger
 */

#include <cmath>
#include "RobotModel.h"
#include "Tool.h"
#include "Transformation.h"
#include "Motion.h"
#include "TaskStop.h"

using namespace std;

const float TaskStop::VELOCITY_THRESHOLD = 1.0e-6;

/**
 * Creates a task object that stops the current motion.
 * @param robotModel a reference to the robot model object to use.
 * @param tool the given tool to use for the calculation of the direct kinematic equations.
 */
TaskStop::TaskStop(RobotModel& robotModel, Tool& tool) : Task(robotModel, tool) {}

/**
 * Deletes the task object.
 */
TaskStop::~TaskStop() {}

/**
 * Initializes this task object.
 */
void TaskStop::init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {
    
    tool.set(this->tool);
}

/**
 * Increments the robots motion states in all coordinate spaces.
 */
int16_t TaskStop::increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period) {
    
    // stop the motion in joint angles
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) {
        
        phi[i].setLimits(robotModel.getProfileVelocity(i), robotModel.getProfileAcceleration(i), robotModel.getProfileDeceleration(i), robotModel.getProfileJerk(i));
        phi[i].incrementToVelocity(0.0f, period);
    }
    
    // calculate the motion states in other coordinate spaces
    
    robotModel.limitJointAngles(phi);
    robotModel.inverseKinematics(phi, q);
    if (robotModel.limitActuatorCoordinates(q)) {
        robotModel.directKinematics(q, phi);
    } else {
        // leave phi unchanged
    }
    robotModel.directKinematics(phi, tool, pose, velocity, acceleration);
    
    // check if motion has stopped
    
    bool motionHasStopped = true;
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) {
        
        if (fabs(phi[i].velocity) > VELOCITY_THRESHOLD) motionHasStopped = false;
    }
    
    return motionHasStopped ? DONE : RUNNING;
}

/**
 * Exits this task object.
 */
void TaskStop::exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {

    // set the target motion states to the current motion states

    poseTarget.set(pose);
    velocityTarget.set(velocity);
    accelerationTarget.set(acceleration);

    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) phiTarget[i].set(phi[i]);
    for (uint16_t i = 0; i < robotModel.getNumberOfActuators(); i++) qTarget[i].set(q[i]);
}
