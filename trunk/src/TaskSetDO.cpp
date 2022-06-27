/*
 * TaskSetDO.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.10.2016
 *      Author: Marcel Honegger
 */

#include "RobotModel.h"
#include "Tool.h"
#include "Transformation.h"
#include "Motion.h"
#include "DigitalOut.h"
#include "TaskSetDO.h"

using namespace std;

/**
 * Creates a task object that sets a digital output.
 * @param robotModel a reference to the robot model object to use.
 * @param tool the given tool to use for the calculation of the direct kinematic equations.
 * @param digitalOut a reference to a digital output.
 * @param value the value to set the digital output to.
 */
TaskSetDO::TaskSetDO(RobotModel& robotModel, Tool& tool, DigitalOut& digitalOut, bool value) : Task(robotModel, tool), digitalOut(digitalOut) {
    
    this->value = value;
}

/**
 * Deletes the task object.
 */
TaskSetDO::~TaskSetDO() {}

/**
 * Initializes this task object.
 */
void TaskSetDO::init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {
    
    tool.set(this->tool);
}

/**
 * Increments the robots motion states in all coordinate spaces.
 */
int16_t TaskSetDO::increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period) {
    
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
    
    // set the output and return
    
    digitalOut = value;
    
    return DONE;
}

/**
 * Exits this task object.
 */
void TaskSetDO::exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {}
