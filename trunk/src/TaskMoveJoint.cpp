/*
 * TaskMoveJoint.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.10.2016
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <algorithm>
#include "RobotModel.h"
#include "Tool.h"
#include "Transformation.h"
#include "Motion.h"
#include "TaskMoveJoint.h"

using namespace std;

const double TaskMoveJoint::MINIMUM_ZONE = 1.0e-6;

/**
 * Creates a task object that moves the robot to a given joint angle configuration.
 * @param robotModel a reference to the robot model object to use.
 * @param tool the given tool to use for the calculation of the direct kinematic equations.
 * @param phi an array of desired joint angles.
 * @param velocity the velocity at which to move the robot joints, given in [rad/s].
 * @param zone the minimum distance to the desired joint angles before the robot moves to the next task in the task list, given in [rad].
 * A zone parameter of 0.0 means that the robot moves precisely to the given joint angle configuration and stops,
 * before moving to the next task in the task list.
 */
TaskMoveJoint::TaskMoveJoint(RobotModel& robotModel, Tool& tool, const double phi[], float velocity, double zone) : Task(robotModel, tool) {
    
    this->phi = new double[robotModel.getNumberOfJoints()];
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) this->phi[i] = phi[i];
    robotModel.limitJointAngles(this->phi);
    
    this->velocity = velocity;
    this->zone = (zone < MINIMUM_ZONE) ? MINIMUM_ZONE : zone;
}

/**
 * Deletes the task object.
 */
TaskMoveJoint::~TaskMoveJoint() {
    
    delete[] phi;
}

/**
 * Initializes this task object.
 */
void TaskMoveJoint::init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {
    
    tool.set(this->tool);
}

/**
 * Increments the robots motion states in all coordinate spaces.
 */
int16_t TaskMoveJoint::increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period) {
    
    // scale and limit the desired profile velocity
    
    float profileVelocity[robotModel.getNumberOfJoints()];
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) {
        profileVelocity[i] = this->velocity*velocityOverride;
        if (profileVelocity[i] > robotModel.getProfileVelocity(i)) profileVelocity[i] = robotModel.getProfileVelocity(i);
    }
    
    // calculate the times to the desired joint angle positions for all joints
    
    float* time = new float[robotModel.getNumberOfJoints()];
    float maxTime = 1.0e-6f;
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) {
        
        phi[i].setLimits(profileVelocity[i], robotModel.getProfileAcceleration(i), robotModel.getProfileDeceleration(i), robotModel.getProfileJerk(i));
        time[i] = phi[i].getTimeToPosition(this->phi[i]);
        if (time[i] > maxTime) maxTime = time[i];
    }
    
    // move the joint angles to the desired position
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) {
        
        phi[i].setLimits(profileVelocity[i]*time[i]/maxTime, robotModel.getProfileAcceleration(i)*time[i]/maxTime, robotModel.getProfileDeceleration(i)*time[i]/maxTime, robotModel.getProfileJerk(i)*time[i]/maxTime);
        phi[i].incrementToPosition(this->phi[i], period);
    }
    
    delete[] time;
    
    // calculate the motion states in other coordinate spaces
    
    robotModel.limitJointAngles(phi);
    robotModel.inverseKinematics(phi, q);
    if (robotModel.limitActuatorCoordinates(q)) {
        robotModel.directKinematics(q, phi);
    } else {
        // leave phi unchanged
    }
    robotModel.directKinematics(phi, tool, pose, velocity, acceleration);
    
    // check if the desired joint angle configuration is reached
    
    double distance = 0.0;
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) distance = max(distance, fabs(phi[i].position-this->phi[i]));
    
    if (distance < zone) return DONE;
    else return RUNNING;
}

/**
 * Exits this task object.
 */
void TaskMoveJoint::exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {

    // set the target motion states of the joint angles

    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) {
        phiTarget[i].set(this->phi[i], 0.0f, 0.0f);
    }

    // calculate the target motion states in other coordinate spaces

    robotModel.inverseKinematics(phiTarget, qTarget);
    if (robotModel.limitActuatorCoordinates(qTarget)) {
        robotModel.directKinematics(qTarget, phiTarget);
    } else {
        // leave phiTarget unchanged
    }
    robotModel.directKinematics(phiTarget, tool, poseTarget, velocityTarget, accelerationTarget);
}
