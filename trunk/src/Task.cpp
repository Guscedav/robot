/*
 * Task.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 05.10.2016
 *      Author: Marcel Honegger
 */

#include "RobotModel.h"
#include "Tool.h"
#include "Transformation.h"
#include "Motion.h"
#include "Task.h"

using namespace std;

/**
 * Creates a task object.
 * @param robotModel a reference to the robot model object to use.
 * @param tool the given tool to use for the calculation of the direct kinematic equations.
 */
Task::Task(RobotModel& robotModel, Tool& tool) : robotModel(robotModel) {
    
    this->tool.set(tool);
}

/**
 * Deletes the task object.
 */
Task::~Task() {}

/**
 * Initializes this task object.
 * @param tool the current tool. This tool object must be set to the tool given in the constructor of this object.
 * @param pose the robots actual pose in Cartesian coordinates.
 * @param velocity the robots actual velocity in Cartesian coordinates.
 * @param acceleration the robots actual acceleration in Cartesian coordinates.
 * @param phi an array of actual motion states of the joint angles.
 * @param q an array of actual motion states of the actuator coordinates.
 * @param poseTarget the robots planned target pose in Cartesian coordinates.
 * @param velocityTarget the robots planned target velocity in Cartesian coordinates.
 * @param accelerationTarget the robots planned target acceleration in Cartesian coordinates.
 * @param phiTarget an array of planned target motion states of the joint angles.
 * @param qTarget an array of planned target motion states of the actuator coordinates.
 */
void Task::init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {
    
    tool.set(this->tool);
}

/**
 * Increments the robots motion states in all coordinate spaces.
 * <br/><br/>
 * This method is called by the task sequencer periodically. It increments the states of
 * the cartesian pose, the joint angles and the actuator coordinates for a given time period.
 * @param pose the robots pose in Cartesian coordinates. This transformation needs to be incremented by this method.
 * @param velocity the robots velocity in Cartesian coordinates.
 * @param acceleration the robots acceleration in Cartesian coordinates.
 * @param phi an array of motion states of the joint angles.
 * These motion states need to be incremented by this method.
 * @param q an array of motion states of the actuator coordinates.
 * These motion states need to be incremented by this method.
 * @param velocityOverride an override parameter to scale the profile velocity with.
 * @param period the time period to increment the motion states for, given in [s].
 * @return the status of this task, i.e. FAULT, RUNNING or DONE.
 */
int16_t Task::increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period) {
    
    return FAULT;
}

/**
 * Exits this task object.
 * This method is called by the task sequencer after it initialized this task object
 * and after it called the <code>increment()</code> method for the last time.
 * @param pose the robots actual pose in Cartesian coordinates.
 * @param velocity the robots actual velocity in Cartesian coordinates.
 * @param acceleration the robots actual acceleration in Cartesian coordinates.
 * @param phi an array of actual motion states of the joint angles.
 * @param q an array of actual motion states of the actuator coordinates.
 * @param poseTarget the robots planned target pose in Cartesian coordinates.
 * @param velocityTarget the robots planned target velocity in Cartesian coordinates.
 * @param accelerationTarget the robots planned target acceleration in Cartesian coordinates.
 * @param phiTarget an array of planned target motion states of the joint angles.
 * @param qTarget an array of planned target motion states of the actuator coordinates.
 */
void Task::exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {}
