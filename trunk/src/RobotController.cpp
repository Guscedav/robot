/*
 * RobotController.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 10.08.2016
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <algorithm>
#include "RobotModel.h"
#include "Motion.h"
#include "RobotController.h"

/**
 * Creates a robot controller object with a reference to a robot model.
 * @param robotModel a reference to a robot model this controller depends on.
 */
RobotController::RobotController(RobotModel& robotModel) : robotModel(robotModel) {
	
    qDesired = new Motion[robotModel.getNumberOfActuators()];
    qActual = new Motion[robotModel.getNumberOfActuators()];
    qError = new Motion[robotModel.getNumberOfActuators()];
    
    phiDesired = new Motion[robotModel.getNumberOfJoints()];
    phiActual = new Motion[robotModel.getNumberOfJoints()];
    phiError = new Motion[robotModel.getNumberOfJoints()];
}

/**
 * Deletes this object and releases all allocated resources.
 */
RobotController::~RobotController() {
    
    delete[] qDesired;
    delete[] qActual;
    delete[] qError;
    
    delete[] phiDesired;
    delete[] phiActual;
    delete[] phiError;
}

/**
 * Sets the tool of this robot controller to the given tool.
 * @param tool a reference to a tool to use.
 */
void RobotController::setTool(Tool& tool) {
    
    this->tool.set(tool);
}

/**
 * Gets the actual tool of this robot controller.
 * @param tool a reference to a tool that is set to the robot controllers tool.
 */
void RobotController::getTool(Tool& tool) {
    
    tool.set(this->tool);
}

/**
 * Sets the desired actuator coordinates.
 * This method is usually called by the task sequencer.
 * @param q an array of the desired motion values of actuators.
 */
void RobotController::setDesiredActuatorCoordinates(const Motion q[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfActuators(); i++) qDesired[i].set(q[i]);
}

/**
 * Gets the desired actuator coordinates.
 * @param q an array of motion values that is set to the desired actuator coordinates of this robot controller.
 */
void RobotController::getDesiredActuatorCoordinates(Motion q[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfActuators(); i++) q[i].set(qDesired[i]);
}

/**
 * Gets the actual actuator coordinates.
 * @param q an array of motion values that is set to the actual actuator coordinates of this robot controller.
 */
void RobotController::getActualActuatorCoordinates(Motion q[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfActuators(); i++) q[i].set(qActual[i]);
}

/**
 * Gets the tracking errors of the actuator coordinates.
 * @param q an array of motion values that is set to the tracking errors of the actuator coordinates.
 */
void RobotController::getActuatorCoordinateErrors(Motion q[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfActuators(); i++) q[i].set(qError[i]);
}

/**
 * Sets the desired joint angles.
 * This method is usually called by the task sequencer.
 * @param phi an array of the desired motion values of joints.
 */
void RobotController::setDesiredJointAngles(const Motion phi[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) phiDesired[i].set(phi[i]);
}

/**
 * Gets the desired joint angles.
 * @param phi an array of motion values that is set to the desired joint angles of this robot controller.
 */
void RobotController::getDesiredJointAngles(Motion phi[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) phi[i].set(phiDesired[i]);
}

/**
 * Gets the actual joint angles.
 * @param phi an array of motion values that is set to the actual joint angles of this robot controller.
 */
void RobotController::getActualJointAngles(Motion phi[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) phi[i].set(phiActual[i]);
}

/**
 * Gets the tracking errors of the joint angles.
 * @param phi an array of motion values that is set to the tracking errors of the joint angles.
 */
void RobotController::getJointAngleErrors(Motion phi[]) {
    
    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) phi[i].set(phiError[i]);
}

/**
 * Sets the desired Cartesian pose.
 * This method is usually called by the task sequencer.
 * @param pose a reference to a transformation object with the desired Cartesian pose.
 */
void RobotController::setDesiredCartesianPose(Transformation& pose) {
    
    this->poseDesired.set(pose);
}

/**
 * Gets the desired Cartesian pose.
 * @param pose a reference to a transformation object that is set to the desired Cartesian pose.
 */
void RobotController::getDesiredCartesianPose(Transformation& pose) {
    
    pose.set(this->poseDesired);
}

/**
 * Gets the actual Cartesian pose.
 * @param pose a reference to a transformation object that is set to the actual Cartesian pose.
 */
void RobotController::getActualCartesianPose(Transformation& pose) {
    
    pose.set(this->poseActual);
}

/**
 * Gets the tracking errors of the Cartesian pose.
 * @param pose a reference to a transformation object that is set to the tracking errors of the Cartesian pose.
 */
void RobotController::getCartesianPoseError(Transformation& pose) {
    
    pose.set(this->poseError);
}

/**
 * Sets the desired Cartesian velocity.
 * This method is usually called by the task sequencer.
 * @param velocity a reference to a transformation object with the desired Cartesian velocity.
 */
void RobotController::setDesiredCartesianVelocity(Transformation& velocity) {
    
    this->velocityDesired.set(velocity);
}

/**
 * Gets the desired Cartesian velocity.
 * @param velocity a reference to a transformation object that is set to the desired Cartesian velocity.
 */
void RobotController::getDesiredCartesianVelocity(Transformation& velocity) {
    
    velocity.set(this->velocityDesired);
}

/**
 * Gets the actual Cartesian velocity.
 * @param velocity a reference to a transformation object that is set to the actual Cartesian velocity.
 */
void RobotController::getActualCartesianVelocity(Transformation& velocity) {
    
    velocity.set(this->velocityActual);
}

/**
 * Gets the tracking errors of the Cartesian velocity.
 * @param velocity a reference to a transformation object that is set to the tracking errors of the Cartesian velocity.
 */
void RobotController::getCartesianVelocityError(Transformation& velocity) {
    
    velocity.set(this->velocityError);
}

/**
 * Sets the desired Cartesian acceleration.
 * This method is usually called by the task sequencer.
 * @param acceleration a reference to a transformation object with the desired Cartesian acceleration.
 */
void RobotController::setDesiredCartesianAcceleration(Transformation& acceleration) {
    
    this->accelerationDesired.set(acceleration);
}

/**
 * Gets the desired Cartesian acceleration.
 * @param acceleration a reference to a transformation object that is set to the desired Cartesian acceleration.
 */
void RobotController::getDesiredCartesianAcceleration(Transformation& acceleration) {
    
    acceleration.set(this->accelerationDesired);
}

/**
 * Gets the actual Cartesian acceleration.
 * @param acceleration a reference to a transformation object that is set to the actual Cartesian acceleration.
 */
void RobotController::getActualCartesianAcceleration(Transformation& acceleration) {
    
    acceleration.set(this->accelerationActual);
}

/**
 * Gets the tracking errors of the Cartesian acceleration.
 * @param acceleration a reference to a transformation object that is set to the tracking errors of the Cartesian acceleration.
 */
void RobotController::getCartesianAccelerationError(Transformation& acceleration) {
    
    acceleration.set(this->accelerationError);
}
