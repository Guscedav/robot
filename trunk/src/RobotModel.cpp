/*
 * RobotModel.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 21.01.2016
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <algorithm>
#include "Tool.h"
#include "Transformation.h"
#include "Motion.h"
#include "RobotModel.h"

using namespace std;

/**
 * Creates a robot model object with a given number of joints.
 * It is assumed that the number of actuators is identical to the number of joints.
 * @param numberOfJoints the number of joints this robot model consists of.
 */
RobotModel::RobotModel(uint16_t numberOfJoints) {
    
    this->numberOfActuators = numberOfJoints;
    this->numberOfJoints = numberOfJoints;
}

/**
 * Creates a robot model object with given numbers of actuators and joints.
 * @param numberOfActuators the number of actuators this robot model consists of.
 * @param numberOfJoints the number of joints this robot model consists of.
 */
RobotModel::RobotModel(uint16_t numberOfActuators, uint16_t numberOfJoints) {
    
    this->numberOfActuators = numberOfActuators;
    this->numberOfJoints = numberOfJoints;
}

/**
 * Deletes this object and releases all allocated resources.
 */
RobotModel::~RobotModel() {}

/**
 * Gets the number of actuators.
 * @return the number of actuators this robot model consists of.
 */
uint16_t RobotModel::getNumberOfActuators() {

    return numberOfActuators;
}

/**
 * Gets the number of joints.
 * @return the number of joints this robot model consists of.
 */
uint16_t RobotModel::getNumberOfJoints() {

    return numberOfJoints;
}

/**
 * Gets the limit of the robots velocity.
 * @return the limit of the velocity.
 */
float RobotModel::getProfileVelocity() {
    
    return 0.0f;
}

/**
 * Gets the limit of the robots translational velocity.
 * The default implementation of this method returns the result
 * of the <code>getProfileVelocity()</code> method.
 * If a specific robot model needs to define a maximum velocity
 * for translational motion, this method must be implemented.
 * @return the limit of the velocity, given in [m/s].
 */
float RobotModel::getTranslationalProfileVelocity() {
    
    return getProfileVelocity();
}

/**
 * Gets the limit of the robots rotational velocity.
 * The default implementation of this method returns the result
 * of the <code>getProfileVelocity()</code> method.
 * If a specific robot model needs to define a maximum velocity
 * for rotational motion, this method must be implemented.
 * @return the limit of the velocity, given in [rad/s].
 */
float RobotModel::getRotationalProfileVelocity() {
    
    return getProfileVelocity();
}

/**
 * Gets the limit of the robots joint velocity.
 * The default implementation of this method returns the result
 * of the <code>getProfileVelocity()</code> method.
 * If a specific robot model needs to define maximum velocities
 * for each joint, this method must be implemented.
 * @param joint the joint to get the velocity limit for.
 * @return the limit of the velocity.
 */
float RobotModel::getProfileVelocity(uint16_t joint) {
    
    return getProfileVelocity();
}

/**
 * Gets the limit of the robots acceleration.
 * @return the limit of the acceleration.
 */
float RobotModel::getProfileAcceleration() {
    
    return 0.0f;
}

/**
 * Gets the limit of the robots translational acceleration.
 * The default implementation of this method returns the result
 * of the <code>getProfileAcceleration()</code> method.
 * If a specific robot model needs to define a maximum acceleration
 * for translational motion, this method must be implemented.
 * @return the limit of the acceleration, given in [m/s&sup2;].
 */
float RobotModel::getTranslationalProfileAcceleration() {
    
    return getProfileAcceleration();
}

/**
 * Gets the limit of the robots rotational acceleration.
 * The default implementation of this method returns the result
 * of the <code>getProfileAcceleration()</code> method.
 * If a specific robot model needs to define a maximum acceleration
 * for rotational motion, this method must be implemented.
 * @return the limit of the acceleration, given in [rad/s&sup2;].
 */
float RobotModel::getRotationalProfileAcceleration() {
    
    return getProfileAcceleration();
}

/**
 * Gets the limit of the robots joint acceleration.
 * The default implementation of this method returns the result
 * of the <code>getProfileAcceleration()</code> method.
 * If a specific robot model needs to define maximum accelerations
 * for each joint, this method must be implemented.
 * @param joint the joint to get the acceleration limit for.
 * @return the limit of the acceleration.
 */
float RobotModel::getProfileAcceleration(uint16_t joint) {
    
    return getProfileAcceleration();
}

/**
 * Gets the limit of the robots deceleration.
 * @return the limit of the deceleration.
 */
float RobotModel::getProfileDeceleration() {
    
    return 0.0f;
}

/**
 * Gets the limit of the robots translational deceleration.
 * The default implementation of this method returns the result
 * of the <code>getProfileDeceleration()</code> method.
 * If a specific robot model needs to define a maximum deceleration
 * for translational motion, this method must be implemented.
 * @return the limit of the deceleration, given in [m/s&sup2;].
 */
float RobotModel::getTranslationalProfileDeceleration() {
    
    return getProfileDeceleration();
}

/**
 * Gets the limit of the robots rotational deceleration.
 * The default implementation of this method returns the result
 * of the <code>getProfileDeceleration()</code> method.
 * If a specific robot model needs to define a maximum deceleration
 * for rotational motion, this method must be implemented.
 * @return the limit of the deceleration, given in [rad/s&sup2;].
 */
float RobotModel::getRotationalProfileDeceleration() {
    
    return getProfileDeceleration();
}

/**
 * Gets the limit of the robots joint deceleration.
 * The default implementation of this method returns the result
 * of the <code>getProfileDeceleration()</code> method.
 * If a specific robot model needs to define maximum decelerations
 * for each joint, this method must be implemented.
 * @param joint the joint to get the deceleration limit for.
 * @return the limit of the deceleration.
 */
float RobotModel::getProfileDeceleration(uint16_t joint) {
    
    return getProfileDeceleration();
}

/**
 * Gets the limit of the robots jerk.
 * @return the limit of the jerk.
 */
float RobotModel::getProfileJerk() {
    
    return 0.0f;
}

/**
 * Gets the limit of the robots translational jerk.
 * The default implementation of this method returns the result
 * of the <code>getProfileJerk()</code> method.
 * If a specific robot model needs to define a maximum jerk
 * for translational motion, this method must be implemented.
 * @return the limit of the jerk, given in [m/s&sup3;].
 */
float RobotModel::getTranslationalProfileJerk() {
    
    return getProfileJerk();
}

/**
 * Gets the limit of the robots rotational jerk.
 * The default implementation of this method returns the result
 * of the <code>getProfileJerk()</code> method.
 * If a specific robot model needs to define a maximum jerk
 * for rotational motion, this method must be implemented.
 * @return the limit of the jerk, given in [rad/s&sup3;].
 */
float RobotModel::getRotationalProfileJerk() {
    
    return getProfileJerk();
}

/**
 * Gets the limit of the robots joint jerk.
 * The default implementation of this method returns the result
 * of the <code>getProfileJerk()</code> method.
 * If a specific robot model needs to define maximum jerks
 * for each joint, this method must be implemented.
 * @param joint the joint to get the jerk limit for.
 * @return the limit of the jerk.
 */
float RobotModel::getProfileJerk(uint16_t joint) {
    
    return getProfileJerk();
}

/**
 * Limits the given actuator coordinates to values that are within the physical range of this robot.
 * @param q an array of given actuator coordinates to limit.
 * @return <code>true</code> if the values were limited, <code>false</code> otherwise.
 */
bool RobotModel::limitActuatorCoordinates(double q[]) {
    
    return false;
}

/**
 * Limits the given actuator coordinates to values that are within the physical range of this robot.
 * @param q an array of Motion objects with actuator coordinates to limit.
 * @return <code>true</code> if the values were limited, <code>false</code> otherwise.
 */
bool RobotModel::limitActuatorCoordinates(Motion q[]) {
    
    double qTemp[numberOfActuators];
    for (uint16_t i = 0; i < numberOfActuators; i++) qTemp[i] = q[i].position;
	bool limit = limitActuatorCoordinates(qTemp);
	if (limit) for (uint16_t i = 0; i < numberOfActuators; i++) q[i].position = qTemp[i];
	
	return limit;
}

/**
 * Limits the given joint angles to values that are within the physical range of this robot.
 * @param phi an array of given joint angles to limit.
 * @return <code>true</code> if the values were limited, <code>false</code> otherwise.
 */
bool RobotModel::limitJointAngles(double phi[]) {
    
    return false;
}

/**
 * Limits the given joint angles to values that are within the physical range of this robot.
 * @param phi an array of Motion objects with joint angles to limit.
 * @return <code>true</code> if the values were limited, <code>false</code> otherwise.
 */
bool RobotModel::limitJointAngles(Motion phi[]) {
    
    double phiTemp[numberOfJoints];
    for (uint16_t i = 0; i < numberOfJoints; i++) phiTemp[i] = phi[i].position;
	bool limit = limitJointAngles(phiTemp);
    if (limit) for (uint16_t i = 0; i < numberOfJoints; i++) phi[i].position = phiTemp[i];
	
	return limit;
}

/**
 * Limits the given Cartesian pose to position and orientation values that are within
 * the physical range of this robot.
 * @param tool a tool object that contains the transformation to the tool center point.
 * @param pose a transformation matrix defining the Cartesian position and orientation of the robots tool.
 * @return <code>true</code> if the pose values were limited, <code>false</code> otherwise.
 */
bool RobotModel::limitCartesianPose(Tool& tool, Transformation& pose) {
    
    return false;
}

/**
 * Calculates the direct kinematic equations from the actuator coordinates to the corresponding joint angles.
 * The default implementation of this method sets the joint angles to the actuator coordinates.
 * @param q an array with given actuator coordinates.
 * @param phi an array with joint angles to calculate.
 */
void RobotModel::directKinematics(const double q[], double phi[]) {

    for (uint16_t i = 0; i < min(numberOfActuators, numberOfJoints); i++) phi[i] = q[i];
}

/**
 * Calculates the direct kinematic equations from the actuator coordinates to the corresponding joint angles.
 * @param q an array of Motion objects with given actuator coordinates.
 * @param phi an array of Motion objects with joint angles to calculate.
 */
void RobotModel::directKinematics(const Motion q[], Motion phi[]) {
    
    const double T = 1.0e-3;
    
    double qOld[numberOfActuators];
    double qNow[numberOfActuators];
    double qNew[numberOfActuators];
    
    for (uint16_t i = 0; i < numberOfActuators; i++) {
        qOld[i] = q[i].position-static_cast<double>(q[i].velocity)*T+static_cast<double>(q[i].acceleration)*0.5*T*T;
        qNow[i] = q[i].position;
        qNew[i] = q[i].position+static_cast<double>(q[i].velocity)*T+static_cast<double>(q[i].acceleration)*0.5*T*T;
    }
    
    double phiOld[numberOfJoints];
    double phiNow[numberOfJoints];
    double phiNew[numberOfJoints];
    
    directKinematics(qOld, phiOld);
    directKinematics(qNow, phiNow);
    directKinematics(qNew, phiNew);
    
    for (uint16_t i = 0; i < numberOfJoints; i++) {
        phi[i].position = phiNow[i];
        phi[i].velocity = static_cast<float>((phiNew[i]-phiOld[i])*0.5/T);
        phi[i].acceleration = static_cast<float>((phiNew[i]-2.0*phiNow[i]+phiOld[i])/T/T);
    }
}

/**
 * Calculates the direct kinematic equations from the joint angles to the Cartesian pose of the robots tool.
 * @param phi an array with given joint angles.
 * @param tool a tool object that contains the transformation to the tool center point.
 * @param pose a transformation matrix defining the Cartesian position and orientation of the robots tool.
 */
void RobotModel::directKinematics(const double phi[], Tool& tool, Transformation& pose) {}

/**
 * Calculates the direct kinematic equations from the joint angles to the Cartesian pose of the robots tool.
 * @param phi an array of Motion objects with given joint angles.
 * @param tool a tool object that contains the transformation to the tool center point.
 * @param pose a transformation matrix defining the Cartesian position and orientation of the robots tool.
 * @param velocity a transformation matrix defining the rate of change of the Cartesian position and orientation of the robots tool.
 * @param acceleration a transformation matrix defining the rate of change of the velocity of the robots tool.
 */
void RobotModel::directKinematics(const Motion phi[], Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration) {
    
    const double T = 1.0e-3;
    
    double phiOld[numberOfJoints];
    double phiNow[numberOfJoints];
    double phiNew[numberOfJoints];
    
    for (uint16_t i = 0; i < numberOfJoints; i++) {
        phiOld[i] = phi[i].position-static_cast<double>(phi[i].velocity)*T+static_cast<double>(phi[i].acceleration)*0.5*T*T;
        phiNow[i] = phi[i].position;
        phiNew[i] = phi[i].position+static_cast<double>(phi[i].velocity)*T+static_cast<double>(phi[i].acceleration)*0.5*T*T;
    }
    
    Transformation poseOld;
    Transformation poseNew;
    
    directKinematics(phiOld, tool, poseOld);
    directKinematics(phiNow, tool, pose);
    directKinematics(phiNew, tool, poseNew);
    
    for (uint16_t i = 0; i < 12; i++) {
        velocity[i] = (poseNew[i]-poseOld[i])*0.5/T;
        acceleration[i] = (poseNew[i]-2.0*pose[i]+poseOld[i])/T/T;
    }
}

/**
 * Calculates the inverse kinematic equations from the Cartesian pose to the joint angles.
 * @param pose a transformation matrix defining the given Cartesian position and orientation of the robots tool.
 * @param tool a tool object that contains the transformation to the tool center point.
 * @param phi an array with joint angles to calculate.
 * @param configuration a parameter defining the desired configuration of the robot.
 */
void RobotModel::inverseKinematics(Tool& tool, Transformation& pose, double phi[], int16_t configuration) {}

/**
 * Calculates the inverse kinematic equations from the Cartesian pose to the joint angles.
 * @param pose a transformation matrix defining the given Cartesian position and orientation of the robots tool.
 * @param tool a tool object that contains the transformation to the tool center point.
 * @param phi an array of Motion objects with joint angles to calculate.
 * @param configuration a parameter defining the desired configuration of the robot.
 */
void RobotModel::inverseKinematics(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], int16_t configuration) {
    
    // calculate previous and next incremental pose
    
    const double T = 1.0e-3;
    
    Transformation poseOld;
    Transformation poseNew;
    
    for (uint16_t i = 0; i < 12; i++) {
        poseOld[i] = pose[i]-velocity[i]*T+acceleration[i]*0.5*T*T;
        poseNew[i] = pose[i]+velocity[i]*T+acceleration[i]*0.5*T*T;
    }
    
    // calculate previous and next incremental joint angles
    
    double phiOld[numberOfJoints];
    double phiNow[numberOfJoints];
    double phiNew[numberOfJoints];
    
    for (uint16_t i = 0; i < numberOfJoints; i++) {
        phiOld[i] = phi[i].position;
        phiNow[i] = phi[i].position;
        phiNew[i] = phi[i].position;
    }
    
    inverseKinematics(tool, poseOld, phiOld, configuration);
    inverseKinematics(tool, pose, phiNow, configuration);
    inverseKinematics(tool, poseNew, phiNew, configuration);
    
    // calculate motion state of joint angles numerically
    
    for (uint16_t i = 0; i < numberOfJoints; i++) {
        phi[i].position = phiNow[i];
        phi[i].velocity = static_cast<float>((phiNew[i]-phiOld[i])*0.5/T);
        phi[i].acceleration = static_cast<float>((phiNew[i]-2.0*phiNow[i]+phiOld[i])/T/T);
    }
}

/**
 * Calculates the inverse kinematic equations from the joint angles to the corresponding actuator coordinates.
 * @param phi an array with given joint angles.
 * @param q an array with actuator coordinates to calculate.
 */
void RobotModel::inverseKinematics(const double phi[], double q[]) {
    
    for (uint16_t i = 0; i < min(numberOfActuators, numberOfJoints); i++) q[i] = phi[i];
}

/**
 * Calculates the inverse kinematic equations from the joint angles to the corresponding actuator coordinates.
 * @param phi an array of Motion objects with given joint angles.
 * @param q an array of Motion objects with actuator coordinates to calculate.
 */
void RobotModel::inverseKinematics(const Motion phi[], Motion q[]) {
    
    const double T = 1.0e-3;
    
    double phiOld[numberOfJoints];
    double phiNow[numberOfJoints];
    double phiNew[numberOfJoints];
    
    for (uint16_t i = 0; i < numberOfJoints; i++) {
        phiOld[i] = phi[i].position-static_cast<double>(phi[i].velocity)*T+static_cast<double>(phi[i].acceleration)*0.5*T*T;
        phiNow[i] = phi[i].position;
        phiNew[i] = phi[i].position+static_cast<double>(phi[i].velocity)*T+static_cast<double>(phi[i].acceleration)*0.5*T*T;
    }
    
    double qOld[numberOfActuators];
    double qNow[numberOfActuators];
    double qNew[numberOfActuators];
    
    inverseKinematics(phiOld, qOld);
    inverseKinematics(phiNow, qNow);
    inverseKinematics(phiNew, qNew);
    
    for (uint16_t i = 0; i < numberOfActuators; i++) {
        q[i].position = qNow[i];
        q[i].velocity = static_cast<float>((qNew[i]-qOld[i])*0.5/T);
        q[i].acceleration = static_cast<float>((qNew[i]-2.0*qNow[i]+qOld[i])/T/T);
    }
}
