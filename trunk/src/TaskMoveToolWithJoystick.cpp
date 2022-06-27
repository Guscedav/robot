/*
 * TaskMoveToolWithJoystick.cpp
 * Copyright (c) 2018, ZHAW
 * All rights reserved.
 *
 *  Created on: 04.04.2018
 *      Author: Marcel Honegger
 */

#include <cmath>
#include "RobotModel.h"
#include "Tool.h"
#include "AnalogIn.h"
#include "Matrix.h"
#include "Transformation.h"
#include "TaskMoveToolWithJoystick.h"

using namespace std;

/**
 * Creates a task object that moves the tool of the robot with velocities given by an analog joystick.
 * @param robotModel a reference to the robot model object to use.
 * @param tool the given tool to use for the calculation of the direct kinematic equations.
 * @param configuration a parameter defining the desired configuration of the robot.
 * @param velocityX a reference to an analog input that defines the velocity in the tools x-direction, given in [m/s].
 * @param velocityY a reference to an analog input that defines the velocity in the tools y-direction, given in [m/s].
 * @param velocityZ a reference to an analog input that defines the velocity in the tools z-direction, given in [m/s].
 * @param velocityA a reference to an analog input that defines the rotation about the tools x-axis, given in [rad/s].
 * @param velocityB a reference to an analog input that defines the rotation about the tools y-axis, given in [rad/s].
 * @param velocityC a reference to an analog input that defines the rotation about the tools z-axis, given in [rad/s].
 */
TaskMoveToolWithJoystick::TaskMoveToolWithJoystick(RobotModel& robotModel, Tool& tool, int16_t configuration, AnalogIn& velocityX, AnalogIn& velocityY, AnalogIn& velocityZ, AnalogIn& velocityA, AnalogIn& velocityB, AnalogIn& velocityC) : Task(robotModel, tool), velocityX(velocityX), velocityY(velocityY), velocityZ(velocityZ), velocityA(velocityA), velocityB(velocityB), velocityC(velocityC) {

    this->configuration = configuration;
}

/**
 * Deletes the task object.
 */
TaskMoveToolWithJoystick::~TaskMoveToolWithJoystick() {}

/**
 * Initializes this task object.
 */
void TaskMoveToolWithJoystick::init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {

    // get current transformation to tcp

    Transformation tcp;
    tool.getTCPPose(tcp);
    tcp.invert();

    // set new tool defined with this task

    tool.set(this->tool);

    // get new transformation to tcp

    Transformation tcpNew;
    tool.getTCPPose(tcpNew);

    tcp.mul(tcpNew);

    // adapt pose, velocity and acceleration for new tool

    pose.mul(tcp);
    velocity.mul(tcp);
    acceleration.mul(tcp);
}

/**
 * Increments the robots motion states in all coordinate spaces.
 */
int16_t TaskMoveToolWithJoystick::increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period) {
    
    // read joystick values and evaluate the dominant motion
    
    float velocityX = this->velocityX;
    float velocityY = this->velocityY;
    float velocityZ = this->velocityZ;
    float velocityA = this->velocityA;
    float velocityB = this->velocityB;
    float velocityC = this->velocityC;
    
    float maxVelocity = fmax(fabs(velocityX), fabs(velocityY));
    maxVelocity = fmax(maxVelocity, fabs(velocityZ));
    maxVelocity = fmax(maxVelocity, fabs(velocityA));
    maxVelocity = fmax(maxVelocity, fabs(velocityB));
    maxVelocity = fmax(maxVelocity, fabs(velocityC));
    
    if (fabs(velocityX) < maxVelocity) velocityX = 0.0f;
    if (fabs(velocityY) < maxVelocity) velocityY = 0.0f;
    if (fabs(velocityZ) < maxVelocity) velocityZ = 0.0f;
    if (fabs(velocityA) < maxVelocity) velocityA = 0.0f;
    if (fabs(velocityB) < maxVelocity) velocityB = 0.0f;
    if (fabs(velocityC) < maxVelocity) velocityC = 0.0f;
    
    // increment the linear motion states in the tool coordinate system
    
    x.position = 0.0;
    x.velocity = pose[0]*velocity[3]+pose[4]*velocity[7]+pose[8]*velocity[11];
    x.acceleration = pose[0]*acceleration[3]+pose[4]*acceleration[7]+pose[8]*acceleration[11];
    x.setLimits(robotModel.getTranslationalProfileVelocity(), robotModel.getTranslationalProfileAcceleration(), robotModel.getTranslationalProfileDeceleration(), robotModel.getTranslationalProfileJerk());
    x.incrementToVelocity(velocityX, period);
    
    y.position = 0.0;
    y.velocity = pose[1]*velocity[3]+pose[5]*velocity[7]+pose[9]*velocity[11];
    y.acceleration = pose[1]*acceleration[3]+pose[5]*acceleration[7]+pose[9]*acceleration[11];
    y.setLimits(robotModel.getTranslationalProfileVelocity(), robotModel.getTranslationalProfileAcceleration(), robotModel.getTranslationalProfileDeceleration(), robotModel.getTranslationalProfileJerk());
    y.incrementToVelocity(velocityY, period);
    
    z.position = 0.0;
    z.velocity = pose[2]*velocity[3]+pose[6]*velocity[7]+pose[10]*velocity[11];
    z.acceleration = pose[2]*acceleration[3]+pose[6]*acceleration[7]+pose[10]*acceleration[11];
    z.setLimits(robotModel.getTranslationalProfileVelocity(), robotModel.getTranslationalProfileAcceleration(), robotModel.getTranslationalProfileDeceleration(), robotModel.getTranslationalProfileJerk());
    z.incrementToVelocity(velocityZ, period);
    
    pose[3] = pose[3]+pose[0]*x.position+pose[1]*y.position+pose[2]*z.position;
    velocity[3] = pose[0]*x.velocity+pose[1]*y.velocity+pose[2]*z.velocity;
    acceleration[3] = pose[0]*x.acceleration+pose[1]*y.acceleration+pose[2]*z.acceleration;
    
    pose[7] = pose[7]+pose[4]*x.position+pose[5]*y.position+pose[6]*z.position;
    velocity[7] = pose[4]*x.velocity+pose[5]*y.velocity+pose[6]*z.velocity;
    acceleration[7] = pose[4]*x.acceleration+pose[5]*y.acceleration+pose[6]*z.acceleration;
    
    pose[11] = pose[11]+pose[8]*x.position+pose[9]*y.position+pose[10]*z.position;
    velocity[11] = pose[8]*x.velocity+pose[9]*y.velocity+pose[10]*z.velocity;
    acceleration[11] = pose[8]*x.acceleration+pose[9]*y.acceleration+pose[10]*z.acceleration;
    
    // calculate the rotational motion states in the tool coordinate system
    
    const double T = 1.0e-3;
    
    Matrix matrix;
    
    Matrix toolPose;
    
    Matrix toolVelocity;
    pose.getMatrix(toolVelocity);
    velocity.getMatrix(matrix);
    toolVelocity.transpose();
    toolVelocity.mul(matrix);
    
    Matrix toolAcceleration;
    pose.getMatrix(toolAcceleration);
    acceleration.getMatrix(matrix);
    toolAcceleration.transpose();
    toolAcceleration.mul(matrix);
    
    Matrix matrixOld;
    Matrix matrixNow;
    Matrix matrixNew;
    
    for (uint16_t i = 0; i < 9; i++) {
        matrixOld[i] = toolPose[i]-toolVelocity[i]*T+toolAcceleration[i]*0.5*T*T;
    }
    
    for (uint16_t i = 0; i < 9; i++) {
        matrixNew[i] = toolPose[i]+toolVelocity[i]*T+toolAcceleration[i]*0.5*T*T;
    }
    
    double alphaOld = 0.0;
    double betaOld = 0.0;
    double gammaOld = 0.0;
    
    matrixOld.getZYXEulerAngles(alphaOld, betaOld, gammaOld);
    
    double alphaNew = 0.0;
    double betaNew = 0.0;
    double gammaNew = 0.0;
    
    matrixNew.getZYXEulerAngles(alphaNew, betaNew, gammaNew);
    
    // increment the rotational motion states in the tool coordinate system
    
    a.position = 0.0;
    a.velocity = (alphaNew-alphaOld)*0.5/T;
    a.acceleration = (alphaNew+alphaOld)/T/T;
    a.setLimits(robotModel.getRotationalProfileVelocity(), robotModel.getRotationalProfileAcceleration(), robotModel.getRotationalProfileDeceleration(), robotModel.getRotationalProfileJerk());
    a.incrementToVelocity(velocityA, period);
    
    b.position = 0.0;
    b.velocity = (betaNew-betaOld)*0.5/T;
    b.acceleration = (betaNew+betaOld)/T/T;
    b.setLimits(robotModel.getRotationalProfileVelocity(), robotModel.getRotationalProfileAcceleration(), robotModel.getRotationalProfileDeceleration(), robotModel.getRotationalProfileJerk());
    b.incrementToVelocity(velocityB, period);
    
    c.position = 0.0;
    c.velocity = (gammaNew-gammaOld)*0.5/T;
    c.acceleration = (gammaNew+gammaOld)/T/T;
    c.setLimits(robotModel.getRotationalProfileVelocity(), robotModel.getRotationalProfileAcceleration(), robotModel.getRotationalProfileDeceleration(), robotModel.getRotationalProfileJerk());
    c.incrementToVelocity(velocityC, period);
    
    matrixOld.set(a.position-a.velocity*T+a.acceleration*0.5*T*T, b.position-b.velocity*T+b.acceleration*0.5*T*T, c.position-c.velocity*T+c.acceleration*0.5*T*T);
    matrixNow.set(a.position, b.position, c.position);
    matrixNew.set(a.position+a.velocity*T+a.acceleration*0.5*T*T, b.position+b.velocity*T+b.acceleration*0.5*T*T, c.position+c.velocity*T+c.acceleration*0.5*T*T);
    
    // calculate the rotational motion states in the tool coordinate system
    
    pose.getMatrix(matrix);
    matrix.mul(matrixOld);
    matrixOld.set(matrix);
    
    pose.getMatrix(matrix);
    matrix.mul(matrixNow);
    matrixNow.set(matrix);
    
    pose.getMatrix(matrix);
    matrix.mul(matrixNew);
    matrixNew.set(matrix);
    
    pose.set(matrixNow);
    
    velocity[0] = (matrixNew[0]-matrixOld[0])*0.5/T;
    velocity[1] = (matrixNew[1]-matrixOld[1])*0.5/T;
    velocity[2] = (matrixNew[2]-matrixOld[2])*0.5/T;
    velocity[4] = (matrixNew[3]-matrixOld[3])*0.5/T;
    velocity[5] = (matrixNew[4]-matrixOld[4])*0.5/T;
    velocity[6] = (matrixNew[5]-matrixOld[5])*0.5/T;
    velocity[8] = (matrixNew[6]-matrixOld[6])*0.5/T;
    velocity[9] = (matrixNew[7]-matrixOld[7])*0.5/T;
    velocity[10] = (matrixNew[8]-matrixOld[8])*0.5/T;
    
    acceleration[0] = (matrixNew[0]-2.0*matrixNow[0]+matrixOld[0])/T/T;
    acceleration[1] = (matrixNew[1]-2.0*matrixNow[1]+matrixOld[1])/T/T;
    acceleration[2] = (matrixNew[2]-2.0*matrixNow[2]+matrixOld[2])/T/T;
    acceleration[4] = (matrixNew[3]-2.0*matrixNow[3]+matrixOld[3])/T/T;
    acceleration[5] = (matrixNew[4]-2.0*matrixNow[4]+matrixOld[4])/T/T;
    acceleration[6] = (matrixNew[5]-2.0*matrixNow[5]+matrixOld[5])/T/T;
    acceleration[8] = (matrixNew[6]-2.0*matrixNow[6]+matrixOld[6])/T/T;
    acceleration[9] = (matrixNew[7]-2.0*matrixNow[7]+matrixOld[7])/T/T;
    acceleration[10] = (matrixNew[8]-2.0*matrixNow[8]+matrixOld[8])/T/T;
    
    // calculate the motion states in other coordinate spaces

    robotModel.limitCartesianPose(tool, pose);
    robotModel.inverseKinematics(tool, pose, velocity, acceleration, phi, configuration);
    if (robotModel.limitJointAngles(phi)) {
        robotModel.inverseKinematics(phi, q);
        if (robotModel.limitActuatorCoordinates(q)) {
            robotModel.directKinematics(q, phi);
        } else {
            // leave phi unchanged
        }
        robotModel.directKinematics(phi, tool, pose, velocity, acceleration);
    } else {
        robotModel.inverseKinematics(phi, q);
        if (robotModel.limitActuatorCoordinates(q)) {
            robotModel.directKinematics(q, phi);
            robotModel.directKinematics(phi, tool, pose, velocity, acceleration);
        } else {
            // states in all coordinate spaces calculated
        }
    }

    return DONE;
}

/**
 * Exits this task object.
 */
void TaskMoveToolWithJoystick::exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {

    // set the target motion states to the current motion states

    poseTarget.set(pose);
    velocityTarget.set(velocity);
    accelerationTarget.set(acceleration);

    for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) phiTarget[i].set(phi[i]);
    for (uint16_t i = 0; i < robotModel.getNumberOfActuators(); i++) qTarget[i].set(q[i]);
}
