/*
 * TaskMoveLinear.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 27.10.2016
 *      Author: Marcel Honegger
 */

#include <cmath>
#include "RobotModel.h"
#include "Tool.h"
#include "TaskMoveLinear.h"

using namespace std;

const double TaskMoveLinear::MINIMUM_ZONE = 1.0e-6;

/**
 * Creates a task object that moves the robot to a given Cartesian pose.
 * @param robotModel a reference to the robot model object to use.
 * @param tool the given tool to use for the calculation of the direct kinematic equations.
 * @param pose a transformation matrix defining the desired Cartesian position and orientation of the robots tool.
 * @param configuration a parameter defining the desired configuration of the robot.
 * @param velocity the velocity at which to move the robot, given in [m/s].
 */
TaskMoveLinear::TaskMoveLinear(RobotModel& robotModel, Tool& tool, Transformation& pose, int16_t configuration, float velocity) : Task(robotModel, tool) {
    
    robotModel.limitCartesianPose(tool, pose);
    this->pose.set(pose);
    this->pose.normalize();
    this->configuration = configuration;
    this->velocity = velocity;
    this->zone = MINIMUM_ZONE;
}

/**
 * Creates a task object that moves the robot to a given Cartesian pose.
 * @param robotModel a reference to the robot model object to use.
 * @param tool the given tool to use for the calculation of the direct kinematic equations.
 * @param pose a transformation matrix defining the desired Cartesian position and orientation of the robots tool.
 * @param configuration a parameter defining the desired configuration of the robot.
 * @param velocity the velocity at which to move the robot, given in [m/s].
 * @param zone the minimum distance to the desired pose before the robot moves to the next task in the task list, given in [m].
 * A zone parameter of 0.0 means that the robot moves precisely to the given pose and stops, before moving to the next task in the task list.
 */
TaskMoveLinear::TaskMoveLinear(RobotModel& robotModel, Tool& tool, Transformation& pose, int16_t configuration, float velocity, double zone) : Task(robotModel, tool) {
    
    robotModel.limitCartesianPose(tool, pose);
    this->pose.set(pose);
    this->pose.normalize();
    this->configuration = configuration;
    this->velocity = velocity;
    this->zone = (zone < MINIMUM_ZONE) ? MINIMUM_ZONE : zone;
}

/**
 * Deletes the task object.
 */
TaskMoveLinear::~TaskMoveLinear() {}

/**
 * Initializes this task object.
 */
void TaskMoveLinear::init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {

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
    
    // define pose, velocity and acceleration in local base system
    
    Matrix matrix;
    
    pose.getMatrix(localBase);
    
    Matrix poseBase;
    
    Matrix velocityBase(localBase);
    velocityBase.invert();
    velocity.getMatrix(matrix);
    velocityBase.mul(matrix);
    
    Matrix accelerationBase(localBase);
    accelerationBase.invert();
    acceleration.getMatrix(matrix);
    accelerationBase.mul(matrix);
    
    // initialize orientation with quaternion
    
    const double T = 1.0e-3;
    
    Quaternion qOld;
    Quaternion qNow;
    Quaternion qNew;
    
    for (uint16_t i = 0; i < 9; i++) {
        matrix[i] = poseBase[i]-velocityBase[i]*T+accelerationBase[i]*0.5*T*T;
    }
    matrix.getQuaternion(qOld);
    
    poseBase.getQuaternion(qNow);
    
    for (uint16_t i = 0; i < 9; i++) {
        matrix[i] = poseBase[i]+velocityBase[i]*T+accelerationBase[i]*0.5*T*T;
    }
    matrix.getQuaternion(qNew);
    
    q0.position = qNow[0];
    q0.velocity = (qNew[0]-qOld[0])*0.5/T;
    q0.acceleration = (qNew[0]-2.0*qNow[0]+qOld[0])/T/T;
    
    q1.position = qNow[1];
    q1.velocity = (qNew[1]-qOld[1])*0.5/T;
    q1.acceleration = (qNew[1]-2.0*qNow[1]+qOld[1])/T/T;
    
    q2.position = qNow[2];
    q2.velocity = (qNew[2]-qOld[2])*0.5/T;
    q2.acceleration = (qNew[2]-2.0*qNow[2]+qOld[2])/T/T;
    
    q3.position = qNow[3];
    q3.velocity = (qNew[3]-qOld[3])*0.5/T;
    q3.acceleration = (qNew[3]-2.0*qNow[3]+qOld[3])/T/T;
    
    // define target orientation in local base system
    
    this->pose.getMatrix(matrix);
    
    Matrix inverseBase(localBase);
    inverseBase.invert();
    inverseBase.mul(matrix);
    inverseBase.getQuaternion(quaternion);
    
    // calculate the rotation matrix for a synchronized linear motion
    
    float a = atan2(this->pose[7]-poseTarget[7], this->pose[3]-poseTarget[3]);
    float sa = sin(a), ca = cos(a);
    
    float b = 0.0f;
    if (fabs(sa) < fabs(ca)) {
        b = atan2(this->pose[11]-poseTarget[11], (this->pose[3]-poseTarget[3])/ca);
    } else {
        b = atan2(this->pose[11]-poseTarget[11], (this->pose[7]-poseTarget[7])/sa);
    }
    float sb = sin(b), cb = cos(b);
    
    rLinear[0][0] = ca*cb;  rLinear[0][1] = -sa;   rLinear[0][2] = -ca*sb;
    rLinear[1][0] = sa*cb;  rLinear[1][1] = ca;    rLinear[1][2] = -sa*sb;
    rLinear[2][0] = sb;     rLinear[2][1] = 0.0f;  rLinear[2][2] = cb;
}

/**
 * Increments the robots motion states in all coordinate spaces.
 */
int16_t TaskMoveLinear::increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period) {
    
    // calculate the desired translational profile velocity
    
    float profileVelocity = this->velocity*velocityOverride;
    if (profileVelocity > robotModel.getTranslationalProfileVelocity()) profileVelocity = robotModel.getTranslationalProfileVelocity();
    
    // transform and increment the Cartesian motion
    
    x.position = rLinear[0][0]*(pose[3]-this->pose[3])+rLinear[1][0]*(pose[7]-this->pose[7])+rLinear[2][0]*(pose[11]-this->pose[11]);
    x.velocity = rLinear[0][0]*velocity[3]+rLinear[1][0]*velocity[7]+rLinear[2][0]*velocity[11];
    x.acceleration = rLinear[0][0]*acceleration[3]+rLinear[1][0]*acceleration[7]+rLinear[2][0]*acceleration[11];
    
    y.position = rLinear[0][1]*(pose[3]-this->pose[3])+rLinear[1][1]*(pose[7]-this->pose[7])+rLinear[2][1]*(pose[11]-this->pose[11]);
    y.velocity = rLinear[0][1]*velocity[3]+rLinear[1][1]*velocity[7]+rLinear[2][1]*velocity[11];
    y.acceleration = rLinear[0][1]*acceleration[3]+rLinear[1][1]*acceleration[7]+rLinear[2][1]*acceleration[11];
    
    z.position = rLinear[0][2]*(pose[3]-this->pose[3])+rLinear[1][2]*(pose[7]-this->pose[7])+rLinear[2][2]*(pose[11]-this->pose[11]);
    z.velocity = rLinear[0][2]*velocity[3]+rLinear[1][2]*velocity[7]+rLinear[2][2]*velocity[11];
    z.acceleration = rLinear[0][2]*acceleration[3]+rLinear[1][2]*acceleration[7]+rLinear[2][2]*acceleration[11];
    
    x.setLimits(profileVelocity, robotModel.getTranslationalProfileAcceleration(), robotModel.getTranslationalProfileDeceleration(), robotModel.getTranslationalProfileJerk());
    x.incrementToPosition(0.0, period);
    
    y.setLimits(profileVelocity, robotModel.getTranslationalProfileAcceleration(), robotModel.getTranslationalProfileDeceleration(), robotModel.getTranslationalProfileJerk());
    y.incrementToPosition(0.0f, period);
    
    z.setLimits(profileVelocity, robotModel.getTranslationalProfileAcceleration(), robotModel.getTranslationalProfileDeceleration(), robotModel.getTranslationalProfileJerk());
    z.incrementToPosition(0.0f, period);
    
    pose[3] = this->pose[3]+rLinear[0][0]*x.position+rLinear[0][1]*y.position+rLinear[0][2]*z.position;
    velocity[3] = rLinear[0][0]*x.velocity+rLinear[0][1]*y.velocity+rLinear[0][2]*z.velocity;
    acceleration[3] = rLinear[0][0]*x.acceleration+rLinear[0][1]*y.acceleration+rLinear[0][2]*z.acceleration;
    
    pose[7] = this->pose[7]+rLinear[1][0]*x.position+rLinear[1][1]*y.position+rLinear[1][2]*z.position;
    velocity[7] = rLinear[1][0]*x.velocity+rLinear[1][1]*y.velocity+rLinear[1][2]*z.velocity;
    acceleration[7] = rLinear[1][0]*x.acceleration+rLinear[1][1]*y.acceleration+rLinear[1][2]*z.acceleration;
    
    pose[11] = this->pose[11]+rLinear[2][0]*x.position+rLinear[2][1]*y.position+rLinear[2][2]*z.position;
    velocity[11] = rLinear[2][0]*x.velocity+rLinear[2][1]*y.velocity+rLinear[2][2]*z.velocity;
    acceleration[11] = rLinear[2][0]*x.acceleration+rLinear[2][1]*y.acceleration+rLinear[2][2]*z.acceleration;
    
    // calculate the rotation matrix for a synchronized rotational motion
    
    double distance = 0.0;
    distance += (q0.position-quaternion[0])*(q0.position-quaternion[0]);
    distance += (q1.position-quaternion[1])*(q1.position-quaternion[1]);
    distance += (q2.position-quaternion[2])*(q2.position-quaternion[2]);
    distance += (q3.position-quaternion[3])*(q3.position-quaternion[3]);
    distance = sqrt(distance);
    
    float a = atan2(quaternion[1]-q1.position, quaternion[0]-q0.position);
    float sa = sin(a), ca = cos(a);
    
    float b = 0.0f;
    if (fabs(sin(a)) < sqrt(2.0f)/2.0f) {
        b = atan2(quaternion[2]-q2.position, (quaternion[0]-q0.position)/cos(a));
    } else {
        b = atan2(quaternion[2]-q2.position, (quaternion[1]-q1.position)/sin(a));
    }
    float sb = sin(b), cb = cos(b);
    
    float c = 0.0f;
    if (fabs(sin(b)) > 1.0e-9f) {
        c = atan2(quaternion[3]-q3.position, (quaternion[2]-q2.position)/sin(b));
    } else if (distance > 1.0e-9f) {
        float argument = (quaternion[3]-q3.position)/distance;
        c = (argument < -1.0f) ? -M_PI/2.0f : (argument > 1.0f) ? M_PI/2.0f : asin(argument);
    }
    float sc = sin(c), cc = cos(c);

    rRotational[0][0] = ca*cb*cc;  rRotational[0][1] = -sa;   rRotational[0][2] = -ca*sb;  rRotational[0][3] = -ca*cb*sc;
    rRotational[1][0] = sa*cb*cc;  rRotational[1][1] = ca;    rRotational[1][2] = -sa*sb;  rRotational[1][3] = -sa*cb*sc;
    rRotational[2][0] = sb*cc;     rRotational[2][1] = 0.0f;  rRotational[2][2] = cb;      rRotational[2][3] = -sb*sc;
    rRotational[3][0] = sc;        rRotational[3][1] = 0.0f;  rRotational[3][2] = 0.0f;    rRotational[3][3] = cc;
    
    // rotate the motion states
    
    Motion qLocal[4];
    for (uint16_t i = 0; i < 4; i++) {
        qLocal[i].position = rRotational[0][i]*(q0.position-quaternion[0])+rRotational[1][i]*(q1.position-quaternion[1])+rRotational[2][i]*(q2.position-quaternion[2])+rRotational[3][i]*(q3.position-quaternion[3]);
        qLocal[i].velocity = rRotational[0][i]*q0.velocity+rRotational[1][i]*q1.velocity+rRotational[2][i]*q2.velocity+rRotational[3][i]*q3.velocity;
        qLocal[i].acceleration = rRotational[0][i]*q0.acceleration+rRotational[1][i]*q1.acceleration+rRotational[2][i]*q2.acceleration+rRotational[3][i]*q3.acceleration;
    }
    
    // calculate the desired rotational profile velocity
    
    profileVelocity = this->velocity*velocityOverride;
    if (profileVelocity > robotModel.getRotationalProfileVelocity()) profileVelocity = robotModel.getRotationalProfileVelocity();
    
    // increment the motion states of the orientation
    
    for (uint16_t i = 0; i < 4; i++) {
        qLocal[i].setLimits(profileVelocity, robotModel.getRotationalProfileAcceleration(), robotModel.getRotationalProfileDeceleration(), robotModel.getRotationalProfileJerk());
    }
    for (uint16_t i = 0; i < 1; i++) {
        qLocal[i].incrementToPosition(0.0, period);
    }
    for (uint16_t i = 1; i < 4; i++) {
        qLocal[i].incrementToVelocity(0.0f, period);
    }
    
    // rotate the motion states back
    
    q0.position = quaternion[0]+rRotational[0][0]*qLocal[0].position+rRotational[0][1]*qLocal[1].position+rRotational[0][2]*qLocal[2].position+rRotational[0][3]*qLocal[3].position;
    q0.velocity = rRotational[0][0]*qLocal[0].velocity+rRotational[0][1]*qLocal[1].velocity+rRotational[0][2]*qLocal[2].velocity+rRotational[0][3]*qLocal[3].velocity;
    q0.acceleration = rRotational[0][0]*qLocal[0].acceleration+rRotational[0][1]*qLocal[1].acceleration+rRotational[0][2]*qLocal[2].acceleration+rRotational[0][3]*qLocal[3].acceleration;
    
    q1.position = quaternion[1]+rRotational[1][0]*qLocal[0].position+rRotational[1][1]*qLocal[1].position+rRotational[1][2]*qLocal[2].position+rRotational[1][3]*qLocal[3].position;
    q1.velocity = rRotational[1][0]*qLocal[0].velocity+rRotational[1][1]*qLocal[1].velocity+rRotational[1][2]*qLocal[2].velocity+rRotational[1][3]*qLocal[3].velocity;
    q1.acceleration = rRotational[1][0]*qLocal[0].acceleration+rRotational[1][1]*qLocal[1].acceleration+rRotational[1][2]*qLocal[2].acceleration+rRotational[1][3]*qLocal[3].acceleration;
    
    q2.position = quaternion[2]+rRotational[2][0]*qLocal[0].position+rRotational[2][1]*qLocal[1].position+rRotational[2][2]*qLocal[2].position+rRotational[2][3]*qLocal[3].position;
    q2.velocity = rRotational[2][0]*qLocal[0].velocity+rRotational[2][1]*qLocal[1].velocity+rRotational[2][2]*qLocal[2].velocity+rRotational[2][3]*qLocal[3].velocity;
    q2.acceleration = rRotational[2][0]*qLocal[0].acceleration+rRotational[2][1]*qLocal[1].acceleration+rRotational[2][2]*qLocal[2].acceleration+rRotational[2][3]*qLocal[3].acceleration;
    
    q3.position = quaternion[3]+rRotational[3][0]*qLocal[0].position+rRotational[3][1]*qLocal[1].position+rRotational[3][2]*qLocal[2].position+rRotational[3][3]*qLocal[3].position;
    q3.velocity = rRotational[3][0]*qLocal[0].velocity+rRotational[3][1]*qLocal[1].velocity+rRotational[3][2]*qLocal[2].velocity+rRotational[3][3]*qLocal[3].velocity;
    q3.acceleration = rRotational[3][0]*qLocal[0].acceleration+rRotational[3][1]*qLocal[1].acceleration+rRotational[3][2]*qLocal[2].acceleration+rRotational[3][3]*qLocal[3].acceleration;
    
    // calculate the new pose, velocity and acceleration transforms
    
    const double T = 1.0e-3;
    
    Quaternion quaternionOld(q0.position-q0.velocity*T+q0.acceleration*0.5*T*T, q1.position-q1.velocity*T+q1.acceleration*0.5*T*T, q2.position-q2.velocity*T+q2.acceleration*0.5*T*T, q3.position-q3.velocity*T+q3.acceleration*0.5*T*T);
    Quaternion quaternionNow(q0.position, q1.position, q2.position, q3.position);
    Quaternion quaternionNew(q0.position+q0.velocity*T+q0.acceleration*0.5*T*T, q1.position+q1.velocity*T+q1.acceleration*0.5*T*T, q2.position+q2.velocity*T+q2.acceleration*0.5*T*T, q3.position+q3.velocity*T+q3.acceleration*0.5*T*T);
    
    quaternionOld.normalize();
    quaternionNow.normalize();
    quaternionNew.normalize();
    
    Matrix matrixOld(quaternionOld);
    Matrix matrixNow(quaternionNow);
    Matrix matrixNew(quaternionNew);
    
    Matrix matrix(localBase);
    matrix.mul(matrixOld);
    matrixOld.set(matrix);
    
    matrix.set(localBase);
    matrix.mul(matrixNow);
    matrixNow.set(matrix);
    
    matrix.set(localBase);
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
    
    // check if the desired Cartesian pose is reached
    
    distance = sqrt((pose[3]-this->pose[3])*(pose[3]-this->pose[3])+(pose[7]-this->pose[7])*(pose[7]-this->pose[7])+(pose[11]-this->pose[11])*(pose[11]-this->pose[11]));
    
    double scalarX = this->pose[0]*pose[0]+this->pose[4]*pose[4]+this->pose[8]*pose[8];
    if (scalarX > 1.0) scalarX = 1.0; else if (scalarX < -1.0) scalarX = -1.0;
    double angleX = acos(scalarX);
    
    double scalarY = this->pose[1]*pose[1]+this->pose[5]*pose[5]+this->pose[9]*pose[9];
    if (scalarY > 1.0) scalarY = 1.0; else if (scalarY < -1.0) scalarY = -1.0;
    double angleY = acos(scalarY);
    
    if ((distance < zone) && (angleX < zone) && (angleY < zone)) {
    
        return DONE;
        
    } else {
        
        return RUNNING;
    }
}

/**
 * Exits this task object.
 */
void TaskMoveLinear::exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]) {

    // set the target motion states in Cartesian coordinates

    poseTarget.set(this->pose);

    for (uint16_t i = 0; i < 16; i++) {
        velocityTarget[i] = 0.0;
        accelerationTarget[i] = 0.0;
    }

    // calculate the target motion states in other coordinate spaces

    robotModel.inverseKinematics(tool, poseTarget, velocityTarget, accelerationTarget, phiTarget, configuration);
    if (robotModel.limitJointAngles(phiTarget)) {
        robotModel.inverseKinematics(phiTarget, qTarget);
        if (robotModel.limitActuatorCoordinates(qTarget)) {
            robotModel.directKinematics(qTarget, phiTarget);
        } else {
            // leave phi unchanged
        }
        robotModel.directKinematics(phiTarget, tool, poseTarget, velocityTarget, accelerationTarget);
    } else {
        robotModel.inverseKinematics(phiTarget, qTarget);
        if (robotModel.limitActuatorCoordinates(qTarget)) {
            robotModel.directKinematics(qTarget, phiTarget);
            robotModel.directKinematics(phiTarget, tool, poseTarget, velocityTarget, accelerationTarget);
        } else {
            // states in all coordinate spaces calculated
        }
    }
}
