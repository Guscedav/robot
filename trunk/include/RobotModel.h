/*
 * RobotModel.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 21.01.2016
 *      Author: Marcel Honegger
 */

#ifndef ROBOT_MODEL_H_
#define ROBOT_MODEL_H_

#include <cstdlib>
#include <stdint.h>

class Tool;
class Transformation;
class Motion;

/**
 * This abstract class implements the interface of a robot model.
 * The robot model describes the kinematic properties of a robotic manipulator.
 */
class RobotModel {

    public:
        
                        RobotModel(uint16_t numberOfJoints);
                        RobotModel(uint16_t numberOfActuators, uint16_t numberOfJoints);
        virtual         ~RobotModel();
        uint16_t        getNumberOfActuators();
        uint16_t        getNumberOfJoints();
        virtual float   getProfileVelocity();
        virtual float   getTranslationalProfileVelocity();
        virtual float   getRotationalProfileVelocity();
        virtual float   getProfileVelocity(uint16_t joint);
        virtual float   getProfileAcceleration();
        virtual float   getTranslationalProfileAcceleration();
        virtual float   getRotationalProfileAcceleration();
        virtual float   getProfileAcceleration(uint16_t joint);
        virtual float   getProfileDeceleration();
        virtual float   getTranslationalProfileDeceleration();
        virtual float   getRotationalProfileDeceleration();
        virtual float   getProfileDeceleration(uint16_t joint);
        virtual float   getProfileJerk();
        virtual float   getTranslationalProfileJerk();
        virtual float   getRotationalProfileJerk();
        virtual float   getProfileJerk(uint16_t joint);
        virtual bool    limitActuatorCoordinates(double q[]);
        virtual bool    limitActuatorCoordinates(Motion q[]);
        virtual bool    limitJointAngles(double phi[]);
        virtual bool    limitJointAngles(Motion phi[]);
        virtual bool    limitCartesianPose(Tool& tool, Transformation& pose);
        virtual void    directKinematics(const double q[], double phi[]);
        virtual void    directKinematics(const Motion q[], Motion phi[]);
        virtual void    directKinematics(const double phi[], Tool& tool, Transformation& pose);
        virtual void    directKinematics(const Motion phi[], Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration);
        virtual void    inverseKinematics(Tool& tool, Transformation& pose, double phi[], int16_t configuration);
        virtual void    inverseKinematics(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], int16_t configuration);
        virtual void    inverseKinematics(const double phi[], double q[]);
        virtual void    inverseKinematics(const Motion phi[], Motion q[]);
        
    private:
        
        uint16_t        numberOfActuators;
        uint16_t        numberOfJoints;
};

#endif /* ROBOT_MODEL_H_ */
