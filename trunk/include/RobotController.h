/*
 * RobotController.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 10.08.2016
 *      Author: Marcel Honegger
 */

#ifndef ROBOT_CONTROLLER_H_
#define ROBOT_CONTROLLER_H_

#include <cstdlib>
#include <stdint.h>
#include "Tool.h"
#include "Transformation.h"

class RobotModel;
class Motion;

/**
 * This abstract class implements the interface of a robot controller.
 * The robot controller contains the actuator control algorithms of a robotic manipulator.
 * The interface of this class therefore offers methods to set desired values and to read
 * actual values and control errors.
 */
class RobotController {
    
    public:
        
                        RobotController(RobotModel& robotModel);
        virtual         ~RobotController();
        void            setTool(Tool& tool);
        void            getTool(Tool& tool);
        virtual void    setDesiredActuatorCoordinates(const Motion q[]);
        virtual void    getDesiredActuatorCoordinates(Motion q[]);
        virtual void    getActualActuatorCoordinates(Motion q[]);
        virtual void    getActuatorCoordinateErrors(Motion q[]);
        virtual void    setDesiredJointAngles(const Motion phi[]);
        virtual void    getDesiredJointAngles(Motion phi[]);
        virtual void    getActualJointAngles(Motion phi[]);
        virtual void    getJointAngleErrors(Motion phi[]);
        virtual void    setDesiredCartesianPose(Transformation& pose);
        virtual void    getDesiredCartesianPose(Transformation& pose);
        virtual void    getActualCartesianPose(Transformation& pose);
        virtual void    getCartesianPoseError(Transformation& pose);
        virtual void    setDesiredCartesianVelocity(Transformation& velocity);
        virtual void    getDesiredCartesianVelocity(Transformation& velocity);
        virtual void    getActualCartesianVelocity(Transformation& velocity);
        virtual void    getCartesianVelocityError(Transformation& velocity);
        virtual void    setDesiredCartesianAcceleration(Transformation& acceleration);
        virtual void    getDesiredCartesianAcceleration(Transformation& acceleration);
        virtual void    getActualCartesianAcceleration(Transformation& acceleration);
        virtual void    getCartesianAccelerationError(Transformation& acceleration);
        
    protected:
        
        RobotModel&     robotModel;
        Tool            tool;
        
        Motion*         qDesired;
        Motion*         qActual;
        Motion*         qError;
        
        Motion*         phiDesired;
        Motion*         phiActual;
        Motion*         phiError;
        
        Transformation  poseDesired;
        Transformation  poseActual;
        Transformation  poseError;
        
        Transformation  velocityDesired;
        Transformation  velocityActual;
        Transformation  velocityError;
        
        Transformation  accelerationDesired;
        Transformation  accelerationActual;
        Transformation  accelerationError;
};

#endif /* ROBOT_CONTROLLER_H_ */
