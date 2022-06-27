/*
 * TaskMoveJoint.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.10.2016
 *      Author: Marcel Honegger
 */

#ifndef TASK_MOVE_JOINT_H_
#define TASK_MOVE_JOINT_H_

#include <cstdlib>
#include <stdint.h>
#include "Task.h"

class RobotModel;
class Tool;
class Transformation;
class Motion;

/**
 * This is a task that moves the robot to a given joint angle configuration.
 */
class TaskMoveJoint : public Task {
    
    public:
        
                    TaskMoveJoint(RobotModel& robotModel, Tool& tool, const double phi[], float velocity, double zone);
        virtual     ~TaskMoveJoint();
        void        init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        int16_t     increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        void        exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        
    private:
        
        static const double MINIMUM_ZONE;
        
        double*     phi;
        float       velocity;
        double      zone;
};

#endif /* TASK_MOVE_JOINT_H_ */
