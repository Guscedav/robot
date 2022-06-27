/*
 * TaskMoveLinear.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 27.10.2016
 *      Author: Marcel Honegger
 */

#ifndef TASK_MOVE_LINEAR_H_
#define TASK_MOVE_LINEAR_H_

#include <cstdlib>
#include <stdint.h>
#include "Task.h"
#include "Transformation.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Motion.h"

class RobotModel;
class Tool;

/**
 * This is a task that moves the robot to a given Cartesian pose.
 */
class TaskMoveLinear : public Task {
    
    public:
        
                    TaskMoveLinear(RobotModel& robotModel, Tool& tool, Transformation& pose, int16_t configuration, float velocity);
                    TaskMoveLinear(RobotModel& robotModel, Tool& tool, Transformation& pose, int16_t configuration, float velocity, double zone);
        virtual     ~TaskMoveLinear();
        void        init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        int16_t     increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        void        exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
    
    private:
    
        static const double MINIMUM_ZONE;
    
        Transformation  pose;
        int16_t         configuration;
        float           velocity;
        double          zone;
    
        Matrix          localBase;
        Quaternion      quaternion;
        float           rLinear[3][3];
        float           rRotational[4][4];
        Motion          x;
        Motion          y;
        Motion          z;
        Motion          q0;
        Motion          q1;
        Motion          q2;
        Motion          q3;
};

#endif /* TASK_MOVE_LINEAR_H_ */
