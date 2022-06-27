/*
 * TaskStop.h
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 08.09.2017
 *      Author: Marcel Honegger
 */

#ifndef TASK_STOP_H_
#define TASK_STOP_H_

#include <cstdlib>
#include <stdint.h>
#include "Task.h"

class RobotModel;
class Tool;
class Transformation;
class Motion;

/**
 * This is a task that stops the current motion.
 */
class TaskStop : public Task {
    
    public:
        
                    TaskStop(RobotModel& robotModel, Tool& tool);
        virtual     ~TaskStop();
        void        init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        int16_t     increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        void        exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);

    private:
    
        static const float VELOCITY_THRESHOLD;
};

#endif /* TASK_STOP_H_ */
