/*
 * TaskWait.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.10.2016
 *      Author: Marcel Honegger
 */

#ifndef TASK_WAIT_H_
#define TASK_WAIT_H_

#include <cstdlib>
#include <stdint.h>
#include "Task.h"

class RobotModel;
class Tool;
class Transformation;
class Motion;

/**
 * This is a task that waits for a given duration.
 */
class TaskWait : public Task {
    
    public:
        
                    TaskWait(RobotModel& robotModel, Tool& tool, float duration);
        virtual     ~TaskWait();
        void        init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        int16_t     increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        void        exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        
    private:
        
        float       duration;
        float       time;
};

#endif /* TASK_WAIT_H_ */
