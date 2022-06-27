/*
 * TaskWaitDI.h
 * Copyright (c) 2017, ZHAW
 * All rights reserved.
 *
 *  Created on: 08.09.2017
 *      Author: Marcel Honegger
 */

#ifndef TASK_WAIT_DI_H_
#define TASK_WAIT_DI_H_

#include <cstdlib>
#include <stdint.h>
#include "Task.h"

class RobotModel;
class Tool;
class Transformation;
class Motion;
class DigitalIn;

/**
 * This is a task that waits for a signal from a digital input.
 */
class TaskWaitDI : public Task {
    
    public:
        
                    TaskWaitDI(RobotModel& robotModel, Tool& tool, DigitalIn& digitalIn, bool value);
        virtual     ~TaskWaitDI();
        void        init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        int16_t     increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        void        exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);

    private:
        
        DigitalIn&  digitalIn;
        bool        value;
};

#endif /* TASK_WAIT_DI_H_ */
