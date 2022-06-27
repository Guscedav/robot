/*
 * TaskSetDO.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 12.10.2016
 *      Author: Marcel Honegger
 */

#ifndef TASK_SET_DO_H_
#define TASK_SET_DO_H_

#include <cstdlib>
#include <stdint.h>
#include "Task.h"

class RobotModel;
class Tool;
class Transformation;
class Motion;
class DigitalOut;

/**
 * This is a task that sets a digital output.
 */
class TaskSetDO : public Task {
    
    public:
        
                    TaskSetDO(RobotModel& robotModel, Tool& tool, DigitalOut& digitalOut, bool value);
        virtual     ~TaskSetDO();
        void        init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        int16_t     increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        void        exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        
    private:
        
        DigitalOut& digitalOut;
        bool        value;
};

#endif /* TASK_SET_DO_H_ */
