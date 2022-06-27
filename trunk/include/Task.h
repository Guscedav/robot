/*
 * Task.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 05.10.2016
 *      Author: Marcel Honegger
 */

#ifndef TASK_H_
#define TASK_H_

#include <cstdlib>
#include <stdint.h>

class RobotModel;
class Tool;
class Transformation;
class Motion;

/**
 * This is an abstract class that defines methods for robot tasks.
 */
class Task {
    
    public:
        
        static const int16_t    FAULT = -1;     /**< Return value of increment() method. */
        static const int16_t    RUNNING = 0;    /**< Return value of increment() method. */
        static const int16_t    DONE = 1;       /**< Return value of increment() method. */
        
                        Task(RobotModel& robotModel, Tool& tool);
        virtual         ~Task();
        virtual void    init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        virtual int16_t increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        virtual void    exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        
    protected:
        
        RobotModel&     robotModel;
        Tool            tool;
};

#endif /* TASK_H_ */
