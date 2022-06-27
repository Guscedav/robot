/*
 * TaskMoveToolWithJoystick.h
 * Copyright (c) 2018, ZHAW
 * All rights reserved.
 *
 *  Created on: 04.04.2018
 *      Author: Marcel Honegger
 */

#ifndef TASK_MOVE_TOOL_WITH_JOYSTICK_H_
#define TASK_MOVE_TOOL_WITH_JOYSTICK_H_

#include <cstdlib>
#include <stdint.h>
#include "Task.h"
#include "Motion.h"

class RobotModel;
class Tool;
class AnalogIn;
class Transformation;

/**
 * This is a task that moves the tool of the robot with velocities given by an analog joystick.
 */
class TaskMoveToolWithJoystick : public Task {

    public:

					TaskMoveToolWithJoystick(RobotModel& robotModel, Tool& tool, int16_t configuration, AnalogIn& velocityX, AnalogIn& velocityY, AnalogIn& velocityZ, AnalogIn& velocityA, AnalogIn& velocityB, AnalogIn& velocityC);
        virtual     ~TaskMoveToolWithJoystick();
        void        init(Tool& tool, Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);
        int16_t     increment(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], float velocityOverride, float period);
        void        exit(Transformation& pose, Transformation& velocity, Transformation& acceleration, Motion phi[], Motion q[], Transformation& poseTarget, Transformation& velocityTarget, Transformation& accelerationTarget, Motion phiTarget[], Motion qTarget[]);

    private:

        int16_t         configuration;
        AnalogIn&		velocityX;
        AnalogIn&		velocityY;
        AnalogIn& 		velocityZ;
        AnalogIn& 		velocityA;
        AnalogIn& 		velocityB;
        AnalogIn& 		velocityC;

        Motion          x;
        Motion          y;
        Motion          z;
        Motion          a;
        Motion          b;
        Motion          c;
};

#endif /* TASK_MOVE_TOOL_WITH_JOYSTICK_H_ */
