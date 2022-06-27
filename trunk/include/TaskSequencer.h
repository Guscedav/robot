/*
 * TaskSequencer.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 06.10.2016
 *      Author: Marcel Honegger
 */

#ifndef TASK_SEQUENCER_H_
#define TASK_SEQUENCER_H_

#include <cstdlib>
#include <deque>
#include <iostream>
#include <exception>
#include <stdint.h>
#include <pthread.h>
#include "RealtimeThread.h"
#include "Mutex.h"
#include "Motion.h"
#include "Transformation.h"
#include "Tool.h"

class RobotModel;
class RobotController;
class Task;

/**
 * The task sequencer processes a list of tasks, like tasks to move the robot to
 * a given pose, tasks to control a digital output (i.e. to control a gripper),
 * tasks to wait for a specified duration, etc.
 * <br/>
 * This class offers a number of methods to set tasks to process and to check the
 * state of the task sequencer. And since it is a realtime thread doing the processing,
 * it also offers methods to set this thread's priority and period, and methods
 * to start and stop this thread.
 */
class TaskSequencer : public RealtimeThread {
    
    public:
        
        static const int16_t    FAULT = -1;                 /**< Task sequencer state. */
        static const int16_t    NO_TASK_LIST = 0;           /**< Task sequencer state. */
        static const int16_t    PROCESSING_TASK_LIST = 1;   /**< Task sequencer state. */
        static const int16_t    TASK_LIST_DONE = 2;         /**< Task sequencer state. */
        
                    TaskSequencer(RobotModel& robotModel, RobotController& robotController);
        virtual     ~TaskSequencer();
        void        setTask(Task* task);
        void        addTask(Task* task);
        void        setVelocityOverride(float velocityOverride);
        float       getVelocityOverride();
        int16_t     getState();
        size_t      getTaskListLength();
        Task*       getTask();
        void        start();
        void        stop();
        
    private:
        
        static const size_t STACK_SIZE = 64*1024;   // stack size of task sequencer thread in [bytes]
        static const float  MIN_VELOCITY_OVERRIDE;
        static const float  MAX_VELOCITY_OVERRIDE;
        
        RobotModel&         robotModel;
        RobotController&    robotController;
        Tool                tool;
        
        Transformation      pose;
        Transformation      velocity;
        Transformation      acceleration;
        Motion*             phi;
        Motion*             q;
        
        Transformation		poseTarget;
        Transformation		velocityTarget;
        Transformation		accelerationTarget;
        Motion*             phiTarget;
        Motion*             qTarget;

        std::deque<Task*>   tasks;
        int16_t             state;
        float               velocityOverride;
        Mutex               mutex;
        
        void        run();
};

#endif /* TASK_SEQUENCER_H_ */
