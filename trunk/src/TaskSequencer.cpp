/*
 * TaskSequencer.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 06.10.2016
 *      Author: Marcel Honegger
 */

#include "RobotModel.h"
#include "RobotController.h"
#include "Task.h"
#include "TaskSequencer.h"

using namespace std;

const float TaskSequencer::MIN_VELOCITY_OVERRIDE = 0.01f;
const float TaskSequencer::MAX_VELOCITY_OVERRIDE = 2.0f;

/**
 * Creates a task sequencer object with references to a robot model and to a robot controller.
 * @param robotModel a reference to a robot model this task sequencer depends on.
 * @param robotController a reference to a robot controller this task sequencer uses.
 */
TaskSequencer::TaskSequencer(RobotModel& robotModel, RobotController& robotController) : RealtimeThread("TaskSequencer", STACK_SIZE), robotModel(robotModel), robotController(robotController) {

    phi = new Motion[robotModel.getNumberOfJoints()];
    q = new Motion[robotModel.getNumberOfActuators()];

    phiTarget = new Motion[robotModel.getNumberOfJoints()];
    qTarget = new Motion[robotModel.getNumberOfActuators()];
    
    state = NO_TASK_LIST;
    velocityOverride = 1.0f;
}

/**
 * Deletes this object and releases all allocated resources.
 */
TaskSequencer::~TaskSequencer() {
    
	delete[] qTarget;
	delete[] phiTarget;

    delete[] q;
    delete[] phi;
}

/**
 * Sets a new task to process.
 * Any pending tasks in the task list are deleted by this method, before the new given task is added to the task list.
 * @param task a reference to a new task to process.
 */
void TaskSequencer::setTask(Task* task) {
    
    mutex.lock();
    
    if (task != NULL) {
        if (tasks.size() > 0) {
            Task* task = tasks.front();
            task->exit(pose, velocity, acceleration, phi, q, poseTarget, velocityTarget, accelerationTarget, phiTarget, qTarget);
        }
        while (tasks.size() > 0) {
            Task* task = tasks.front();
            delete task;
            tasks.pop_front();
        }
        tasks.push_back(task);
        task->init(tool, pose, velocity, acceleration, phi, q, poseTarget, velocityTarget, accelerationTarget, phiTarget, qTarget);
        state = PROCESSING_TASK_LIST;
    } else {
        // leave task list unchanged
    }
    
    mutex.unlock();
}

/**
 * Adds a new task to the existing task list to process.
 * @param task a reference to a new task to add to the task list to process.
 */
void TaskSequencer::addTask(Task* task) {
    
    mutex.lock();
    
    if (task != NULL) {
        if (tasks.size() == 0) {
            tasks.push_back(task);
            task->init(tool, pose, velocity, acceleration, phi, q, poseTarget, velocityTarget, accelerationTarget, phiTarget, qTarget);
        } else {
            tasks.push_back(task);
        }
        state = PROCESSING_TASK_LIST;
    } else {
        // leave task list unchanged
    }
    
    mutex.unlock();
}

/**
 * Sets the velocity override parameter for all motion tasks.
 * The desired profile velocity value of motion tasks is multiplied by this value.
 * @param velocityOverride the velocity override parameter. This value must be in the range [0.01 .. 2].
 */
void TaskSequencer::setVelocityOverride(float velocityOverride) {
    
    if (velocityOverride < MIN_VELOCITY_OVERRIDE) this->velocityOverride = MIN_VELOCITY_OVERRIDE;
    else if (velocityOverride > MAX_VELOCITY_OVERRIDE) this->velocityOverride = MAX_VELOCITY_OVERRIDE;
    else this->velocityOverride = velocityOverride;
}

/**
 * Gets the actual velocity override parameter.
 * @return the actual velocity override parameter.
 */
float TaskSequencer::getVelocityOverride() {
    
    return velocityOverride;
}

/**
 * Gets the actual state of the task sequencer.
 * @return the actual state, either <code>FAULT</code>, <code>NO_TASK_LIST</code>,
 * <code>PROCESSING_TASK_LIST</code> or <code>TASK_LIST_DONE</code>.
 */
int16_t TaskSequencer::getState() {
    
    return state;
}

/**
 * Gets the number of remaining tasks in the task list to process.
 * @return the number of remaining tasks.
 */
size_t TaskSequencer::getTaskListLength() {
    
    size_t size = 0;
    
    mutex.lock();
    
    if (state == PROCESSING_TASK_LIST) size = tasks.size();
    
    mutex.unlock();
    
    return size;
}

/**
 * Gets a referece to the task that is curretly running.
 * @return a reference to the current task, or <code>NULL</code> if the task list is empty.
 */
Task* TaskSequencer::getTask() {
    
    Task* task = NULL;
    
    mutex.lock();
    
    if (tasks.size() > 0) {
        task = tasks.front();
    }
    
    mutex.unlock();
    
    return task;
}

/**
 * Starts the task sequencer.
 */
void TaskSequencer::start() {
    
    if (!RealtimeThread::isAlive()) {
        
        mutex.lock();
        
        while (tasks.size() > 0) {
            Task* task = tasks.front();
            delete task;
            tasks.pop_front();
        }
        
        robotController.getTool(tool);

        robotController.getDesiredCartesianPose(pose);
        robotController.getDesiredCartesianVelocity(velocity);
        robotController.getDesiredCartesianAcceleration(acceleration);
        robotController.getDesiredJointAngles(phi);
        robotController.getDesiredActuatorCoordinates(q);

        robotController.getDesiredCartesianPose(poseTarget);
        robotController.getDesiredCartesianVelocity(velocityTarget);
        robotController.getDesiredCartesianAcceleration(accelerationTarget);
        robotController.getDesiredJointAngles(phiTarget);
        robotController.getDesiredActuatorCoordinates(qTarget);
        
        mutex.unlock();
        
        RealtimeThread::start();
    }
}

/**
 * Stops the task sequencer and resets the desired velocity and acceleration values.
 */
void TaskSequencer::stop() {
    
    if (RealtimeThread::isAlive()) {
        
        RealtimeThread::stop();
        
        mutex.lock();
        
        for (uint16_t i = 0; i < 12; i++) {
            velocity[i] = 0.0;
            acceleration[i] = 0.0;
        }
        
        for (uint16_t i = 0; i < robotModel.getNumberOfJoints(); i++) {
            phi[i].velocity = 0.0f;
            phi[i].acceleration = 0.0f;
        }
        
        for (uint16_t i = 0; i < robotModel.getNumberOfActuators(); i++) {
            q[i].velocity = 0.0f;
            q[i].acceleration = 0.0f;
        }
        
        robotController.setDesiredCartesianPose(pose);
        robotController.setDesiredCartesianVelocity(velocity);
        robotController.setDesiredCartesianAcceleration(acceleration);
        robotController.setDesiredJointAngles(phi);
        robotController.setDesiredActuatorCoordinates(q);
        
        while (tasks.size() > 0) {
            Task* task = tasks.front();
            delete task;
            tasks.pop_front();
        }
        
        mutex.unlock();
    }
}

/**
 * This is the run logic of the task sequencer.
 */
void TaskSequencer::run() {
    
    while (waitForNextPeriod()) try {
        
        mutex.lock();
        
        if (tasks.size() > 0) {
            
            Task* task = tasks.front();
            int16_t result = task->increment(pose, velocity, acceleration, phi, q, velocityOverride, static_cast<float>(getPeriod()));
            
            robotController.setTool(tool);
        	robotController.setDesiredCartesianPose(pose);
            robotController.setDesiredCartesianVelocity(velocity);
            robotController.setDesiredCartesianAcceleration(acceleration);
            robotController.setDesiredJointAngles(phi);
            robotController.setDesiredActuatorCoordinates(q);
            
            if (result == Task::FAULT) {
                
                state = FAULT;
                
            } else if (result == Task::DONE) {
                
                if (tasks.size() > 1) {
                    
                    task->exit(pose, velocity, acceleration, phi, q, poseTarget, velocityTarget, accelerationTarget, phiTarget, qTarget);
                    delete task;
                    tasks.pop_front();
                    task = tasks.front();
                    task->init(tool, pose, velocity, acceleration, phi, q, poseTarget, velocityTarget, accelerationTarget, phiTarget, qTarget);
                    
                    state = PROCESSING_TASK_LIST;
                    
                } else {
                    
                    state = TASK_LIST_DONE;
                }
                
            } else {
                
                state = PROCESSING_TASK_LIST;
            }
            
        } else {
            
            state = NO_TASK_LIST;
        }
        
        mutex.unlock();
        
    } catch (exception& e) {
		
		cerr << "TaskSequencer: " << e.what() << endl;
	}
}
