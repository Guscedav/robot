/*
 * Tool.cpp
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 22.01.2016
 *      Author: Marcel Honegger
 */

#include "Tool.h"

using namespace std;

/**
 * Creates a tool object and initializes all parameters to zero.
 */
Tool::Tool() {

    mass = 0.0f;
}

/**
 * Creates a tool object and initializes the tool center point to the given transformation.
 * The mass and the mass center point are initialized to zero.
 * @param tcpPose the transformation to the tool center point.
 */
Tool::Tool(Transformation& tcpPose) {

    this->tcpPose.set(tcpPose);
    this->mass = 0.0f;
}

/**
 * Creates a tool object and initializes the tool center point, the mass and the mass center point to the given values.
 * @param tcpPose the transformation to the tool center point.
 * @param mass the mass of this tool, given in [kg].
 * @param massCenterPoint the vector to the mass center point.
 */
Tool::Tool(Transformation& tcpPose, float mass, Vector& massCenterPoint) {

    this->tcpPose.set(tcpPose);
    this->mass = mass;
    this->massCenterPoint.set(massCenterPoint);
}

/**
 * Deletes the tool object.
 */
Tool::~Tool() {}

/**
 * Sets the properties of this tool object to those of another tool.
 * @param tool another tool object to copy the values from.
 */
void Tool::set(Tool& tool) {
    
    tcpPose.set(tool.tcpPose);
    mass = tool.mass;
    massCenterPoint.set(tool.massCenterPoint);
}

/**
 * Sets the properties of this tool object to those of another tool.
 * @param tool another tool object to copy the values from.
 */
void Tool::set(Transformation& tcpPose, float mass, Vector& massCenterPoint) {
    
    this->tcpPose.set(tcpPose);
    this->mass = mass;
    this->massCenterPoint.set(massCenterPoint);
}

/**
 * Gets the transformation to the tool center point.
 * @param tcpPose the transformation to the tool center point.
 */
void Tool::getTCPPose(Transformation& tcpPose) {
    
    tcpPose.set(this->tcpPose);
}

/**
 * Gets the mass of this tool.
 * @return the mass of this tool, given in [kg].
 */
float Tool::getMass() {
    
    return mass;
}

/**
 * Gets the vector to the mass center point.
 * @param massCenterPoint the vector to the mass center point.
 */
void Tool::getMassCenterPoint(Vector& massCenterPoint) {
    
    massCenterPoint.set(this->massCenterPoint);
}
