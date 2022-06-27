/*
 * Tool.h
 * Copyright (c) 2016, ZHAW
 * All rights reserved.
 *
 *  Created on: 22.01.2016
 *      Author: Marcel Honegger
 */

#ifndef TOOL_H_
#define TOOL_H_

#include <cstdlib>
#include "Transformation.h"
#include "Vector.h"

/**
 * This class represents the properties of a tool that can be
 * connected to the last joint of a robot system.
 */
class Tool {

    public:

                        Tool();
                        Tool(Transformation& tcpPose);
                        Tool(Transformation& tcpPose, float mass, Vector& massCenterPoint);
        virtual         ~Tool();
        void            set(Tool& tool);
        void            set(Transformation& tcpPose, float mass, Vector& massCenterPoint);
        void            getTCPPose(Transformation& tcpPose);
        float           getMass();
        void            getMassCenterPoint(Vector& massCenterPoint);

    private:

        Transformation  tcpPose;
        float           mass;
        Vector          massCenterPoint;
};

#endif /* TOOL_H_ */
