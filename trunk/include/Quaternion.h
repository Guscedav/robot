/*
 * Quaternion.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 16.11.2015
 *      Author: Marcel Honegger
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <cstdlib>
#include <string>
#include <stdint.h>

class Quaternion {

    public:
                        
                    Quaternion();
                    Quaternion(double q[]);
                    Quaternion(double q0, double q1, double q2, double q3);
                    Quaternion(const Quaternion& quaternion);
        virtual     ~Quaternion();
        double&     operator[] (const uint16_t i);
        void        set(double q[]);
        void        set(double q0, double q1, double q2, double q3);
        void        set(const Quaternion& quaternion);
        void        normalize();
        std::string toString();
        
    private:
        
        double      q[4];
};

#endif /* QUATERNION_H_ */
