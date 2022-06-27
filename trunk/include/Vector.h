/*
 * Vector.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.11.2015
 *      Author: Marcel Honegger
 */

#ifndef VECTOR_H_
#define VECTOR_H_

#include <cstdlib>
#include <string>
#include <stdint.h>

/**
 * This class represents a 3-dimensional vector. It may be used to describe the origin
 * of a coordinate system with respect to another system.
 * It offers methods to calculate the length, normalize this vector, add another
 * vector to this one, and some other miscellaneous methods.
 */
class Vector {
    
    public:
        
                    Vector();
                    Vector(double v[]);
                    Vector(double v0, double v1, double v2);
                    Vector(const Vector& vector);
        virtual     ~Vector();
        double&     operator[] (const uint16_t i);
        void        set(const Vector& vector);
        double      length();
        void        normalize();
        void        add(const Vector& vector);
        void        scale(double scalar);
        void        cross(const Vector& v1, const Vector& v2);
        std::string toString();
        
    private:
        
        double      v[3];
};

#endif /* VECTOR_H_ */
