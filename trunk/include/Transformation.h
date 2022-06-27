/*
 * Transformation.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 27.11.2015
 *      Author: Marcel Honegger
 */

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include <cstdlib>
#include <string>
#include <stdint.h>

class Vector;
class Matrix;
class Quaternion;

/**
 * This class represents a 4x4 homogeneous transformation matrix used to describe the pose
 * of a coordinate system. It offers methods to multiply itself with another transformation,
 * to rotate the represented coordinate system around the local X, Y or Z axis, to translate
 * the coordinate system in X, Y and Z direction, and a number of other miscellaneous methods.
 */
class Transformation {
    
    public:
        
                    Transformation();
                    Transformation(double t[]);
                    Transformation(double t00, double t01, double t02, double t03, double t10, double t11, double t12, double t13, double t20, double t21, double t22, double t23, double t30, double t31, double t32, double t33);
                    Transformation(double x, double y, double z, double alpha, double beta, double gamma);
                    Transformation(Matrix& matrix, Vector& vector);
                    Transformation(Quaternion& quaternion, Vector& vector);
                    Transformation(const Transformation& transformation);
        virtual     ~Transformation();
        double&     operator[] (const uint16_t i);
        void        set(Matrix& matrix);
        void        set(Quaternion& quaternion);
        void        set(Vector& vector);
        void        set(double t00, double t01, double t02, double t03, double t10, double t11, double t12, double t13, double t20, double t21, double t22, double t23, double t30, double t31, double t32, double t33);
        void        set(double x, double y, double z, double alpha, double beta, double gamma);
        void        set(Matrix& matrix, Vector& vector);
        void        set(Quaternion& quaternion, Vector& Vector);
        void        set(const Transformation& transformation);
        void        normalize();
        void        invert();
        void        mul(const Transformation& transformation);
        void        rotX(double angle);
        void        rotY(double angle);
        void        rotZ(double angle);
        void        transX(double x);
        void        transY(double y);
        void        transZ(double z);
        void        get(Matrix& matrix, Vector& vector);
        void        getMatrix(Matrix& matrix);
        void        getVector(Vector& vector);
        void        getXYZEulerAngles(double& alpha, double& beta, double& gamma);
        void        getZYXEulerAngles(double& alpha, double& beta, double& gamma);
        void        getQuaternion(Quaternion& quaternion);
        std::string toString();
        
    private:
        
        double      t[16];
};

#endif /* TRANSFORMATION_H_ */
