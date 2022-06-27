/*
 * Matrix.h
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.11.2015
 *      Author: Marcel Honegger
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <cstdlib>
#include <string>
#include <stdint.h>

class Quaternion;
class Vector;

/**
 * This class represents a 3x3 matrix used to describe the orientation of a coordinate system.
 * It offers methods to invert this matrix, to multiply itself with another matrix, to rotate
 * the represented coordinate system around the local X, Y or Z axis, and a number of other
 * miscellaneous methods.
 */
class Matrix {

    public:
                        
                    Matrix();
                    Matrix(double m[]);
                    Matrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);
                    Matrix(double alpha, double beta, double gamma);
                    Matrix(Quaternion& quaternion);
                    Matrix(const Matrix& matrix);
        virtual     ~Matrix();
        double&     operator[] (const uint16_t i);
        void        set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);
        void        set(double alpha, double beta, double gamma);
        void        set(Quaternion& quaternion);
        void        set(const Matrix& matrix);
        void        normalize();
        double      determinant();
        void        invert();
        void        transpose();
        void        add(double scalar);
        void        add(const Matrix& Matrix);
        void        mul(double scalar);
        void        mul(const Matrix& matrix);
        void        mul(Vector& vector, Vector& result);
        void        rotX(double angle);
        void        rotY(double angle);
        void        rotZ(double angle);
        void        getXYZEulerAngles(double& alpha, double& beta, double& gamma);
        void        getZYXEulerAngles(double& alpha, double& beta, double& gamma);
        void        getQuaternion(Quaternion& quaternion);
        std::string toString();
        
    private:

        double      m[9];
};

#endif /* MATRIX_H_ */
