/*
 * Transformation.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 27.11.2015
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <stdexcept>
#include <sstream>
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Transformation.h"

using namespace std;

/**
 * Creates a transformation object and initializes the values to a unit matrix.
 */
Transformation::Transformation() {
    
     t[0] = 1.0;  t[1] = 0.0;  t[2] = 0.0;  t[3] = 0.0;
     t[4] = 0.0;  t[5] = 1.0;  t[6] = 0.0;  t[7] = 0.0;
     t[8] = 0.0;  t[9] = 0.0; t[10] = 1.0; t[11] = 0.0;
    t[12] = 0.0; t[13] = 0.0; t[14] = 0.0; t[15] = 1.0;
}

/**
 * Creates a transformation object and initializes the values to the given sixteen-element array.
 * @param t the array of length 16 containing initial values for the transformation.
 */
Transformation::Transformation(double t[]) {
    
    for (uint16_t i = 0; i < 16; i++) {
        this->t[i] = t[i];
    }
}

/**
 * Creates a transformation object and initializes the values to the given sixteen matrix values.
 * @param t00 the [0][0] element of the transformation.
 * @param t01 the [0][1] element of the transformation.
 * @param t02 the [0][2] element of the transformation.
 * @param t03 the [0][3] element of the transformation.
 * @param t10 the [1][0] element of the transformation.
 * @param t11 the [1][1] element of the transformation.
 * @param t12 the [1][2] element of the transformation.
 * @param t13 the [1][3] element of the transformation.
 * @param t20 the [2][0] element of the transformation.
 * @param t21 the [2][1] element of the transformation.
 * @param t22 the [2][2] element of the transformation.
 * @param t23 the [2][3] element of the transformation.
 * @param t30 the [3][0] element of the transformation.
 * @param t31 the [3][1] element of the transformation.
 * @param t32 the [3][2] element of the transformation.
 * @param t33 the [3][3] element of the transformation.
 */
Transformation::Transformation(double t00, double t01, double t02, double t03, double t10, double t11, double t12, double t13, double t20, double t21, double t22, double t23, double t30, double t31, double t32, double t33) {
    
    t[ 0] = t00; t[ 1] = t01; t[ 2] = t02; t[ 3] = t03;
    t[ 4] = t10; t[ 5] = t11; t[ 6] = t12; t[ 7] = t13;
    t[ 8] = t20; t[ 9] = t21; t[10] = t22; t[11] = t23;
    t[12] = t30; t[13] = t31; t[14] = t32; t[15] = t33;
}

/**
 * Creates a transformation object and initializes the values based on the given
 * linear translation and the Z-Y-X Euler angles.
 * @param x the translation along the X axis, given in [m].
 * @param y the translation along the Y axis, given in [m].
 * @param z the translation along the Z axis, given in [m].
 * @param alpha the angle to rotate about the X axis, given in [rad].
 * @param beta the angle to rotate about the Y axis, given in [rad].
 * @param gamma the angle to rotate about the Z axis, given in [rad].
 */
Transformation::Transformation(double x, double y, double z, double alpha, double beta, double gamma) {
    
    t[ 0] = cos(gamma)*cos(beta); t[ 1] = -sin(gamma)*cos(alpha)+cos(gamma)*sin(beta)*sin(alpha); t[ 2] = sin(gamma)*sin(alpha)+cos(gamma)*sin(beta)*cos(alpha);  t[ 3] = x;
    t[ 4] = sin(gamma)*cos(beta); t[ 5] = cos(gamma)*cos(alpha)+sin(gamma)*sin(beta)*sin(alpha);  t[ 6] = -cos(gamma)*sin(alpha)+sin(gamma)*sin(beta)*cos(alpha); t[ 7] = y;
    t[ 8] = -sin(beta);           t[ 9] = cos(beta)*sin(alpha);                                   t[10] = cos(beta)*cos(alpha);                                   t[11] = z;
    t[12] = 0.0;                  t[13] = 0.0;                                                    t[14] = 0.0;                                                    t[15] = 1.0;
}

/**
 * Creates a transformation object from the given rotation matrix and translation vector.
 * @param matrix the matrix representing the rotational component.
 * @param vector the translational component of the transformation.
 */
Transformation::Transformation(Matrix& matrix, Vector& vector) {
    
    t[ 0] = matrix[0]; t[ 1] = matrix[1]; t[ 2] = matrix[2]; t[ 3] = vector[0];
    t[ 4] = matrix[3]; t[ 5] = matrix[4]; t[ 6] = matrix[5]; t[ 7] = vector[1];
    t[ 8] = matrix[6]; t[ 9] = matrix[7]; t[10] = matrix[8]; t[11] = vector[2];
    t[12] = 0.0;       t[13] = 0.0;       t[14] = 0.0;       t[15] = 1.0;
}

/**
 * Creates a transformation object from the given quaternion and translation vector.
 * @param quaternion the quaternion describing the orientation.
 * @param vector the translational component of the transformation.
 */
Transformation::Transformation(Quaternion& quaternion, Vector& vector) {
    
    t[0] = 1.0-2.0*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
    t[1] = 2.0*(quaternion[1]*quaternion[2]-quaternion[0]*quaternion[3]);
    t[2] = 2.0*(quaternion[1]*quaternion[3]+quaternion[0]*quaternion[2]);
    t[3] = vector[0];
    
    t[4] = 2.0*(quaternion[1]*quaternion[2]+quaternion[0]*quaternion[3]);
    t[5] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[3]*quaternion[3]);
    t[6] = 2.0*(quaternion[2]*quaternion[3]-quaternion[0]*quaternion[1]);
    t[7] = vector[1];
    
    t[8] = 2.0*(quaternion[1]*quaternion[3]-quaternion[0]*quaternion[2]);
    t[9] = 2.0*(quaternion[2]*quaternion[3]+quaternion[0]*quaternion[1]);
    t[10] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]);
    t[11] = vector[2];
    
    t[12] = 0.0;
    t[13] = 0.0;
    t[14] = 0.0;
    t[15] = 1.0;
}

/**
 * Creates a new transformation object with the same values as the given transformation.
 * @param transformation the source transformation.
 */
Transformation::Transformation(const Transformation& transformation) {
    
    for (uint16_t i = 0; i < 16; i++) {
        t[i] = transformation.t[i];
    }
}

/**
 * Deletes the transformation object.
 */
Transformation::~Transformation() {}

/**
 * Access a single element of this matrix.
 * This method throws an <code>out_of_range</code> exception if the index is out of range.
 */
double& Transformation::operator[] (const uint16_t i) {
    
    if (i >= 16) throw out_of_range("Transformation: index number is out of range.");
    
    return t[i];
}

/**
 * Sets the orientation of this transformation object to the orientation
 * of the given rotation matrix.
 * @param matrix the matrix representing the orientation.
 */
void Transformation::set(Matrix& matrix) {
    
    t[ 0] = matrix[0]; t[ 1] = matrix[1]; t[ 2] = matrix[2];
    t[ 4] = matrix[3]; t[ 5] = matrix[4]; t[ 6] = matrix[5];
    t[ 8] = matrix[6]; t[ 9] = matrix[7]; t[10] = matrix[8];
}

/**
 * Sets the orientation of this transformation object to the orientation
 * of the given quaternion.
 * @param quaternion the quaternion describing the orientation.
 */
void Transformation::set(Quaternion& quaternion) {
    
    t[0] = 1.0-2.0*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
    t[1] = 2.0*(quaternion[1]*quaternion[2]-quaternion[0]*quaternion[3]);
    t[2] = 2.0*(quaternion[1]*quaternion[3]+quaternion[0]*quaternion[2]);
    
    t[4] = 2.0*(quaternion[1]*quaternion[2]+quaternion[0]*quaternion[3]);
    t[5] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[3]*quaternion[3]);
    t[6] = 2.0*(quaternion[2]*quaternion[3]-quaternion[0]*quaternion[1]);
    
    t[8] = 2.0*(quaternion[1]*quaternion[3]-quaternion[0]*quaternion[2]);
    t[9] = 2.0*(quaternion[2]*quaternion[3]+quaternion[0]*quaternion[1]);
    t[10] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]);
}

/**
 * Sets the translational component of this transformation object to the values
 * of the given translation vector.
 * @param vector the translational component of the transformation.
 */
void Transformation::set(Vector& vector) {
    
    t[3] = vector[0];
    t[7] = vector[1];
    t[11] = vector[2];
}

/**
 * Sets the values of this transformation object to the given sixteen matrix values.
 * @param t00 the [0][0] element of the transformation.
 * @param t01 the [0][1] element of the transformation.
 * @param t02 the [0][2] element of the transformation.
 * @param t03 the [0][3] element of the transformation.
 * @param t10 the [1][0] element of the transformation.
 * @param t11 the [1][1] element of the transformation.
 * @param t12 the [1][2] element of the transformation.
 * @param t13 the [1][3] element of the transformation.
 * @param t20 the [2][0] element of the transformation.
 * @param t21 the [2][1] element of the transformation.
 * @param t22 the [2][2] element of the transformation.
 * @param t23 the [2][3] element of the transformation.
 * @param t30 the [3][0] element of the transformation.
 * @param t31 the [3][1] element of the transformation.
 * @param t32 the [3][2] element of the transformation.
 * @param t33 the [3][3] element of the transformation.
 */
void Transformation::set(double t00, double t01, double t02, double t03, double t10, double t11, double t12, double t13, double t20, double t21, double t22, double t23, double t30, double t31, double t32, double t33) {
    
    t[ 0] = t00; t[ 1] = t01; t[ 2] = t02; t[ 3] = t03;
    t[ 4] = t10; t[ 5] = t11; t[ 6] = t12; t[ 7] = t13;
    t[ 8] = t20; t[ 9] = t21; t[10] = t22; t[11] = t23;
    t[12] = t30; t[13] = t31; t[14] = t32; t[15] = t33;
}

/**
 * Sets the values of this transformation object to the given linear
 * translation and the Z-Y-X Euler angles.
 * @param x the translation along the X axis, given in [m].
 * @param y the translation along the Y axis, given in [m].
 * @param z the translation along the Z axis, given in [m].
 * @param alpha the angle to rotate about the X axis, given in [rad].
 * @param beta the angle to rotate about the Y axis, given in [rad].
 * @param gamma the angle to rotate about the Z axis, given in [rad].
 */
void Transformation::set(double x, double y, double z, double alpha, double beta, double gamma) {
    
    t[ 0] = cos(gamma)*cos(beta); t[ 1] = -sin(gamma)*cos(alpha)+cos(gamma)*sin(beta)*sin(alpha); t[ 2] = sin(gamma)*sin(alpha)+cos(gamma)*sin(beta)*cos(alpha);  t[ 3] = x;
    t[ 4] = sin(gamma)*cos(beta); t[ 5] = cos(gamma)*cos(alpha)+sin(gamma)*sin(beta)*sin(alpha);  t[ 6] = -cos(gamma)*sin(alpha)+sin(gamma)*sin(beta)*cos(alpha); t[ 7] = y;
    t[ 8] = -sin(beta);           t[ 9] = cos(beta)*sin(alpha);                                   t[10] = cos(beta)*cos(alpha);                                   t[11] = z;
    t[12] = 0.0;                  t[13] = 0.0;                                                    t[14] = 0.0;                                                    t[15] = 1.0;
}

/**
 * Sets the values of this transformation object to the pose defined
 * by the given rotation matrix and translation vector.
 * @param matrix the matrix representing the rotational component.
 * @param vector the translational component of the transformation.
 */
void Transformation::set(Matrix& matrix, Vector& vector) {

    t[ 0] = matrix[0]; t[ 1] = matrix[1]; t[ 2] = matrix[2]; t[ 3] = vector[0];
    t[ 4] = matrix[3]; t[ 5] = matrix[4]; t[ 6] = matrix[5]; t[ 7] = vector[1];
    t[ 8] = matrix[6]; t[ 9] = matrix[7]; t[10] = matrix[8]; t[11] = vector[2];
    t[12] = 0.0;       t[13] = 0.0;       t[14] = 0.0;       t[15] = 1.0;
}

/**
 * Sets the values of this transformation object to the pose defined
 * by the given quaternion and translation vector.
 * @param quaternion the quaternion describing the orientation.
 * @param vector the translational component of the transformation.
 */
void Transformation::set(Quaternion& quaternion, Vector& vector) {
    
    t[0] = 1.0-2.0*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
    t[1] = 2.0*(quaternion[1]*quaternion[2]-quaternion[0]*quaternion[3]);
    t[2] = 2.0*(quaternion[1]*quaternion[3]+quaternion[0]*quaternion[2]);
    t[3] = vector[0];
    
    t[4] = 2.0*(quaternion[1]*quaternion[2]+quaternion[0]*quaternion[3]);
    t[5] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[3]*quaternion[3]);
    t[6] = 2.0*(quaternion[2]*quaternion[3]-quaternion[0]*quaternion[1]);
    t[7] = vector[1];
    
    t[8] = 2.0*(quaternion[1]*quaternion[3]-quaternion[0]*quaternion[2]);
    t[9] = 2.0*(quaternion[2]*quaternion[3]+quaternion[0]*quaternion[1]);
    t[10] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]);
    t[11] = vector[2];
    
    t[12] = 0.0;
    t[13] = 0.0;
    t[14] = 0.0;
    t[15] = 1.0;
}

/**
 * Sets the values of this transformation object to the same values as the given transformation.
 * @param transformation the source transformation.
 */
void Transformation::set(const Transformation& transformation) {

    for (uint16_t i = 0; i < 16; i++) {
        t[i] = transformation.t[i];
    }
}

/**
 * Perform a cross product normalization of the orientation matrix of this transformation.
 */
void Transformation::normalize() {
    
    // calculate y axis as the cross product from the z and x axis
    
    t[1] = t[6]*t[8]-t[10]*t[4];
    t[5] = t[10]*t[0]-t[2]*t[8];
    t[9] = t[2]*t[4]-t[6]*t[0];
    
    // recalculate the x axis as the cross product from the y and z axis
    
    t[0] = t[5]*t[10]-t[9]*t[6];
    t[4] = t[9]*t[2]-t[1]*t[10];
    t[8] = t[1]*t[6]-t[5]*t[2];
    
    // normalize the length of the column vectors
    
    for (uint16_t i = 0; i < 3; i++) {
        
        double length = sqrt(t[i+0]*t[i+0]+t[i+4]*t[i+4]+t[i+8]*t[i+8]);
        
        if (length > 0.0) {
            t[i+0] /= length;
            t[i+4] /= length;
            t[i+8] /= length;
        }
    }
}

/**
 * Inverts this transformation.
 * This means, the translation and rotation described by this transformation is reversed.
 */
void Transformation::invert() {
    
    double n[16];
    
    n[0] = t[0];
    n[1] = t[4];
    n[2] = t[8];
    n[3] = -t[0]*t[3]-t[4]*t[7]-t[8]*t[11];
    
    n[4] = t[1];
    n[5] = t[5];
    n[6] = t[9];
    n[7] = -t[1]*t[3]-t[5]*t[7]-t[9]*t[11];
    
    n[8] = t[2];
    n[9] = t[6];
    n[10] = t[10];
    n[11] = -t[2]*t[3]-t[6]*t[7]-t[10]*t[11];
    
    n[12] = 0.0;
    n[13] = 0.0;
    n[14] = 0.0;
    n[15] = 1.0;
    
    for (uint16_t i = 0; i < 16; i++) {
        t[i] = n[i];
    }
}

/**
 * Sets the value of this transformation to the result of multiplying itself with the given transformation.
 * @param transformation the other transformation to multiply this transformation with.
 */
void Transformation::mul(const Transformation& transformation) {
    
    double n[16];
    
    for (uint16_t i = 0; i < 4; i++) {
        for (uint16_t j = 0; j < 4; j++) {
            double d = 0.0;
            for (uint16_t k = 0; k < 4; k++) {
                d += t[4*i+k]*transformation.t[4*k+j];
            }
            n[4*i+j] = d;
        }
    }
    
    for (uint16_t i = 0; i < 16; i++) {
        t[i] = n[i];
    }
}

/**
 * Sets the value of this transformation to a counter clockwise rotation about the X axis.
 * @param angle the angle to rotate about the X axis, given in [rad].
 */
void Transformation::rotX(double angle) {
    
    double sa = sin(angle);
    double ca = cos(angle);
    
    double n[16];
    
    n[0] = t[0]; n[1] = t[1]*ca+t[2]*sa;  n[2] = -t[1]*sa+t[2]*ca;   n[3] = t[3];
    n[4] = t[4]; n[5] = t[5]*ca+t[6]*sa;  n[6] = -t[5]*sa+t[6]*ca;   n[7] = t[7];
    n[8] = t[8]; n[9] = t[9]*ca+t[10]*sa; n[10] = -t[9]*sa+t[10]*ca; n[11] = t[11];
    n[12] = 0.0; n[13] = 0.0;             n[14] = 0.0;               n[15] = 1.0;
    
    for (uint16_t i = 0; i < 16; i++) {
        t[i] = n[i];
    }
}

/**
 * Sets the value of this transformation to a counter clockwise rotation about the Y axis.
 * @param angle the angle to rotate about the Y axis, given in [rad].
 */
void Transformation::rotY(double angle) {
    
    double sa = sin(angle);
    double ca = cos(angle);
    
    double n[16];
    
    n[0] = t[0]*ca-t[2]*sa;  n[1] = t[1]; n[2] = t[0]*sa+t[2]*ca;   n[3] = t[3];
    n[4] = t[4]*ca-t[6]*sa;  n[5] = t[5]; n[6] = t[4]*sa+t[6]*ca;   n[7] = t[7];
    n[8] = t[8]*ca-t[10]*sa; n[9] = t[9]; n[10] = t[8]*sa+t[10]*ca; n[11] = t[11];
    n[12] = 0.0;             n[13] = 0.0; n[14] = 0.0;              n[15] = 1.0;
    
    for (uint16_t i = 0; i < 16; i++) {
        t[i] = n[i];
    }
}

/**
 * Sets the value of this transformation to a counter clockwise rotation about the Z axis.
 * @param angle the angle to rotate about the Z axis, given in [rad].
 */
void Transformation::rotZ(double angle) {
    
    double sa = sin(angle);
    double ca = cos(angle);
    
    double n[16];
    
    n[0] = t[0]*ca+t[1]*sa; n[1] = -t[0]*sa+t[1]*ca; n[2] = t[2];   n[3] = t[3];
    n[4] = t[4]*ca+t[5]*sa; n[5] = -t[4]*sa+t[5]*ca; n[6] = t[6];   n[7] = t[7];
    n[8] = t[8]*ca+t[9]*sa; n[9] = -t[8]*sa+t[9]*ca; n[10] = t[10]; n[11] = t[11];
    n[12] = 0.0;            n[13] = 0.0;             n[14] = 0.0;   n[15] = 1.0;
    
    for (uint16_t i = 0; i < 16; i++) {
        t[i] = n[i];
    }
}

/**
 * Translates the value of this transformation along the local X axis.
 * @param x the distance to translate, given in [m].
 */
void Transformation::transX(double x) {
    
    t[3] += t[0]*x;
    t[7] += t[4]*x;
    t[11] += t[8]*x;
}

/**
 * Translates the value of this transformation along the local Y axis.
 * @param y the distance to translate, given in [m].
 */
void Transformation::transY(double y) {
    
    t[3] += t[1]*y;
    t[7] += t[5]*y;
    t[11] += t[9]*y;
}

/**
 * Translates the value of this transformation along the local Z axis.
 * @param z the distance to translate, given in [m].
 */
void Transformation::transZ(double z) {
    
    t[3] += t[2]*z;
    t[7] += t[6]*z;
    t[11] += t[10]*z;
}

/**
 * Gets the rotation matrix and translation vector of this transformation.
 * @param matrix the matrix representing the rotational component.
 * @param vector the translational component of the transformation.
 */
void Transformation::get(Matrix& matrix, Vector& vector) {
    
    matrix[0] = t[0]; matrix[1] = t[1]; matrix[2] = t[2];
    matrix[3] = t[4]; matrix[4] = t[5]; matrix[5] = t[6];
    matrix[6] = t[8]; matrix[7] = t[9]; matrix[8] = t[10];
    
    vector[0] = t[3];
    vector[1] = t[7];
    vector[2] = t[11];
}

/**
 * Gets the rotation matrix of this transformation.
 * @param matrix the matrix representing the rotational component.
 */
void Transformation::getMatrix(Matrix& matrix) {
    
    matrix[0] = t[0]; matrix[1] = t[1]; matrix[2] = t[2];
    matrix[3] = t[4]; matrix[4] = t[5]; matrix[5] = t[6];
    matrix[6] = t[8]; matrix[7] = t[9]; matrix[8] = t[10];
}

/**
 * Gets the translation vector of this transformation.
 * @param vector the translational component of the transformation.
 */
void Transformation::getVector(Vector& vector) {
    
    vector[0] = t[3];
    vector[1] = t[7];
    vector[2] = t[11];
}

/**
 * Gets the X-Y-Z Euler angles that define the orientation of this transformation.
 * @param alpha a reference to the angle to rotate about the X axis, given in [rad].
 * @param beta a reference to the angle to rotate about the Y axis, given in [rad].
 * @param gamma a reference to the angle to rotate about the Z axis, given in [rad].
 */
void Transformation::getXYZEulerAngles(double& alpha, double& beta, double& gamma) {
    
    beta = asin(t[2]);                  // returns an angle between -PI/2 and +PI/2
    if (cos(beta) > 1.0e-9) {
        alpha = atan2(-t[6], t[10]);    // returns an angle between -PI and +PI
        gamma = atan2(-t[1], t[0]);     // returns an angle between -PI and +PI
    } else {                            // X axis is vertical, therefore alpha and gamma cannot be distinguished
        alpha = 0.0;                    // this angle is defined to be zero
        gamma = atan2(-t[4], t[5]);     // returns an angle between -PI and +PI
    }
}

/**
 * Gets the Z-Y-X Euler angles that define the orientation of this transformation.
 * @param alpha a reference to the angle to rotate about the X axis, given in [rad].
 * @param beta a reference to the angle to rotate about the Y axis, given in [rad].
 * @param gamma a reference to the angle to rotate about the Z axis, given in [rad].
 */
void Transformation::getZYXEulerAngles(double& alpha, double& beta, double& gamma) {
    
    beta = asin(-t[8]);                 // returns an angle between -PI/2 and +PI/2
    if (cos(beta) > 1.0e-9) {
        alpha = atan2(t[9], t[10]);     // returns an angle between -PI and +PI
        gamma = atan2(t[4], t[0]);      // returns an angle between -PI and +PI
    } else {                            // X axis is vertical, therefore alpha and gamma cannot be distinguished
        alpha = 0.0;                    // this angle is defined to be zero
        gamma = atan2(-t[1], t[5]);     // returns an angle between -PI and +PI
    }
}

/**
 * Gets a quaternion representing the orientation of this transformation.
 * @param a reference to a quaternion to set to the orientation of this transformation.
 */
void Transformation::getQuaternion(Quaternion& quaternion) {
    
    double tr = t[0]+t[5]+t[10];
    if (tr > 0.0) {
        double s = sqrt(tr+1.0)*2.0; // s = 4*qw
        quaternion[0] = 0.25*s;
        quaternion[1] = (t[9]-t[6])/s;
        quaternion[2] = (t[2]-t[8])/s;
        quaternion[3] = (t[4]-t[1])/s;
    } else if ((t[0] > t[5]) && (t[0] > t[10])) {
        double s = sqrt(1.0+t[0]-t[5]-t[10])*2.0; // s = 4*qx
        quaternion[0] = (t[9]-t[6])/s;
        quaternion[1] = 0.25*s;
        quaternion[2] = (t[1]+t[4])/s;
        quaternion[3] = (t[2]+t[8])/s;
    } else if (t[5] > t[10]) {
        double s = sqrt(1.0+t[5]-t[0]-t[10])*2.0; // s = 4*qy
        quaternion[0] = (t[2]-t[8])/s;
        quaternion[1] = (t[1]+t[4])/s;
        quaternion[2] = 0.25*s;
        quaternion[3] = (t[6]+t[9])/s;
    } else {
        double s = sqrt(1.0+t[10]-t[0]-t[5])*2.0; // s = 4*qz
        quaternion[0] = (t[4]-t[1])/s;
        quaternion[1] = (t[2]+t[8])/s;
        quaternion[2] = (t[6]+t[9])/s;
        quaternion[3] = 0.25*s;
    }
}

/**
 * Return a string representation of this transformation object.
 * @return a string that contains the components of this transformation.
 */
string Transformation::toString() {
    
    stringstream s;
    
	s << t[0] << " " << t[1] << " " << t[2] << " " << t[3] << endl << t[4] << " " << t[5] << " " << t[6] << " " << t[7] << endl << t[8] << " " << t[9] << " " << t[10] << " " << t[11] << endl << t[12] << " " << t[13] << " " << t[14] << " " << t[15] << endl;
    
	return s.str();
}
