/*
 * Matrix.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.11.2015
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <stdexcept>
#include <sstream>
#include "Quaternion.h"
#include "Vector.h"
#include "Matrix.h"

using namespace std;

/**
 * Creates a matrix object and initializes the values to a unit matrix.
 */
Matrix::Matrix() {
    
    m[0] = 1.0; m[1] = 0.0; m[2] = 0.0;
    m[3] = 0.0; m[4] = 1.0; m[5] = 0.0;
    m[6] = 0.0; m[7] = 0.0; m[8] = 1.0;
}

/**
 * Creates a matrix object and initializes the values to the given nine-element array.
 * @param m the array of length 9 containing initial values for the matrix.
 */
Matrix::Matrix(double m[]) {
    
    for (uint16_t i = 0; i < 9; i++) {
        this->m[i] = m[i];
    }
}

/**
 * Creates a matrix object and initializes the values to the given nine matrix values.
 * @param m00 the [0][0] element of the matrix.
 * @param m01 the [0][1] element of the matrix.
 * @param m02 the [0][2] element of the matrix.
 * @param m10 the [1][0] element of the matrix.
 * @param m11 the [1][1] element of the matrix.
 * @param m12 the [1][2] element of the matrix.
 * @param m20 the [2][0] element of the matrix.
 * @param m21 the [2][1] element of the matrix.
 * @param m22 the [2][2] element of the matrix.
 */
Matrix::Matrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22) {
    
    m[0] = m00; m[1] = m01; m[2] = m02;
    m[3] = m10; m[4] = m11; m[5] = m12;
    m[6] = m20; m[7] = m21; m[8] = m22;
}

/**
 * Creates a matrix object and initializes the values based on the given Z-Y-X Euler angles.
 * @param alpha the angle to rotate about the X axis, given in [rad].
 * @param beta the angle to rotate about the Y axis, given in [rad].
 * @param gamma the angle to rotate about the Z axis, given in [rad].
 */
Matrix::Matrix(double alpha, double beta, double gamma) {
    
    m[0] = cos(gamma)*cos(beta); m[1] = -sin(gamma)*cos(alpha)+cos(gamma)*sin(beta)*sin(alpha); m[2] = sin(gamma)*sin(alpha)+cos(gamma)*sin(beta)*cos(alpha);
    m[3] = sin(gamma)*cos(beta); m[4] = cos(gamma)*cos(alpha)+sin(gamma)*sin(beta)*sin(alpha);  m[5] = -cos(gamma)*sin(alpha)+sin(gamma)*sin(beta)*cos(alpha);
    m[6] = -sin(beta);           m[7] = cos(beta)*sin(alpha);                                   m[8] = cos(beta)*cos(alpha);
}

/**
 * Creates a matrix object and initializes the values based on the given quaternion object.
 * @param quaternion the quaternion describing the orientation.
 */
Matrix::Matrix(Quaternion& quaternion) {
    
    m[0] = 1.0-2.0*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
    m[1] = 2.0*(quaternion[1]*quaternion[2]-quaternion[0]*quaternion[3]);
    m[2] = 2.0*(quaternion[1]*quaternion[3]+quaternion[0]*quaternion[2]);
    
    m[3] = 2.0*(quaternion[1]*quaternion[2]+quaternion[0]*quaternion[3]);
    m[4] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[3]*quaternion[3]);
    m[5] = 2.0*(quaternion[2]*quaternion[3]-quaternion[0]*quaternion[1]);
    
    m[6] = 2.0*(quaternion[1]*quaternion[3]-quaternion[0]*quaternion[2]);
    m[7] = 2.0*(quaternion[2]*quaternion[3]+quaternion[0]*quaternion[1]);
    m[8] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]);
}

/**
 * Creates a new matrix object with the same values as the given matrix.
 * @param matrix the source matrix.
 */
Matrix::Matrix(const Matrix& matrix) {
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] = matrix.m[i];
    }
}

/**
 * Deletes the matrix object.
 */
Matrix::~Matrix() {}

/**
 * Access a single element of this matrix.
 * This method throws an <code>out_of_range</code> exception if the index is out of range.
 */
double& Matrix::operator[] (const uint16_t i) {
    
    if (i >= 9) throw out_of_range("Matrix: index number is out of range.");
    
    return m[i];
}

/**
 * Sets the values of this matrix object to the given nine matrix values.
 * @param m00 the [0][0] element of the matrix.
 * @param m01 the [0][1] element of the matrix.
 * @param m02 the [0][2] element of the matrix.
 * @param m10 the [1][0] element of the matrix.
 * @param m11 the [1][1] element of the matrix.
 * @param m12 the [1][2] element of the matrix.
 * @param m20 the [2][0] element of the matrix.
 * @param m21 the [2][1] element of the matrix.
 * @param m22 the [2][2] element of the matrix.
 */
void Matrix::set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22) {
    
    m[0] = m00; m[1] = m01; m[2] = m02;
    m[3] = m10; m[4] = m11; m[5] = m12;
    m[6] = m20; m[7] = m21; m[8] = m22;
}

/**
 * Sets the values of this matrix object based on the given Z-Y-X Euler angles.
 * @param alpha the angle to rotate about the X axis, given in [rad].
 * @param beta the angle to rotate about the Y axis, given in [rad].
 * @param gamma the angle to rotate about the Z axis, given in [rad].
 */
void Matrix::set(double alpha, double beta, double gamma) {
    
    m[0] = cos(gamma)*cos(beta); m[1] = -sin(gamma)*cos(alpha)+cos(gamma)*sin(beta)*sin(alpha); m[2] = sin(gamma)*sin(alpha)+cos(gamma)*sin(beta)*cos(alpha);
    m[3] = sin(gamma)*cos(beta); m[4] = cos(gamma)*cos(alpha)+sin(gamma)*sin(beta)*sin(alpha);  m[5] = -cos(gamma)*sin(alpha)+sin(gamma)*sin(beta)*cos(alpha);
    m[6] = -sin(beta);           m[7] = cos(beta)*sin(alpha);                                   m[8] = cos(beta)*cos(alpha);
}

/**
 * Sets the values of this matrix object based on the given quaternion object.
 * @param quaternion the quaternion describing the orientation.
 */
void Matrix::set(Quaternion& quaternion) {

    m[0] = 1.0-2.0*(quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
    m[1] = 2.0*(quaternion[1]*quaternion[2]-quaternion[0]*quaternion[3]);
    m[2] = 2.0*(quaternion[1]*quaternion[3]+quaternion[0]*quaternion[2]);

    m[3] = 2.0*(quaternion[1]*quaternion[2]+quaternion[0]*quaternion[3]);
    m[4] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[3]*quaternion[3]);
    m[5] = 2.0*(quaternion[2]*quaternion[3]-quaternion[0]*quaternion[1]);

    m[6] = 2.0*(quaternion[1]*quaternion[3]-quaternion[0]*quaternion[2]);
    m[7] = 2.0*(quaternion[2]*quaternion[3]+quaternion[0]*quaternion[1]);
    m[8] = 1.0-2.0*(quaternion[1]*quaternion[1]+quaternion[2]*quaternion[2]);
}

/**
 * Sets the values of this matrix object to the same values as the given matrix.
 * @param matrix the source matrix.
 */
void Matrix::set(const Matrix& matrix) {

    for (uint16_t i = 0; i < 9; i++) {
        m[i] = matrix.m[i];
    }
}

/**
 * Perform a cross product normalization of this matrix.
 */
void Matrix::normalize() {
    
    // calculate y axis as the cross product from the z and x axis
    
    m[1] = m[5]*m[6]-m[8]*m[3];
    m[4] = m[8]*m[0]-m[2]*m[6];
    m[7] = m[2]*m[3]-m[5]*m[0];
    
    // recalculate the x axis as the cross product from the y and z axis
    
    m[0] = m[4]*m[8]-m[7]*m[5];
    m[3] = m[7]*m[2]-m[1]*m[8];
    m[6] = m[1]*m[5]-m[4]*m[2];
    
    // normalize the length of the column vectors
    
    for (uint16_t i = 0; i < 3; i++) {
        
        double length = sqrt(m[i+0]*m[i+0]+m[i+3]*m[i+3]+m[i+6]*m[i+6]);
        
        if (length > 1.0e-9) {
            m[i+0] /= length;
            m[i+3] /= length;
            m[i+6] /= length;
        }
    }
}

/**
 * Calculates the determinant of this matrix.
 * @return the determinant of this matrix.
 */
double Matrix::determinant() {
    
    return m[0]*m[4]*m[8]-m[6]*m[4]*m[2]+m[3]*m[7]*m[2]-m[0]*m[7]*m[5]+m[6]*m[1]*m[5]-m[3]*m[1]*m[8];
}

/**
 * Inverts this matrix.
 * Note that the inverse of an orthogonal matrix is equal to its transpose, and
 * calculating the transpose requires significantly less computing power.
 */
void Matrix::invert() {
    
    double determinant = this->determinant();
    
    if (fabs(determinant) > 1.0e-9) {
        
        double n[9];
        
        n[0] = m[4]*m[8]-m[7]*m[5]; n[1] = m[2]*m[7]-m[8]*m[1]; n[2] = m[1]*m[5]-m[4]*m[2];
        n[3] = m[5]*m[6]-m[8]*m[3]; n[4] = m[0]*m[8]-m[6]*m[2]; n[5] = m[2]*m[3]-m[5]*m[0];
        n[6] = m[3]*m[7]-m[6]*m[4]; n[7] = m[1]*m[6]-m[7]*m[0]; n[8] = m[0]*m[4]-m[3]*m[1];
        
        for (uint16_t i = 0; i < 9; i++) {
            m[i] = n[i]/determinant;
        }
    }
}

/**
 * Sets the value of this matrix to its transpose.
 */
void Matrix::transpose() {

    double d = 0.0;
    
    d = m[1]; m[1] = m[3]; m[3] = d;
    d = m[2]; m[2] = m[6]; m[6] = d;
    d = m[5]; m[5] = m[7]; m[7] = d;
}

/**
 * Adds a scalar to each component of this matrix.
 * @param scalar the scalar adder.
 */
void Matrix::add(double scalar) {
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] += scalar;
    }
}

/**
 * Sets the value of this matrix to the matrix sum of itself and the given matrix.
 * @param matrix the other matrix to add.
 */
void Matrix::add(const Matrix& matrix) {
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] += matrix.m[i];
    }
}

/**
 * Multiplies each element of this matrix by a scalar.
 * @param scalar the scalar multiplier.
 */
void Matrix::mul(double scalar) {
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] *= scalar;
    }
}

/**
 * Sets the value of this matrix to the result of multiplying itself with the given matrix.
 * @param matrix the other matrix to multiply this matrix with.
 */
void Matrix::mul(const Matrix& matrix) {
    
    double n[9];
    
    for (uint16_t i = 0; i < 3; i++) {
        for (uint16_t j = 0; j < 3; j++) {
            double d = 0.0;
            for (uint16_t k = 0; k < 3; k++) {
                d += m[3*i+k]*matrix.m[3*k+j];
            }
            n[3*i+j] = d;
        }
    }
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] = n[i];
    }
}

/**
 * Calculates the multiplication of this matrix with a given vector, and copies the
 * result into a given vector.
 * @param vector the vector to multiply this matrix with.
 * @param result the vector getting the result of the multiplication.
 */
void Matrix::mul(Vector& vector, Vector& result) {
    
    for (uint16_t i = 0; i < 3; i++) {
        result[i] = m[3*i+0]*vector[0]+m[3*i+1]*vector[1]+m[3*i+2]*vector[2];
    }
}

/**
 * Sets the value of this matrix to a counter clockwise rotation about the X axis.
 * @param angle the angle to rotate about the X axis, given in [rad].
 */
void Matrix::rotX(double angle) {
    
    double n[9];
    
    n[0] = m[0]; n[1] = m[1]*cos(angle)+m[2]*sin(angle); n[2] = -m[1]*sin(angle)+m[2]*cos(angle);
    n[3] = m[3]; n[4] = m[4]*cos(angle)+m[5]*sin(angle); n[5] = -m[4]*sin(angle)+m[5]*cos(angle);
    n[6] = m[6]; n[7] = m[7]*cos(angle)+m[8]*sin(angle); n[8] = -m[7]*sin(angle)+m[8]*cos(angle);
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] = n[i];
    }
}

/**
 * Sets the value of this matrix to a counter clockwise rotation about the Y axis.
 * @param angle the angle to rotate about the Y axis, given in [rad].
 */
void Matrix::rotY(double angle) {
    
    double n[9];
    
    n[0] = m[0]*cos(angle)-m[2]*sin(angle); n[1] = m[1]; n[2] = m[0]*sin(angle)+m[2]*cos(angle);
    n[3] = m[3]*cos(angle)-m[5]*sin(angle); n[4] = m[4]; n[5] = m[3]*sin(angle)+m[5]*cos(angle);
    n[6] = m[6]*cos(angle)-m[8]*sin(angle); n[7] = m[7]; n[8] = m[6]*sin(angle)+m[8]*cos(angle);
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] = n[i];
    }
}

/**
 * Sets the value of this matrix to a counter clockwise rotation about the Z axis.
 * @param angle the angle to rotate about the Z axis, given in [rad].
 */
void Matrix::rotZ(double angle) {
    
    double n[9];
    
    n[0] = m[0]*cos(angle)+m[1]*sin(angle); n[1] = -m[0]*sin(angle)+m[1]*cos(angle); n[2] = m[2];
    n[3] = m[3]*cos(angle)+m[4]*sin(angle); n[4] = -m[3]*sin(angle)+m[4]*cos(angle); n[5] = m[5];
    n[6] = m[6]*cos(angle)+m[7]*sin(angle); n[7] = -m[6]*sin(angle)+m[7]*cos(angle); n[8] = m[8];
    
    for (uint16_t i = 0; i < 9; i++) {
        m[i] = n[i];
    }
}

/**
 * Gets the X-Y-Z Euler angles that define this orientation.
 * @param alpha a reference to the angle to rotate about the X axis, given in [rad].
 * @param beta a reference to the angle to rotate about the Y axis, given in [rad].
 * @param gamma a reference to the angle to rotate about the Z axis, given in [rad].
 */
void Matrix::getXYZEulerAngles(double& alpha, double& beta, double& gamma) {
    
    beta = asin(m[2]);                  // returns an angle between -PI/2 and +PI/2
    if (cos(beta) > 1.0e-9) {
        alpha = atan2(-m[5], m[8]);     // returns an angle between -PI and +PI
        gamma = atan2(-m[1], m[0]);     // returns an angle between -PI and +PI
    } else {                            // X axis is vertical, therefore alpha and gamma cannot be distinguished
        alpha = 0.0;                    // this angle is defined to be zero
        gamma = atan2(-m[3], m[4]);     // returns an angle between -PI and +PI
    }
}

/**
 * Gets the Z-Y-X Euler angles that define this orientation.
 * @param alpha a reference to the angle to rotate about the X axis, given in [rad].
 * @param beta a reference to the angle to rotate about the Y axis, given in [rad].
 * @param gamma a reference to the angle to rotate about the Z axis, given in [rad].
 */
void Matrix::getZYXEulerAngles(double& alpha, double& beta, double& gamma) {
    
    beta = asin(-m[6]);                 // returns an angle between -PI/2 and +PI/2
    if (cos(beta) > 1.0e-9) {
        alpha = atan2(m[7], m[8]);      // returns an angle between -PI and +PI
        gamma = atan2(m[3], m[0]);      // returns an angle between -PI and +PI
    } else {                            // X axis is vertical, therefore alpha and gamma cannot be distinguished
        alpha = 0.0;                    // this angle is defined to be zero
        gamma = atan2(-m[1], m[4]);     // returns an angle between -PI and +PI
    }
}

/**
 * Gets a quaternion representing the orientation of this matrix.
 * @param a reference to a quaternion to set to the orientation of this matrix.
 */
void Matrix::getQuaternion(Quaternion& quaternion) {
    
    double tr = m[0]+m[4]+m[8];
    if (tr > 0.0) {
        double s = sqrt(tr+1.0)*2.0; // s = 4*qw
        quaternion[0] = 0.25*s;
        quaternion[1] = (m[7]-m[5])/s;
        quaternion[2] = (m[2]-m[6])/s;
        quaternion[3] = (m[3]-m[1])/s;
    } else if ((m[0] > m[4]) && (m[0] > m[8])) {
        double s = sqrt(1.0+m[0]-m[4]-m[8])*2.0; // s = 4*qx
        quaternion[0] = (m[7]-m[5])/s;
        quaternion[1] = 0.25*s;
        quaternion[2] = (m[1]+m[3])/s;
        quaternion[3] = (m[2]+m[6])/s;
    } else if (m[4] > m[8]) {
        double s = sqrt(1.0+m[4]-m[0]-m[8])*2.0; // s = 4*qy
        quaternion[0] = (m[2]-m[6])/s;
        quaternion[1] = (m[1]+m[3])/s;
        quaternion[2] = 0.25*s;
        quaternion[3] = (m[5]+m[7])/s;
    } else {
        double s = sqrt(1.0+m[8]-m[0]-m[4])*2.0; // s = 4*qz
        quaternion[0] = (m[3]-m[1])/s;
        quaternion[1] = (m[2]+m[6])/s;
        quaternion[2] = (m[5]+m[7])/s;
        quaternion[3] = 0.25*s;
    }
}

/**
 * Return a string representation of this matrix object.
 * @return a string that contains the components of this matrix.
 */
string Matrix::toString() {
    
	stringstream s;
    
	s << m[0] << " " << m[1] << " " << m[2] << endl << m[3] << " " << m[4] << " " << m[5] << endl << m[6] << " " << m[7] << " " << m[8] << endl;
    
	return s.str();
}
