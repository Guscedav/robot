/*
 * Quaternion.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 16.11.2015
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <stdexcept>
#include <sstream>
#include "Quaternion.h"

using namespace std;

/**
 * Creates a quaternion object and initializes the values.
 */
Quaternion::Quaternion() {
    
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
}

/**
 * Creates a quaternion object and initializes the values to the given four-element array.
 * @param q the array of length 4 containing initial values for the quaternion.
 */
Quaternion::Quaternion(double q[]) {
    
    for (uint16_t i = 0; i < 4; i++) {
        this->q[i] = q[i];
    }
}

/**
 * Creates a quaternion object and initializes the values to the given four values.
 * @param q0 the real element of the quaternion.
 * @param q1 the first imaginary element of the quaternion.
 * @param q2 the second imaginary element of the quaternion.
 * @param q3 the third imaginary element of the quaternion.
 */
Quaternion::Quaternion(double q0, double q1, double q2, double q3) {
    
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**
 * Creates a new quaternion object with the same values as the given quaternion.
 * @param quaternion the source quaternion.
 */
Quaternion::Quaternion(const Quaternion& quaternion) {
    
    for (uint16_t i = 0; i < 4; i++) {
        q[i] = quaternion.q[i];
    }
}

/**
 * Deletes the quaternion object.
 */
Quaternion::~Quaternion() {}

/**
 * Access a single element of this quaternion.
 * This method throws an <code>out_of_range</code> exception if the index is out of range.
 */
double& Quaternion::operator[] (const uint16_t i) {
    
    if (i >= 4) throw out_of_range("Quaternion: index number is out of range.");
    
    return q[i];
}

/**
 * Sets the values of this quaternion object to the values of the given four-element array.
 * @param q the array of length 4 containing the values for the quaternion.
 */
void Quaternion::set(double q[]) {
    
    for (uint16_t i = 0; i < 4; i++) {
        this->q[i] = q[i];
    }
}

/**
 * Sets the values of this quaternion object to the given four values.
 * @param q0 the real element of the quaternion.
 * @param q1 the first imaginary element of the quaternion.
 * @param q2 the second imaginary element of the quaternion.
 * @param q3 the third imaginary element of the quaternion.
 */
void Quaternion::set(double q0, double q1, double q2, double q3) {
    
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**
 * Sets the values of this quaternion object to the same values as the given quaternion.
 * @param quaternion the source quaternion.
 */
void Quaternion::set(const Quaternion& quaternion) {
    
    for (uint16_t i = 0; i < 4; i++) {
        q[i] = quaternion.q[i];
    }
}

/**
 * Perform a normalization of this quaternion.
 */
void Quaternion::normalize() {
    
    double length = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    
    if (length > 0.0) {
        for (uint16_t i = 0; i < 4; i++) {
            q[i] /= length;
        }
    }
}

/**
 * Return a string representation of this quaternion object.
 * @return a string that contains the components of this quaternion. 
 */
string Quaternion::toString() {
    
	stringstream s;
    
	s << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    
	return s.str();
}
