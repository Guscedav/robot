/*
 * Vector.cpp
 * Copyright (c) 2015, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.11.2015
 *      Author: Marcel Honegger
 */

#include <cmath>
#include <stdexcept>
#include <sstream>
#include "Vector.h"

using namespace std;

/**
 * Creates a vector object and initialzes the values to zero.
 */
Vector::Vector() {
    
    for (uint16_t i = 0; i < 3; i++) {
        v[i] = 0.0;
    }
}

/**
 * Creates a vector object and initializes the values to the given three-element array.
 * @param v the array of length 3 containing initial values for the vector.
 */
Vector::Vector(double v[]) {
    
    for (uint16_t i = 0; i < 3; i++) {
        this->v[i] = v[i];
    }
}

/**
 * Creates a vector object and initializes the values to the given three vector values.
 * @param v0 the [0] element of the vector.
 * @param v1 the [1] element of the vector.
 * @param v2 the [2] element of the vector.
 */
Vector::Vector(double v0, double v1, double v2) {
    
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
}

/**
 * Creates a new vector object with the same values as the given vector.
 * @param vector the source vector.
 */
Vector::Vector(const Vector& vector) {
    
    for (uint16_t i = 0; i < 3; i++) {
        v[i] = vector.v[i];
    }
}

/**
 * Deletes the vector object.
 */
Vector::~Vector() {}

/**
 * Access a single element of this vector.
 * This method throws an <code>out_of_range</code> exception if the index is out of range.
 */
double& Vector::operator[] (const uint16_t i) {
    
    if (i >= 3) throw out_of_range("Vector: index number is out of range.");
    
    return v[i];
}

/**
 * Sets the values of this vector object to the same values as the given vector.
 * @param vector the source vector.
 */
void Vector::set(const Vector& vector) {
    
    for (uint16_t i = 0; i < 3; i++) {
        v[i] = vector.v[i];
    }
}

/**
 * Returns the length of this vector.
 * @return the length of this vector.
 */
double Vector::length() {
    
    return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
}

/**
 * Perform a normalization of this vector.
 */
void Vector::normalize() {
    
    double length = this->length();
    
    if (length > 0.0) {
        for (uint16_t i = 0; i < 3; i++) {
            v[i] /= length;
        }
    }
}

/**
 * Sets the value of this vector to the vector sum of itself and the given vector.
 * @param vector the other vector.
 */
void Vector::add(const Vector& vector) {
    
    for (uint16_t i = 0; i < 3; i++) {
        v[i] += vector.v[i];
    }
}

/**
 * Sets the value of this vector to the scalar multiplication of the scale factor with this.
 * @param scalar the scalar value.
 */
void Vector::scale(double scalar) {
    
	for (uint16_t i = 0; i < 3; i++) {
        v[i] *= scalar;
    }
}

/**
 * Sets this vector to the vector cross product of vectors v1 and v2.
 * @param v1 the first vector.
 * @param v2 the second vector.
 */
void Vector::cross(const Vector& v1, const Vector& v2) {
    
    v[0] = v1.v[1]*v2.v[2]-v1.v[2]*v2.v[1];
    v[1] = v1.v[2]*v2.v[0]-v1.v[0]*v2.v[2];
    v[2] = v1.v[0]*v2.v[1]-v1.v[1]*v2.v[0];
}

/**
 * Return a string representation of this vector object.
 * @return a string that contains the components of this vector. 
 */
string Vector::toString() {
    
	stringstream s;
    
	s << v[0] << " " << v[1] << " " << v[2] << endl;
    
	return s.str();
}
