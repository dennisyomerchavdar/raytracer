#include "MathUtil.h"
#include <cmath>
Vector3f MathUtil::addVectors(Vector3f v1, Vector3f v2) {
    Vector3f result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
    return result;
}

Vector3f MathUtil::subVectors(Vector3f v1, Vector3f v2) {
    Vector3f result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    result.z = v1.z - v2.z;
    return result;
}

Vector3f MathUtil::scalarMultVector(float t, Vector3f v) {
    Vector3f result;
    result.x = v.x * t;
    result.y = v.y * t;
    result.z = v.z * t;
    return result;
}

Vector3f MathUtil::crossProdVectors(Vector3f v1, Vector3f v2) {
    Vector3f result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

float MathUtil::dotProdVectors(Vector3f v1, Vector3f v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float MathUtil::sqDistancePoints(Vector3f p1, Vector3f p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
}

Vector3f MathUtil::normalize(Vector3f v) {
    float norm = sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
    return {v.x / norm, v.y / norm, v.z / norm};
}