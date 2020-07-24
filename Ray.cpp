#include "Ray.h"
Ray::Ray() {
    origin = {0, 0, 0};
    direction = {0, 0, 0};
}

Ray::Ray(const Vector3f& origin, const Vector3f& direction)
    : origin(origin), direction(direction) {}

/* Takes a parameter t and returns the point accoring to t. t is the parametric
variable in the ray equation o+td.*/
Vector3f Ray::getPoint(float t) const {
    return MathUtil::addVectors(origin,
                                MathUtil::scalarMultVector(t, direction));
}

/* Takes a point p and returns the parameter t according to p such that p =
 * o+t*d. */
float Ray::gett(const Vector3f& p) const {
    Vector3f differenceVector = MathUtil::subVectors(p, origin);
    return differenceVector.x / direction.x;
}