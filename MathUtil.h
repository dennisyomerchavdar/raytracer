#ifndef _MATHUTIL_H_
#define _MATHUTIL_H_
#include <vector>
#include "defs.h"

class MathUtil {
   public:
    Vector3f static addVectors(Vector3f v1, Vector3f v2);

    Vector3f static subVectors(Vector3f v1, Vector3f v2);

    Vector3f static scalarMultVector(float t, Vector3f v);

    Vector3f static crossProdVectors(Vector3f v1, Vector3f v2);

    float static dotProdVectors(Vector3f v1, Vector3f v2);
    float static sqDistancePoints(Vector3f p1, Vector3f p2);

    Vector3f static normalize(Vector3f v);
};

#endif