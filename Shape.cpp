#include "Shape.h"
#include <cstdio>
#include "Scene.h"
Shape::Shape(void) : id(0), matIndex(0) {}

Shape::Shape(int id, int matIndex) : id(id), matIndex(matIndex) {}

Sphere::Sphere(void) : Shape(), cIndex(0), radius(0), vertices() {}

/* Constructor for sphere. You will implement this. */
Sphere::Sphere(int id,
               int matIndex,
               int cIndex,
               float R,
               vector<Vector3f>* vertices)
    : Shape(id, matIndex), cIndex(cIndex), radius(R), vertices(vertices) {}

/* Sphere-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the
intersection point, e.g., coordinate of that point, normal at that point etc.
You should to declare the variables in ReturnVal structure you think you will
need. It is in defs.h file. */
ReturnVal Sphere::intersect(const Ray& ray) const {
    ReturnVal res;
    res.valid = false;
    Vector3f center = pScene->vertices[cIndex - 1];
    Vector3f eminusc = MathUtil::subVectors(ray.origin, center);
    Vector3f d = ray.direction;

    float delta =
        pow(MathUtil::dotProdVectors(d, eminusc), 2.0) -
        MathUtil::dotProdVectors(d, d) *
            (MathUtil::dotProdVectors(eminusc, eminusc) - pow(radius, 2.0));
    if (delta < 0) {
        return res;
    }

    float t1 =
        (MathUtil::dotProdVectors(MathUtil::scalarMultVector(-1, d), eminusc) -
         sqrt(delta)) /
        MathUtil::dotProdVectors(d, d);

    float t2 =
        (MathUtil::dotProdVectors(MathUtil::scalarMultVector(-1, d), eminusc) +
         sqrt(delta)) /
        MathUtil::dotProdVectors(d, d);

    Vector3f lowTPoint = MathUtil::addVectors(
        ray.origin, MathUtil::scalarMultVector(t1, ray.direction));

    Vector3f highTPoint = MathUtil::addVectors(
        ray.origin, MathUtil::scalarMultVector(t2, ray.direction));

    if (ray.gett(lowTPoint) >= 0) {
        res.point = lowTPoint;
        res.normal =
            MathUtil::normalize(MathUtil::subVectors(res.point, center));
        res.matID = matIndex;
        res.valid = true;
        return res;
    }

    else if (ray.gett(highTPoint) >= 0 &&
             sqrtf(MathUtil::sqDistancePoints(center, ray.origin)) < radius) {
        res.point = highTPoint;
        res.normal =
            MathUtil::normalize(MathUtil::subVectors(res.point, center));
        res.matID = matIndex;
        res.valid = true;
    } else {
        return res;
    }
}

Triangle::Triangle(void)
    : Shape(), p1Index(0), p2Index(0), p3Index(0), vertices() {}

/* Constructor for triangle. You will implement this. */
Triangle::Triangle(int id,
                   int matIndex,
                   int p1Index,
                   int p2Index,
                   int p3Index,
                   vector<Vector3f>* vertices)
    : Shape(id, matIndex),
      p1Index(p1Index),
      p2Index(p2Index),
      p3Index(p3Index),
      vertices(vertices) {}

/* Triangle-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the
intersection point, e.g., coordinate of that point, normal at that point
etc. You should to declare the variables in ReturnVal structure you think
you will need. It is in defs.h file. */
ReturnVal Triangle::intersect(const Ray& ray) const {
    ReturnVal res;
    res.valid = false;
    Vector3f p1 = pScene->vertices[p1Index - 1];
    Vector3f p2 = pScene->vertices[p2Index - 1];
    Vector3f p3 = pScene->vertices[p3Index - 1];
    float a = p1.x - p2.x;
    float b = p1.y - p2.y;
    float c = p1.z - p2.z;
    float d = p1.x - p3.x;
    float e = p1.y - p3.y;
    float f = p1.z - p3.z;
    float g = ray.direction.x;
    float h = ray.direction.y;
    float i = ray.direction.z;
    float j = p1.x - ray.origin.x;
    float k = p1.y - ray.origin.y;
    float l = p1.z - ray.origin.z;

    float determinant =
        a * (e * i - h * f) + b * (g * f - d * i) + c * (d * h - e * g);
    if (determinant == 0) {
        return res;
    }

    else {
        float beta =
            (j * (e * i - h * f) + k * (g * f - d * i) + l * (d * h - e * g)) /
            determinant;
        float gamma =
            (i * (a * k - j * b) + h * (j * c - a * l) + g * (b * l - k * c)) /
            determinant;
        float t =
            -(f * (a * k - j * b) + e * (j * c - a * l) + d * (b * l - k * c)) /
            determinant;

        res.point = MathUtil::addVectors(
            ray.origin, MathUtil::scalarMultVector(t, ray.direction));

        if (beta >= 0 && gamma >= 0 && beta + gamma <= 1 &&
            ray.gett(res.point) >= 0) {
            res.valid = true;
            res.normal = MathUtil::normalize(MathUtil::crossProdVectors(
                MathUtil::subVectors(p1, p2), MathUtil::subVectors(p2, p3)));
            res.matID = matIndex;
            return res;

        }

        else {
            return res;
        }
    }
}

Mesh::Mesh() : Shape(), faces(), pIndices(0), vertices() {}

/* Constructor for mesh. You will implement this. */
Mesh::Mesh(int id,
           int matIndex,
           const vector<Triangle>& faces,
           vector<int>* pIndices,
           vector<Vector3f>* vertices)
    : Shape(id, matIndex),
      faces(faces),
      pIndices(pIndices),
      vertices(vertices) {
    xmin = INFINITY;
    ymin = INFINITY;
    zmin = INFINITY;
    xmax = -INFINITY;
    ymax = -INFINITY;
    zmax = -INFINITY;
    for (int i : *pIndices) {
        xmax = (*vertices)[i - 1].x > xmax ? (*vertices)[i - 1].x : xmax;
        ymax = (*vertices)[i - 1].y > ymax ? (*vertices)[i - 1].y : ymax;
        zmax = (*vertices)[i - 1].z > zmax ? (*vertices)[i - 1].z : zmax;
        xmin = (*vertices)[i - 1].x < xmin ? (*vertices)[i - 1].x : xmin;
        ymin = (*vertices)[i - 1].y < ymin ? (*vertices)[i - 1].y : ymin;
        zmin = (*vertices)[i - 1].z < zmin ? (*vertices)[i - 1].z : zmin;
    }
}

/* Mesh-ray intersection routine. You will implement this.
Note that ReturnVal structure should hold the information related to the
intersection point, e.g., coordinate of that point, normal at that point
etc. You should to declare the variables in ReturnVal structure you think
you will need. It is in defs.h file. */
ReturnVal Mesh::intersect(const Ray& ray) const {
    ReturnVal res;
    res.valid = false;
    float lowestt = INFINITY;
    if (!boundingBoxHit(ray)) {
        return res;
    }
    for (Triangle tri : faces) {
        ReturnVal candidate = tri.intersect(ray);
        if (candidate.valid) {
            float tForCandidate = ray.gett(candidate.point);

            if (tForCandidate < lowestt) {
                lowestt = tForCandidate;
                res = candidate;
            }
        }
        res.matID = matIndex;
    }

    return res;
}
bool Mesh::boundingBoxHit(Ray ray) const {
    Vector3f p1;
    Vector3f p2;
    Vector3f p3;

    p1 = {xmax, ymin, zmin};
    p2 = {xmin, ymin, zmin};
    p3 = {xmin, ymin, zmax};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    p1 = {xmax, ymin, zmin};
    p2 = {xmax, ymin, zmax};
    p3 = {xmin, ymin, zmax};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    p1 = {xmin, ymax, zmax};
    p2 = {xmax, ymax, zmax};
    p3 = {xmax, ymax, zmin};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    p1 = {xmin, ymax, zmax};
    p2 = {xmin, ymax, zmin};
    p3 = {xmax, ymax, zmin};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;
    p1 = {xmax, ymax, zmin};
    p2 = {xmax, ymin, zmin};
    p3 = {xmax, ymax, zmax};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    p1 = {xmax, ymin, zmax};
    p2 = {xmax, ymin, zmin};
    p3 = {xmax, ymax, zmax};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    p1 = {xmin, ymin, zmax};
    p2 = {xmin, ymax, zmax};
    p3 = {xmin, ymin, zmin};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    p1 = {xmin, ymax, zmin};
    p2 = {xmin, ymax, zmax};
    p3 = {xmin, ymin, zmin};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    p1 = {xmin, ymin, zmax};
    p2 = {xmax, ymax, zmax};
    p3 = {xmin, ymax, zmax};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;
    p1 = {xmin, ymin, zmax};
    p2 = {xmax, ymax, zmax};
    p3 = {xmax, ymin, zmax};
    if (triangleIntersects(p1, p2, p3, ray))
        return true;
    p1 = {xmax, ymax, zmin};
    p2 = {xmin, ymin, zmin};
    p3 = {xmax, ymin, zmin};

    if (triangleIntersects(p1, p2, p3, ray))
        return true;
    p1 = {xmax, ymax, zmin};
    p2 = {xmin, ymin, zmin};
    p3 = {xmin, ymax, zmin};
    if (triangleIntersects(p1, p2, p3, ray))
        return true;

    return false;
}
bool Mesh::triangleIntersects(Vector3f p1,
                              Vector3f p2,
                              Vector3f p3,
                              Ray ray) const {
    float a = p1.x - p2.x;
    float b = p1.y - p2.y;
    float c = p1.z - p2.z;
    float d = p1.x - p3.x;
    float e = p1.y - p3.y;
    float f = p1.z - p3.z;
    float g = ray.direction.x;
    float h = ray.direction.y;
    float i = ray.direction.z;
    float j = p1.x - ray.origin.x;
    float k = p1.y - ray.origin.y;
    float l = p1.z - ray.origin.z;

    float determinant =
        a * (e * i - h * f) + b * (g * f - d * i) + c * (d * h - e * g);
    if (determinant == 0) {
        return false;
    }

    else {
        float beta =
            (j * (e * i - h * f) + k * (g * f - d * i) + l * (d * h - e * g)) /
            determinant;
        float gamma =
            (i * (a * k - j * b) + h * (j * c - a * l) + g * (b * l - k * c)) /
            determinant;
        float t =
            -(f * (a * k - j * b) + e * (j * c - a * l) + d * (b * l - k * c)) /
            determinant;

        if (beta >= 0 && gamma >= 0 && beta + gamma <= 1) {
            return true;

        }

        else {
            return false;
        }
    }
}
