#ifndef _SHAPE_H_
#define _SHAPE_H_
#include <cmath>
#include <vector>
#include "Ray.h"
#include "defs.h"

using namespace std;

// Base class for any shape object
class Shape {
 public:
  int id;        // Id of the shape
  int matIndex;  // Material index of the shape

  virtual ReturnVal intersect(const Ray& ray)
      const = 0;  // Pure virtual method for intersection test. You must
                  // implement this for sphere, triangle, and mesh.

  Shape(void);
  Shape(int id, int matIndex);  // Constructor

 private:
  // Write any other stuff here
};

// Class for sphere
class Sphere : public Shape {
 public:
  Sphere(void);  // Constructor
  Sphere(int id,
         int matIndex,
         int cIndex,
         float R,
         vector<Vector3f>* vertices);  // Constructor

  ReturnVal intersect(const Ray& ray)
      const;  // Will take a ray and return a structure related to the
              // intersection information. You will implement this.

 private:
  const float radius;
  const int cIndex;
  const vector<Vector3f>* vertices;
};

// Class for triangle
class Triangle : public Shape {
 public:
  Triangle(void);  // Constructor
  Triangle(int id,
           int matIndex,
           int p1Index,
           int p2Index,
           int p3Index,
           vector<Vector3f>* vertices);  // Constructor

  ReturnVal intersect(const Ray& ray)
      const;  // Will take a ray and return a structure related to the
              // intersection information. You will implement this.

 private:
  const int p1Index;
  const int p2Index;
  const int p3Index;
  // Write any other stuff here
  const vector<Vector3f>* vertices;
};

// Class for mesh
class Mesh : public Shape {
 public:
  Mesh(void);  // Constructor
  Mesh(int id,
       int matIndex,
       const vector<Triangle>& faces,
       vector<int>* pIndices,
       vector<Vector3f>* vertices);  // Constructor

  ReturnVal intersect(const Ray& ray)
      const;  // Will take a ray and return a structure related to the
              // intersection information. You will implement this.

 private:
  const vector<Triangle> faces;
  const vector<Vector3f>* vertices;
  const vector<int>* pIndices;
  bool triangleIntersects(Vector3f p1, Vector3f p2, Vector3f p3, Ray ray) const;
  bool boundingBoxHit(Ray ray) const;
  float xmin;
  float ymin;
  float zmin;
  float xmax;
  float ymax;
  float zmax;
};

#endif
