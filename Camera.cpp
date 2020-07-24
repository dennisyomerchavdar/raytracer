#include "Camera.h"
#include "MathUtil.h"
#include "cstring"
Camera::Camera(int id,                      // Id of the camera
               const char* imageName,       // Name of the output PPM file
               const Vector3f& pos,         // Camera position
               const Vector3f& gaze,        // Camera gaze direction
               const Vector3f& up,          // Camera up direction
               const ImagePlane& imgPlane)  // Image plane parameters
{
    this->id = id;
    strcpy(this->imageName, imageName);
    this->imgPlane = imgPlane;
    this->pos = pos;
    this->gaze = gaze;
    this->up = up;
}

/* Takes coordinate of an image pixel as row and col, and
 * returns the ray going through that pixel.
 */
Ray Camera::getPrimaryRay(int row, int col) const {
    float u = imgPlane.left + (imgPlane.right - imgPlane.left) * (col + 0.5) / imgPlane.nx;
    float v = imgPlane.bottom + (imgPlane.top - imgPlane.bottom) * (imgPlane.ny - row - 0.5) / imgPlane.ny;

    Vector3f origin = pos;
    Vector3f wvec = MathUtil::scalarMultVector(-1, gaze);
    Vector3f vvec = up;
    Vector3f uvec = MathUtil::crossProdVectors(vvec, wvec);
    Vector3f direction = MathUtil::normalize(MathUtil::addVectors(MathUtil::addVectors(MathUtil::scalarMultVector(-imgPlane.distance, wvec), MathUtil::scalarMultVector(u, uvec)), MathUtil::scalarMultVector(v, vvec)));

    Ray primRay(origin, direction);

    return primRay;
}
