#include "Scene.h"
#include <pthread.h>
#include <cmath>
#include <string>
#include "tinyxml2.h"
using namespace tinyxml2;

/*
 * Must render the scene from each camera's viewpoint and create an image.
 * You can use the methods of the Image class to save the image as a PPM file.
 */

typedef struct ThreadData {
    int x;
    int y;
    Camera* camera;
    Image* resultImage;
} ThreadData;

void* renderPixel(void* threadArg) {
    struct ThreadData* threadData;
    threadData = (struct ThreadData*)threadArg;
    int x = threadData->x;
    int y = threadData->y;

    Camera& cam = *threadData->camera;
    Image& resultImage = *threadData->resultImage;
    Ray primRay = cam.getPrimaryRay(y, x);

    Vector3f totalCol = {0, 0, 0};
    Vector3f multiplier = {1, 1, 1};
    for (int recursionCount = 0; recursionCount < pScene->maxRecursionDepth + 1;
         recursionCount++) {
        ReturnVal res;
        Vector3f col = {0, 0, 0};
        res.valid = false;
        float lowestt = INFINITY;
        for (Shape* shap : pScene->objects) {
            if (shap->id == -1) {
                continue;
            }
            ReturnVal candidate = shap->intersect(primRay);
            if (candidate.valid) {
                float tForCandidate = primRay.gett(candidate.point);
                if (tForCandidate < lowestt) {
                    lowestt = tForCandidate;
                    res = candidate;
                }
            }
        }

        if (res.valid) {
            Vector3f diff = {pScene->materials[res.matID - 1]->diffuseRef.r,
                             pScene->materials[res.matID - 1]->diffuseRef.g,
                             pScene->materials[res.matID - 1]->diffuseRef.b};

            Vector3f amb = {pScene->materials[res.matID - 1]->ambientRef.r,
                            pScene->materials[res.matID - 1]->ambientRef.g,
                            pScene->materials[res.matID - 1]->ambientRef.b};

            Vector3f spec = {pScene->materials[res.matID - 1]->specularRef.r,
                             pScene->materials[res.matID - 1]->specularRef.g,
                             pScene->materials[res.matID - 1]->specularRef.b};

            Vector3f refl = {pScene->materials[res.matID - 1]->mirrorRef.r,
                             pScene->materials[res.matID - 1]->mirrorRef.g,
                             pScene->materials[res.matID - 1]->mirrorRef.b};

            int p = pScene->materials[res.matID - 1]->phongExp;

            for (PointLight* pointLight : pScene->lights) {
                Vector3f illuminance =
                    pointLight->computeLightContribution(res.point);
                Vector3f n = MathUtil::normalize(res.normal);
                Vector3f l = MathUtil::normalize(MathUtil::addVectors(
                    pointLight->position,
                    MathUtil::scalarMultVector(-1, res.point)));
                Vector3f v = MathUtil::normalize(
                    MathUtil::subVectors(primRay.origin, res.point));

                Vector3f h = MathUtil::normalize(MathUtil::addVectors(v, l));
                float prod = max(0.0f, MathUtil::dotProdVectors(n, l));

                float blinn = pow(max(0.0f, MathUtil::dotProdVectors(n, h)), p);
                Ray shadowRay;
                shadowRay = {MathUtil::addVectors(res.point,
                                                  MathUtil::scalarMultVector(
                                                      pScene->shadowRayEps, l)),
                             l};
                bool shadow = false;
                for (Shape* obstacle : pScene->objects) {
                    ReturnVal obshit = obstacle->intersect(shadowRay);
                    if (obshit.valid) {
                        if (shadowRay.gett(obshit.point) <
                            shadowRay.gett(pointLight->position)) {
                            shadow = true;
                        }
                    }
                }
                if (!shadow) {
                    col.r += diff.r * illuminance.r * prod +
                             spec.r * illuminance.r * blinn;
                    col.g += diff.g * illuminance.g * prod +
                             spec.g * illuminance.g * blinn;
                    col.b += diff.b * illuminance.b * prod +
                             spec.b * illuminance.b * blinn;
                }
            }

            col.r += amb.r * pScene->ambientLight.r;
            col.g += amb.g * pScene->ambientLight.g;
            col.b += amb.b * pScene->ambientLight.b;

            totalCol.r += col.r * multiplier.r;
            totalCol.g += col.g * multiplier.g;
            totalCol.b += col.b * multiplier.b;
            if (refl.r == 0 && refl.g == 0 && refl.b == 0) {
                break;
            }
            multiplier.r *= refl.r;
            multiplier.g *= refl.g;
            multiplier.b *= refl.b;

            primRay.origin = res.point;
            primRay.direction = MathUtil::normalize(MathUtil::subVectors(
                MathUtil::normalize(primRay.direction),
                MathUtil::scalarMultVector(
                    2 * (MathUtil::dotProdVectors(primRay.direction,
                                                  res.normal)),
                    res.normal)));
            primRay.origin = MathUtil::addVectors(
                primRay.origin, MathUtil::scalarMultVector(pScene->shadowRayEps,
                                                           primRay.direction));
        } else {
            totalCol.r += pScene->backgroundColor.r;
            totalCol.g += pScene->backgroundColor.g;
            totalCol.b += pScene->backgroundColor.b;
            break;
        }

    }  // close recur

    Color resultColor = {
        totalCol.r > 255 ? (unsigned char)255 : (unsigned char)totalCol.r,
        totalCol.g > 255 ? (unsigned char)255 : (unsigned char)totalCol.g,
        totalCol.b > 255 ? (unsigned char)255 : (unsigned char)totalCol.b};

    resultImage.setPixelValue(x, y, resultColor);
    pthread_exit(NULL);
}

void Scene::renderScene(void) {
    int camnum = 0;
    for (Camera* cam : cameras) {
        int nx = cam->imgPlane.nx;
        int ny = cam->imgPlane.ny;
        pthread_t threads[nx][ny];
        Image* resultImage = new Image(nx, ny);
        ThreadData** threadDatas = new ThreadData*[nx];
        for (int i = 0; i < nx; ++i)
            threadDatas[i] = new ThreadData[ny];
        int blockSize = 10;
        for (int x = 0; x < nx; x += blockSize) {
            for (int y = 0; y < ny; y += blockSize) {
                for (int i = x; i < x + blockSize && i < nx; i++) {
                    for (int j = y; j < y + blockSize && j < ny; j++) {
                        threadDatas[i][j].x = i;
                        threadDatas[i][j].y = j;
                        threadDatas[i][j].camera = cam;
                        threadDatas[i][j].resultImage = resultImage;
                        pthread_create(&threads[i][j], NULL, renderPixel,
                                       (void*)&threadDatas[i][j]);
                    }
                }
                for (int i = x; i < x + blockSize && i < nx; i++) {
                    for (int j = y; j < y + blockSize && j < ny; j++) {
                        pthread_join(threads[i][j], NULL);
                    }
                }
            }
        }
        resultImage->saveImage((std::to_string(camnum) + ".ppm").c_str());

        camnum++;
    }
}
// Parses XML file.
Scene::Scene(const char* xmlPath) {
    const char* str;
    XMLDocument xmlDoc;
    XMLError eResult;
    XMLElement* pElement;

    maxRecursionDepth = 1;
    shadowRayEps = 0.001;

    eResult = xmlDoc.LoadFile(xmlPath);

    XMLNode* pRoot = xmlDoc.FirstChild();

    pElement = pRoot->FirstChildElement("MaxRecursionDepth");
    if (pElement != nullptr)
        pElement->QueryIntText(&maxRecursionDepth);

    pElement = pRoot->FirstChildElement("BackgroundColor");
    str = pElement->GetText();
    sscanf(str, "%f %f %f", &backgroundColor.r, &backgroundColor.g,
           &backgroundColor.b);

    pElement = pRoot->FirstChildElement("ShadowRayEpsilon");
    if (pElement != nullptr)
        pElement->QueryFloatText(&shadowRayEps);

    pElement = pRoot->FirstChildElement("IntersectionTestEpsilon");
    if (pElement != nullptr)
        eResult = pElement->QueryFloatText(&intTestEps);

    // Parse cameras
    pElement = pRoot->FirstChildElement("Cameras");
    XMLElement* pCamera = pElement->FirstChildElement("Camera");
    XMLElement* camElement;
    while (pCamera != nullptr) {
        int id;
        char imageName[64];
        Vector3f pos, gaze, up;
        ImagePlane imgPlane;

        eResult = pCamera->QueryIntAttribute("id", &id);
        camElement = pCamera->FirstChildElement("Position");
        str = camElement->GetText();
        sscanf(str, "%f %f %f", &pos.x, &pos.y, &pos.z);
        camElement = pCamera->FirstChildElement("Gaze");
        str = camElement->GetText();
        sscanf(str, "%f %f %f", &gaze.x, &gaze.y, &gaze.z);
        camElement = pCamera->FirstChildElement("Up");
        str = camElement->GetText();
        sscanf(str, "%f %f %f", &up.x, &up.y, &up.z);
        camElement = pCamera->FirstChildElement("NearPlane");
        str = camElement->GetText();
        sscanf(str, "%f %f %f %f", &imgPlane.left, &imgPlane.right,
               &imgPlane.bottom, &imgPlane.top);
        camElement = pCamera->FirstChildElement("NearDistance");
        eResult = camElement->QueryFloatText(&imgPlane.distance);
        camElement = pCamera->FirstChildElement("ImageResolution");
        str = camElement->GetText();
        sscanf(str, "%d %d", &imgPlane.nx, &imgPlane.ny);
        camElement = pCamera->FirstChildElement("ImageName");
        str = camElement->GetText();
        strcpy(imageName, str);

        cameras.push_back(new Camera(id, imageName, pos, gaze, up, imgPlane));

        pCamera = pCamera->NextSiblingElement("Camera");
    }

    // Parse materals
    pElement = pRoot->FirstChildElement("Materials");
    XMLElement* pMaterial = pElement->FirstChildElement("Material");
    XMLElement* materialElement;
    while (pMaterial != nullptr) {
        materials.push_back(new Material());

        int curr = materials.size() - 1;

        eResult = pMaterial->QueryIntAttribute("id", &materials[curr]->id);
        materialElement = pMaterial->FirstChildElement("AmbientReflectance");
        str = materialElement->GetText();
        sscanf(str, "%f %f %f", &materials[curr]->ambientRef.r,
               &materials[curr]->ambientRef.g, &materials[curr]->ambientRef.b);
        materialElement = pMaterial->FirstChildElement("DiffuseReflectance");
        str = materialElement->GetText();
        sscanf(str, "%f %f %f", &materials[curr]->diffuseRef.r,
               &materials[curr]->diffuseRef.g, &materials[curr]->diffuseRef.b);
        materialElement = pMaterial->FirstChildElement("SpecularReflectance");
        str = materialElement->GetText();
        sscanf(str, "%f %f %f", &materials[curr]->specularRef.r,
               &materials[curr]->specularRef.g,
               &materials[curr]->specularRef.b);
        materialElement = pMaterial->FirstChildElement("MirrorReflectance");
        if (materialElement != nullptr) {
            str = materialElement->GetText();
            sscanf(str, "%f %f %f", &materials[curr]->mirrorRef.r,
                   &materials[curr]->mirrorRef.g,
                   &materials[curr]->mirrorRef.b);
        } else {
            materials[curr]->mirrorRef.r = 0.0;
            materials[curr]->mirrorRef.g = 0.0;
            materials[curr]->mirrorRef.b = 0.0;
        }
        materialElement = pMaterial->FirstChildElement("PhongExponent");
        if (materialElement != nullptr)
            materialElement->QueryIntText(&materials[curr]->phongExp);

        pMaterial = pMaterial->NextSiblingElement("Material");
    }

    // Parse vertex data
    pElement = pRoot->FirstChildElement("VertexData");
    int cursor = 0;
    Vector3f tmpPoint;
    str = pElement->GetText();
    while (str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
        cursor++;
    while (str[cursor] != '\0') {
        for (int cnt = 0; cnt < 3; cnt++) {
            if (cnt == 0)
                tmpPoint.x = atof(str + cursor);
            else if (cnt == 1)
                tmpPoint.y = atof(str + cursor);
            else
                tmpPoint.z = atof(str + cursor);
            while (str[cursor] != ' ' && str[cursor] != '\t' &&
                   str[cursor] != '\n')
                cursor++;
            while (str[cursor] == ' ' || str[cursor] == '\t' ||
                   str[cursor] == '\n')
                cursor++;
        }
        vertices.push_back(tmpPoint);
    }

    // Parse objects
    pElement = pRoot->FirstChildElement("Objects");

    // Parse spheres
    XMLElement* pObject = pElement->FirstChildElement("Sphere");
    XMLElement* objElement;
    while (pObject != nullptr) {
        int id;
        int matIndex;
        int cIndex;
        float R;

        eResult = pObject->QueryIntAttribute("id", &id);
        objElement = pObject->FirstChildElement("Material");
        eResult = objElement->QueryIntText(&matIndex);
        objElement = pObject->FirstChildElement("Center");
        eResult = objElement->QueryIntText(&cIndex);
        objElement = pObject->FirstChildElement("Radius");
        eResult = objElement->QueryFloatText(&R);

        objects.push_back(new Sphere(id, matIndex, cIndex, R, &vertices));

        pObject = pObject->NextSiblingElement("Sphere");
    }

    // Parse triangles
    pObject = pElement->FirstChildElement("Triangle");
    while (pObject != nullptr) {
        int id;
        int matIndex;
        int p1Index;
        int p2Index;
        int p3Index;

        eResult = pObject->QueryIntAttribute("id", &id);
        objElement = pObject->FirstChildElement("Material");
        eResult = objElement->QueryIntText(&matIndex);
        objElement = pObject->FirstChildElement("Indices");
        str = objElement->GetText();
        sscanf(str, "%d %d %d", &p1Index, &p2Index, &p3Index);

        objects.push_back(
            new Triangle(id, matIndex, p1Index, p2Index, p3Index, &vertices));

        pObject = pObject->NextSiblingElement("Triangle");
    }

    // Parse meshes
    pObject = pElement->FirstChildElement("Mesh");
    while (pObject != nullptr) {
        int id;
        int matIndex;
        int p1Index;
        int p2Index;
        int p3Index;
        int cursor = 0;
        int vertexOffset = 0;
        vector<Triangle> faces;
        vector<int>* meshIndices = new vector<int>;

        eResult = pObject->QueryIntAttribute("id", &id);
        objElement = pObject->FirstChildElement("Material");
        eResult = objElement->QueryIntText(&matIndex);
        objElement = pObject->FirstChildElement("Faces");
        objElement->QueryIntAttribute("vertexOffset", &vertexOffset);
        str = objElement->GetText();
        while (str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
            cursor++;
        while (str[cursor] != '\0') {
            for (int cnt = 0; cnt < 3; cnt++) {
                if (cnt == 0)
                    p1Index = atoi(str + cursor) + vertexOffset;
                else if (cnt == 1)
                    p2Index = atoi(str + cursor) + vertexOffset;
                else
                    p3Index = atoi(str + cursor) + vertexOffset;
                while (str[cursor] != ' ' && str[cursor] != '\t' &&
                       str[cursor] != '\n')
                    cursor++;
                while (str[cursor] == ' ' || str[cursor] == '\t' ||
                       str[cursor] == '\n')
                    cursor++;
            }
            faces.push_back(*(new Triangle(-1, matIndex, p1Index, p2Index,
                                           p3Index, &vertices)));
            meshIndices->push_back(p1Index);
            meshIndices->push_back(p2Index);
            meshIndices->push_back(p3Index);
        }

        objects.push_back(
            new Mesh(id, matIndex, faces, meshIndices, &vertices));

        pObject = pObject->NextSiblingElement("Mesh");
    }

    // Parse lights
    int id;
    Vector3f position;
    Vector3f intensity;
    pElement = pRoot->FirstChildElement("Lights");

    XMLElement* pLight = pElement->FirstChildElement("AmbientLight");
    XMLElement* lightElement;
    str = pLight->GetText();
    sscanf(str, "%f %f %f", &ambientLight.r, &ambientLight.g, &ambientLight.b);

    pLight = pElement->FirstChildElement("PointLight");
    while (pLight != nullptr) {
        eResult = pLight->QueryIntAttribute("id", &id);
        lightElement = pLight->FirstChildElement("Position");
        str = lightElement->GetText();
        sscanf(str, "%f %f %f", &position.x, &position.y, &position.z);
        lightElement = pLight->FirstChildElement("Intensity");
        str = lightElement->GetText();
        sscanf(str, "%f %f %f", &intensity.r, &intensity.g, &intensity.b);

        lights.push_back(new PointLight(position, intensity));

        pLight = pLight->NextSiblingElement("PointLight");
    }
}