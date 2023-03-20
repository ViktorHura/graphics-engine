//
// Created by viktor on 29.05.20.
//

#ifndef ENGINE_ZBUFFERINGTRI_H
#define ENGINE_ZBUFFERINGTRI_H


#include "../easy_image.h"
#include "Utils.h"
#include "../ini_configuration.h"
#include "Lighting.h"

class ZBufferingTri {
    friend class Lighting;
private:
    static void draw_zbuf_tri(img::EasyImage &image, ZBuffer &buf, Vector3D &A, Vector3D &B, Vector3D &C, double &d, double &dx, double &dy, Color &color, bool lighting = false, Color diffuseReflection = Color(0,0,0), Color specularReflection = Color(0,0,0), double reflectionCoeff = 0,
                              std::list<Light> lights = Lights3D());
    static double min(double &y1, double &y2, double &y3);
    static double max(double &y1, double &y2, double &y3);
    static void calcLeftandRight(int& i, Point2D& a, Point2D& b, Point2D& c, double& xlab, double& xlac, double& xlbc, double& xrab, double& xrac, double& xrbc);
public:
    static img::EasyImage parseConfig(std::string& type, const ini::Configuration &conf);
};


#endif //ENGINE_ZBUFFERINGTRI_H
