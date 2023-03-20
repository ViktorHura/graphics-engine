//
// Created by viktor on 30.05.20.
//

#ifndef ENGINE_LIGHTING_H
#define ENGINE_LIGHTING_H

#include "Utils.h"
#include "../ini_configuration.h"

class Light
{
public:
    //de ambiente licht component
    Color ambientLight;
    //de diffuse licht component
    Color diffuseLight = Color();
    //de diffuse licht component
    Color specularLight = Color();

    bool inf = true;
    Vector3D ldVector = Vector3D::vector(1,1,1);
    Vector3D location;
    double angle;

};


typedef std::list<Light> Lights3D;

class Lighting {
    friend class ZBufferingTri;
    static void parseLight(std::string &index, const ini::Configuration &conf, Lights3D &lights, Matrix& eyepma);
    static void applyLight(Light &light, Color &color, Color& diffuseReflection, Color& specularReflection, double& reflectionCoeff, Color &final, Vector3D& normal, int& x, int& y, double& dx, double& dy, double& d, double zr);
    static void applyLightTri(Light &light, Color &color, Color &diffuseReflection, Color &final, Vector3D &vector3D);
public:
    static img::EasyImage parseConfig(std::string& type, const ini::Configuration &conf);
};


#endif //ENGINE_LIGHTING_H
