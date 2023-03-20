//
// Created by viktor on 28.05.20.
//

#ifndef ENGINE_ZBUFFEREDWIREFRAME_H
#define ENGINE_ZBUFFEREDWIREFRAME_H
#include "../easy_image.h"
#include "../ini_configuration.h"
#include <string>
#include "Utils.h"

class ZBufferedWireframe {
    friend class ZBufferingTri;
    friend class Lighting;
private:
    static Lines2D doProjection(const Figures3D &);
    static void projectFigure(const Figure& figure, Lines2D& lines);
    static Point2D projectPoint(const Vector3D &point, const double& d, double& z, const double& dx = 0, const double& dy = 0);
public:
    static img::EasyImage parseConfig(std::string& type, const ini::Configuration &conf);
};


#endif //ENGINE_ZBUFFEREDWIREFRAME_H
