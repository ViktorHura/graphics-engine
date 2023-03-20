//
// Created by viktor on 18.04.20.
//

#ifndef ENGINE_WIREFRAME_H
#define ENGINE_WIREFRAME_H

#include "../easy_image.h"
#include "../ini_configuration.h"
#include <string>
#include "Utils.h"

class Wireframe {
    friend class ZBufferedWireframe;
    friend class ZBufferingTri;
    friend class Lighting;
private:
    static bool parseFigure(std::string &index, const ini::Configuration &conf, Figures3D& figures, bool light = false);
    static Lines2D doProjection(const Figures3D &);
    static void projectFigure(const Figure& figure, Lines2D& lines);
    static Point2D projectPoint(const Vector3D &point, const double d);
public:
    static img::EasyImage parseConfig(std::string& type, const ini::Configuration &conf);
};


#endif //ENGINE_WIREFRAME_H
