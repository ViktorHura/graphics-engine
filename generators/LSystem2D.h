#ifndef ENGINE_LSYSTEM2D_H
#define ENGINE_LSYSTEM2D_H


#include "../easy_image.h"
#include "../ini_configuration.h"
#include <string>
#include "../l_parser.h"
#include <fstream>
#include "Utils.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <stack>

class LSystem2D {
private:
    static Lines2D drawLSystem(const LParser::LSystem2D &lsystem,Color& color);
public:
    static img::EasyImage parseConfig(std::string& type, const ini::Configuration &conf);
};


#endif //ENGINE_LSYSTEM2D_H
