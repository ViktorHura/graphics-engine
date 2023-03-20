//
// Created by viktor on 11.05.20.
//

#ifndef ENGINE_LSYSTEM3D_H
#define ENGINE_LSYSTEM3D_H


#include "Utils.h"
#include "../l_parser.h"

class LSystem3D {
public:
    static Figure drawLSystem(const LParser::LSystem3D &lsystem, Color& color);
};


#endif //ENGINE_LSYSTEM3D_H
