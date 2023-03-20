//
// Created by viktor on 30.05.20.
//

#ifndef ENGINE_FRACTAL3D_H
#define ENGINE_FRACTAL3D_H


#include "Utils.h"

class Fractal3D {
public:
    static void generateFractal(Figure& fig, Figures3D& fractal, int nr_iterations, const double scale);
};


#endif //ENGINE_FRACTAL3D_H
