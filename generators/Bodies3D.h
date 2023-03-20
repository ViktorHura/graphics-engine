//
// Created by viktor on 08.05.20.
//

#ifndef ENGINE_BODIES3D_H
#define ENGINE_BODIES3D_H


#include "Utils.h"

class Bodies3D {
public:
static Figure Cube(Color &color);
static Figure Tetrahedron(Color &color);
static Figure Octahedron(Color &color);
static Figure Icosahedron(Color &color);
static Figure Dodecahedron(Color &color);
static Figure Sphere(int n, Color &color);
static Figure Cone(int n, double h, Color &color);
static Figure Cylinder(int n, double h, Color &color, bool tube = false);
static Figure Torus(double R, double r, int n, int m, Color &color);
static void generateThickFigures(Figure& figure,Figures3D& figures,const double r,const int n, const int m);
};


#endif //ENGINE_BODIES3D_H
