//
// Created by viktor on 30.05.20.
//

#include "Fractal3D.h"

void Fractal3D::generateFractal(Figure &fig, Figures3D &fractal, int nr_iterations, const double scale) {
    for (int i =0; i < fig.points.size(); i++){
        Vector3D& point = fig.points[i];

        Figure copy = Figure(fig);
        Matrix scl = TransformMatrices::scaleFigure(1/scale);
        copy.applyTransformation(scl);

        Vector3D transvec = point - copy.points[i];
        Matrix trans = TransformMatrices::translate(transvec);
        copy.applyTransformation(trans);

        if (nr_iterations == 1) {
            fractal.push_back(copy);
        }else{
            generateFractal(copy,fractal,nr_iterations - 1, scale);
        }
    }
}
