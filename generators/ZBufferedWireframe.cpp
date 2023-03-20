//
// Created by viktor on 28.05.20.
//

#include <fstream>
#include "ZBufferedWireframe.h"
#include "Bodies3D.h"
#include "LSystem3D.h"
#include "Wireframe.h"

img::EasyImage ZBufferedWireframe::parseConfig(std::string &type, const ini::Configuration &conf) {
    int size = conf["General"]["size"].as_int_or_die();
    std::vector<double> eye = conf["General"]["eye"].as_double_tuple_or_die();
    std::vector<double> bcol = conf["General"]["backgroundcolor"].as_double_tuple_or_die();

    Color backgroundcolor = Color(bcol);

    int nrfigures = conf["General"]["nrFigures"].as_int_or_die();

    Figures3D figures;

    for (int i=0; i < nrfigures; i++){
        std::string figurename = "Figure" + std::to_string(i);

        if(!Wireframe::parseFigure(figurename, conf, figures)){
            return img::EasyImage();
        }

    }

    Vector3D eyepoint = Vector3D::point(eye[0], eye[1], eye[2]);
    Matrix eyepmat= TransformMatrices::eyePointTrans(eyepoint);

    for (Figures3D::iterator it = figures.begin(); it != figures.end(); it++){
        it->applyTransformation(eyepmat);
    }

    Lines2D lines = doProjection(figures);

    return Line2D::draw2DLinesZ(lines, size, backgroundcolor);
}



Lines2D ZBufferedWireframe::doProjection(const Figures3D& figures) {
    Lines2D lines;

    for (std::_List_const_iterator<Figure> it = figures.begin(); it != figures.end(); it++){
        projectFigure(*it, lines);
    }

    return lines;
}

void ZBufferedWireframe::projectFigure(const Figure &figure, Lines2D &lines) {
    int nrFaces = figure.faces.size();
    for (int i=0; i < nrFaces; i++){
        Face face = figure.faces[i];

        for (int j=0;j<face.points.size(); j++){

            int next = j+1;
            if(next == face.points.size()){
                next = 0;
            }

            double z1;
            double z2;

            Point2D p1 = projectPoint(figure.points[face.points[j]],1, z1);
            Point2D p2 = projectPoint(figure.points[face.points[next]],1, z2);

            Color col = figure.color;
            Line2D line = Line2D(p1,p2, col);
            line.z1 = z1;
            line.z2 = z2;

            lines.push_back(line);
        }
    }

}

Point2D ZBufferedWireframe::projectPoint(const Vector3D &point, const double& d, double& z, const double& dx, const double& dy) {
    Point2D pnt;

    pnt.x = (d * point.x) / (-point.z) + dx;
    pnt.y = (d * point.y) / (-point.z) + dy;
    z = point.z;

    return pnt;
}