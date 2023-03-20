//
// Created by viktor on 18.04.20.
//

#include <cassert>
#include <fstream>
#include "Wireframe.h"
#include "Bodies3D.h"
#include "LSystem3D.h"
#include "Fractal3D.h"

img::EasyImage Wireframe::parseConfig(std::string &type, const ini::Configuration &conf) {

    int size = conf["General"]["size"].as_int_or_die();
    std::vector<double> eye = conf["General"]["eye"].as_double_tuple_or_die();
    std::vector<double> bcol = conf["General"]["backgroundcolor"].as_double_tuple_or_die();

    Color backgroundcolor = Color(bcol);

    int nrfigures = conf["General"]["nrFigures"].as_int_or_die();

    Figures3D figures;

    for (int i=0; i < nrfigures; i++){
        std::string figurename = "Figure" + std::to_string(i);

        if(!parseFigure(figurename, conf, figures)){
            return img::EasyImage();
        }

    }

    Vector3D eyepoint = Vector3D::point(eye[0], eye[1], eye[2]);
    Matrix eyepmat= TransformMatrices::eyePointTrans(eyepoint);

    for (Figures3D::iterator it = figures.begin(); it != figures.end(); it++){
        it->applyTransformation(eyepmat);
    }

    Lines2D lines = doProjection(figures);

    return Line2D::draw2DLines(lines, size, backgroundcolor);
}

bool Wireframe::parseFigure(std::string &index, const ini::Configuration &conf, Figures3D &figures, bool light) {
    std::string type = conf[index]["type"].as_string_or_die();

    double scale = conf[index]["scale"].as_double_or_die();

    double rotX = conf[index]["rotateX"].as_double_or_die();
    double rotY = conf[index]["rotateY"].as_double_or_die();
    double rotZ = conf[index]["rotateZ"].as_double_or_die();

    rotX = Utils::degTorad(rotX);
    rotY = Utils::degTorad(rotY);
    rotZ = Utils::degTorad(rotZ);

    std::vector<double> ctr = conf[index]["center"].as_double_tuple_or_die();
    Vector3D center = Vector3D::point(ctr[0], ctr[1], ctr[2]);

    std::vector<double> clr;
    if (light){
        clr = conf[index]["ambientReflection"].as_double_tuple_or_die();
    }else{
        clr = conf[index]["color"].as_double_tuple_or_die();
    }

    Color color = Color(clr);

    double fractalScale = 0;
    conf[index]["fractalScale"].as_double_if_exists(fractalScale);

    int fractalit = 0;
    conf[index]["nrIterations"].as_int_if_exists(fractalit);

    Figure figure = Figure();

    if (type == "LineDrawing" || type =="ThickLineDrawing"){
        std::vector<Vector3D> points;
        std::vector<Face> faces;

        int nrPoints = conf[index]["nrPoints"].as_int_or_die();

        for (int i=0; i < nrPoints; i++){
            std::string pi = "point" + std::to_string(i);

            std::vector<double> pnt = conf[index][pi].as_double_tuple_or_die();
            Vector3D point = Vector3D::point(pnt[0], pnt[1], pnt[2]);
            points.push_back(point);
        }

        int nrLines = conf[index]["nrLines"].as_int_or_die();

        for (int i=0; i < nrLines; i++){
            std::string li = "line" + std::to_string(i);

            std::vector<double> ln = conf[index][li].as_double_tuple_or_die();
            std::vector<int> pts;
            pts.push_back(ln[0]);
            pts.push_back(ln[1]);
            Face face = Face(pts);
            faces.push_back(face);
        }

        figure.set(points,faces,color);
    }else if (type=="Cube" || type =="FractalCube" || type =="ThickCube"){
        figure = Bodies3D::Cube(color);
    }else if (type == "Tetrahedron" || type == "FractalTetrahedron" || type =="ThickTetrahedron"){
        figure = Bodies3D::Tetrahedron(color);
    }else if (type == "Octahedron" || type == "FractalOctahedron" || type =="ThickOctahedron"){
        figure = Bodies3D::Octahedron(color);
    }else if (type == "Icosahedron" || type == "FractalIcosahedron" || type =="ThickIcosahedron"){
        figure = Bodies3D::Icosahedron(color);
    }else if(type =="Dodecahedron" || type =="FractalDodecahedron" || type =="ThickDodecahedron"){
        figure = Bodies3D::Dodecahedron(color);
    }else if(type =="Sphere"){
        int n = conf[index]["n"].as_int_or_die();
        figure = Bodies3D::Sphere(n, color);
    }else if(type =="Cone"){
        int n = conf[index]["n"].as_int_or_die();
        double h = conf[index]["height"].as_double_or_die();
        figure = Bodies3D::Cone(n, h, color);
    }else if(type =="Cylinder"){
        int n = conf[index]["n"].as_int_or_die();
        double h = conf[index]["height"].as_double_or_die();
        figure = Bodies3D::Cylinder(n, h, color);
    }else if(type =="Torus"){
        double R = conf[index]["R"].as_double_or_die();
        double r = conf[index]["r"].as_double_or_die();
        int n = conf[index]["n"].as_int_or_die();
        int m = conf[index]["m"].as_int_or_die();

        figure = Bodies3D::Torus(R,r,n, m, color);
    }else if(type =="3DLSystem" || type == "Thick3DLSystem"){
        std::string lfile = conf[index]["inputfile"].as_string_or_die();

        LParser::LSystem3D lsystem;

        std::ifstream input_stream(lfile);
        input_stream >> lsystem;
        input_stream.close();

        figure = LSystem3D::drawLSystem(lsystem,color);
    }else{
        return false;
    }

    Matrix transforms = TransformMatrices::scaleFigure(scale) * TransformMatrices::rotX(rotX) * TransformMatrices::rotY(rotY) * TransformMatrices::rotZ(rotZ) * TransformMatrices::translate(center);
    figure.applyTransformation(transforms);

    if (light){
        std::vector<double> difuse;
        if(conf[index]["diffuseReflection"].as_double_tuple_if_exists(difuse)){
            Color dif = Color(difuse);
            figure.diffuseReflection = dif;
        }

        std::vector<double> specular;
        if(conf[index]["specularReflection"].as_double_tuple_if_exists(specular)){
            Color spec = Color(specular);
            figure.specularReflection = spec;
            figure.reflectionCoefficient = conf[index]["reflectionCoefficient"].as_double_or_die();
        }
    }

    if (type.substr(0, 7) == "Fractal" && fractalit > 0){
        Fractal3D::generateFractal(figure,figures,fractalit,fractalScale);
    } else if (type.substr(0, 5) == "Thick"){
        double radius = conf[index]["radius"].as_double_or_die();
        int nthick = conf[index]["n"].as_int_or_die();
        int mthick = conf[index]["m"].as_int_or_die();
        Bodies3D::generateThickFigures(figure,figures,radius,nthick,mthick);
        //figures.push_back(figure);
    }else {
        figures.push_back(figure);
    }
    return true;
}

Lines2D Wireframe::doProjection(const Figures3D& figures) {
    Lines2D lines;

    for (std::_List_const_iterator<Figure> it = figures.begin(); it != figures.end(); it++){
        projectFigure(*it, lines);
    }

    return lines;
}

void Wireframe::projectFigure(const Figure &figure, Lines2D &lines) {
    int nrFaces = figure.faces.size();
    for (int i=0; i < nrFaces; i++){
        Face face = figure.faces[i];

        for (int j=0;j<face.points.size(); j++){

            int next = j+1;
            if(next == face.points.size()){
                next = 0;
            }

            Point2D p1 = projectPoint(figure.points[face.points[j]],1);
            Point2D p2 = projectPoint(figure.points[face.points[next]],1);

            Color col = figure.color;
            Line2D line = Line2D(p1,p2, col);

            lines.push_back(line);
        }
    }

}

Point2D Wireframe::projectPoint(const Vector3D &point, const double d) {
    Point2D pnt;

    pnt.x = (d * point.x) / (-point.z);
    pnt.y = (d * point.y) / (-point.z);

    return pnt;
}
