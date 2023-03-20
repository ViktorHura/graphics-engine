//
// Created by viktor on 20.03.20.
//

#include "../easy_image.h"
#include "../vector3d.h"
#include "ZBuffer.h"
#include <list>
#include <utility>
#include <cmath>

#ifndef ENGINE_UTILS_H
#define ENGINE_UTILS_H

class Color{
public:
    int r;
    int g;
    int b;
    double R;
    double G;
    double B;
    Color(int red, int green, int blue){
        r = red;
        g = green;
        b = blue;
        R = (double)red / 255.0;
        G = (double)green / 255.0;
        B = (double)blue / 255.0;
    }

    Color() {
        r = 0;
        g = 0;
        b = 0;
        R=0;
        G=0;
        B=0;
    }

    Color (const Color &to_copy);

    void updateInts(){
        if (R>1){R=1;}
        if (G>1){G=1;}
        if (B>1){B=1;}
        r = colorDouble_to_Int(R);
        g = colorDouble_to_Int(G);
        b = colorDouble_to_Int(B);
    }

    static int colorDouble_to_Int(double color);
    Color(std::vector<double>& color);
    Color(std::vector<int>& color);
};

class Point2D{
public:
    double x;
    double y;
    Point2D(double xpos, double ypos){
        x = xpos;
        y = ypos;
    }

    Point2D() {
        x = 0;
        y = 0;
    }
};

class Line2D;
using Lines2D = std::list<Line2D>;

class Line2D{
public:
    Point2D p1;
    Point2D p2;
    Color color;

    double z1;
    double z2;

    Line2D(Point2D& p1, Point2D& p2, Color& color){
        Line2D::p1 = p1;
        Line2D::p2 = p2;
        Line2D::color = Color(color.r, color.g, color.b);
    }

    void Draw(img::EasyImage& image);
    void DrawZ(img::EasyImage& image, ZBuffer& zbuf);
    static void drawLine(img::EasyImage &image, int &x1, int &y1, int &x2, int &y2, Color& color);
    static void drawLineZ(img::EasyImage &image, int &x1, int &y1, double z1, int &x2, int &y2, double z2, ZBuffer& zbuf, Color& color);
    static img::EasyImage draw2DLines(Lines2D &lines, const int size, Color& backgroundcolor);
    static img::EasyImage draw2DLinesZ(Lines2D &lines, const int size, Color &backgroundcolor);
};

class Face
{
public:
    //De indexen refereren naar
    //punten in de ‘points’ vector
    //van de Figure-klasse
    std::vector<int> points;

    Face(std::vector<int> pts){
        points = std::move(pts);
    }
};

class Figure
{
public:
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color color;
    Color diffuseReflection;
    Color specularReflection;
    double reflectionCoefficient;

    Figure(std::vector<Vector3D> pts, std::vector<Face> fcs, Color clr, Color diffuseReflection = Color(0,0,0), Color specularReflection = Color(0,0,0), double reflectionCoeff = 0){
        points = std::move(pts);
        faces = std::move(fcs);
        color = clr;
        Figure::diffuseReflection = diffuseReflection;
        Figure::specularReflection = specularReflection;
        Figure::reflectionCoefficient = reflectionCoeff;
    }

    Figure(){
    }

    void triangulate();

    void set(std::vector<Vector3D> pts, std::vector<Face> fcs, Color clr){
        points = std::move(pts);
        faces = std::move(fcs);
        color = clr;
    }

    void applyTransformation(const Matrix &mat);
    Figure (const Figure &to_copy);
};

typedef std::list<Figure> Figures3D;

class TransformMatrices{
public:
    static Matrix scaleFigure(double scale);
    static Matrix rotX(double angle);
    static Matrix rotY(double angle);
    static Matrix rotZ(double angle);
    static Matrix translate(const Vector3D &vector);
    static Matrix eyePointTrans(const Vector3D &eyepoint);
    static void toPolar(const Vector3D &point, double &theta, double &phi, double &r);
};

class Utils {
public:
    static void setIflessOrMore(double& min, double& max,const double& va);
    static void colorPixel(img::EasyImage& image, int& x, int& y, Color& color);
    static void fillColor(img::EasyImage& image, Color& color);
    static double degTorad(double deg);
    static Vector3D averagePoint(Vector3D& p1, Vector3D& p2, Vector3D& p3);
    static Vector3D averagePoint(Vector3D& p1, Vector3D& p2);
};


#endif //ENGINE_UTILS_H
