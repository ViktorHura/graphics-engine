//
// Created by viktor on 20.03.20.
//

#include "Utils.h"
#include "math.h"


void Utils::colorPixel(img::EasyImage &image, int &x, int &y, Color &color) {
    image(x,y).red = color.r;
    image(x,y).green = color.g;
    image(x,y).blue = color.b;
}

void Utils::fillColor(img::EasyImage &image, Color& color) {
    int w = image.get_width();
    int h = image.get_height();

    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            colorPixel(image, x, y, color);
        }
    }
}

double Utils::degTorad(double deg) {
    return deg* M_PI / 180;
}

Vector3D Utils::averagePoint(Vector3D &p1, Vector3D &p2, Vector3D &p3) {
    return Vector3D::point((p1.x + p2.x + p3.x) / 3, (p1.y + p2.y + p3.y) / 3, (p1.z + p2.z + p3.z) / 3);
}

Vector3D Utils::averagePoint(Vector3D &p1, Vector3D &p2) {
    return Vector3D::point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, (p1.z + p2.z) / 2);
}

Color::Color(std::vector<double> &color) {
    r = colorDouble_to_Int(color[0]);
    R = color[0];
    g = colorDouble_to_Int(color[1]);
    G = color[1];
    b = colorDouble_to_Int(color[2]);
    B= color[2];
}

int Color::colorDouble_to_Int(double color) {
    return (int)(255*color);
}

Color::Color(std::vector<int> &color) {
    r = color[0];
    g = color[1];
    b = color[2];
}

Color::Color(const Color &to_copy) {
    r = to_copy.r;
    g = to_copy.g;
    b = to_copy.b;
    R = to_copy.R;
    G = to_copy.G;
    B = to_copy.B;
}

void Line2D::Draw(img::EasyImage &image) {
    int x1 = std::round(p1.x);
    int y1 = std::round(p1.y);
    int x2 = std::round(p2.x);
    int y2 = std::round(p2.y);

    drawLine(image, x1, y1, x2,y2, color);
}

void Line2D::drawLine(img::EasyImage &image, int &x1, int &y1, int &x2, int &y2, Color &color) {
    img::Color clr = img::Color(color.r,color.g,color.b);
    return image.draw_line(x1,y1,x2,y2,clr);
    /*Utils::colorPixel(image, x1, y1, color);
    Utils::colorPixel(image, x2, y2, color);

    if (x1 == x2){
        for (int y = std::min(y1,y2); y < std::max(y1,y2); y++ ){
            Utils::colorPixel(image, x1, y, color);
        }
        return;
    }

    if (y1 == y2){
        for (int x = std::min(x1,x2); x < std::max(x1,x2); x++ ){
            Utils::colorPixel(image, x, y1, color);
        }
        return;
    }

    int xA = std::min(x1,x2);
    int xB = std::max(x1,x2);
    int yA;
    int yB;

    if (xB == x2){
        yB = y2;
        yA = y1;
    }else{
        yB = y1;
        yA = y2;
    }

    double m = ((double)yB - (double)yA) / ((double)xB - (double)xA);

    if ( m <= 1 && m >= -1 && m != 0 ){
        int dist = xB-xA;
        for (int i = 0; i < dist; i++){
            int x = xA + i;
            int y = std::round((double)yA + m* double(i));

            Utils::colorPixel(image, x, y, color);
        }
        return;
    }

    if (m > 1){
        int dist = yB-yA;
        for (int i = 0; i < dist; i++){
            int x = std::round((double)xA + (double)i / m);
            int y = yA + i;

            Utils::colorPixel(image, x, y, color);
        }
        return;
    }

    if (m < -1){
        int dist = yA-yB;
        for (int i = 0; i < dist; i++){
            int x = std::round((double)xA - (double)i / m);
            int y = yA - i;

            Utils::colorPixel(image, x, y, color);
        }
        return;
    }*/
}

void Utils::setIflessOrMore(double& min, double& max,const double& val){
    if (val < min){
        min = val;
    }
    if (val > max){
        max = val;
    }
}

img::EasyImage Line2D::draw2DLines(Lines2D &lines, const int size, Color& backgroundcolor) {
    double xmin = INFINITY;
    double xmax = -INFINITY;
    double ymin = INFINITY;
    double ymax = -INFINITY;


    for (std::_List_const_iterator<Line2D> it = lines.begin(); it != lines.end(); ++it){
        Utils::setIflessOrMore(xmin, xmax, it->p1.x);
        Utils::setIflessOrMore(ymin, ymax, it->p1.y);

        Utils::setIflessOrMore(xmin, xmax, it->p2.x);
        Utils::setIflessOrMore(ymin, ymax, it->p2.y);
    }

    double xrange = xmax - xmin;
    double yrange = ymax - ymin;
    double maxrange = std::max(xrange, yrange);

    double imageX = size * xrange / maxrange;
    double imageY = size * yrange / maxrange;

    img::EasyImage image(std::round(imageX), std::round(imageY));
    Utils::fillColor(image,backgroundcolor);

    double d = 0.95 * imageX / xrange;

    double dcx = d*(xmin + xmax)/2;
    double dcy = d*(ymin + ymax)/2;

    double xoffset = imageX/2 - dcx;
    double yoffset = imageY/2 - dcy;

    for (std::_List_const_iterator<Line2D> it = lines.begin(); it != lines.end(); ++it){
        Line2D line = *it;

        line.p1.x *= d;
        line.p1.y *= d;

        line.p2.x *= d;
        line.p2.y *= d;

        line.p1.x += xoffset;
        line.p1.y += yoffset;

        line.p2.x += xoffset;
        line.p2.y += yoffset;

        line.Draw(image);
    }

    return image;
}

void Line2D::DrawZ(img::EasyImage &image, ZBuffer& zbuf) {
    int x1 = std::round(p1.x);
    int y1 = std::round(p1.y);
    int x2 = std::round(p2.x);
    int y2 = std::round(p2.y);

    drawLineZ(image, x1, y1, z1, x2,y2, z2, zbuf, color);
}

img::EasyImage Line2D::draw2DLinesZ(Lines2D &lines, const int size, Color &backgroundcolor) {
    double xmin = INFINITY;
    double xmax = -INFINITY;
    double ymin = INFINITY;
    double ymax = -INFINITY;


    for (std::_List_const_iterator<Line2D> it = lines.begin(); it != lines.end(); ++it){
        Utils::setIflessOrMore(xmin, xmax, it->p1.x);
        Utils::setIflessOrMore(ymin, ymax, it->p1.y);

        Utils::setIflessOrMore(xmin, xmax, it->p2.x);
        Utils::setIflessOrMore(ymin, ymax, it->p2.y);
    }

    double xrange = xmax - xmin;
    double yrange = ymax - ymin;
    double maxrange = std::max(xrange, yrange);

    double imageX = size * xrange / maxrange;
    double imageY = size * yrange / maxrange;

    img::EasyImage image(std::round(imageX), std::round(imageY));
    Utils::fillColor(image,backgroundcolor);

    double d = 0.95 * imageX / xrange;

    double dcx = d*(xmin + xmax)/2;
    double dcy = d*(ymin + ymax)/2;

    double xoffset = imageX/2 - dcx;
    double yoffset = imageY/2 - dcy;

    ZBuffer zbuf = ZBuffer(imageX,imageY);

    for (std::_List_const_iterator<Line2D> it = lines.begin(); it != lines.end(); ++it){
        Line2D line = *it;

        line.p1.x *= d;
        line.p1.y *= d;

        line.p2.x *= d;
        line.p2.y *= d;

        line.p1.x += xoffset;
        line.p1.y += yoffset;

        line.p2.x += xoffset;
        line.p2.y += yoffset;

        line.DrawZ(image, zbuf);
    }

    return image;
}

void Line2D::drawLineZ(img::EasyImage &image, int &x1, int &y1, double z1, int &x2, int &y2, double z2, ZBuffer &zbuf, Color &color) {
    img::Color clr = img::Color(color.r,color.g,color.b);
    return image.draw_lineZ(x1,y1,z1,x2,y2,z2,zbuf,clr);
}

Matrix TransformMatrices::scaleFigure(const double scale) {
    Matrix mat;

    mat(1,1) = scale;
    mat(2,2) = scale;
    mat(3,3) = scale;

    return mat;
}

Matrix TransformMatrices::rotX(const double angle) {
    Matrix mat;

    double cs = cos(angle);
    double sn = sin(angle);

    mat(2,2) = cs;
    mat(3,3) = cs;
    mat(2,3) = sn;
    mat(3,2) = -sn;

    return mat;
}

Matrix TransformMatrices::rotY(const double angle) {
    Matrix mat;

    double cs = cos(angle);
    double sn = sin(angle);

    mat(1,1) = cs;
    mat(3,3) = cs;
    mat(3,1) = sn;
    mat(1,3) = -sn;

    return mat;
}

Matrix TransformMatrices::rotZ(const double angle) {
    Matrix mat;

    double cs = cos(angle);
    double sn = sin(angle);

    mat(1,1) = cs;
    mat(2,2) = cs;
    mat(1,2) = sn;
    mat(2,1) = -sn;

    return mat;
}

Matrix TransformMatrices::translate(const Vector3D &vector) {
    Matrix mat;

    mat(4,1) = vector.x;
    mat(4,2) = vector.y;
    mat(4,3) = vector.z;

    return mat;
}

void TransformMatrices::toPolar(const Vector3D &point, double &theta, double &phi, double &r){
    r = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    theta = std::atan2(point.y, point.x);
    phi = std::acos(point.z / r);
}

Matrix TransformMatrices::eyePointTrans(const Vector3D &eyepoint) {
    Matrix mat;
    double theta;
    double phi;
    double r;

    toPolar(eyepoint,theta,phi,r);

    mat(1,1) = -sin(theta);
    mat(1,2) = -cos(theta) * cos(phi);
    mat(1,3) = cos(theta)*sin(phi);

    mat(2,1) = cos(theta);
    mat(2,2) = -sin(theta) * cos(phi);
    mat(2,3) = sin(theta) * sin(phi);

    mat(3,2) = sin(phi);
    mat(3,3) = cos(phi);

    mat(4,3) = -r;

    return mat;
}


void Figure::applyTransformation(const Matrix &mat) {
    int nrPoints = points.size();
    for (int i=0; i < nrPoints; i++){
        Vector3D newP = points[i] * mat;
        points[i] = newP;
    }
}

void Figure::triangulate() {
    int size = faces.size();

    for (int i=0; i<size; i++){
        Face face = faces[i];

        for (int j=1; j<face.points.size() - 1; j++){
            std::vector<int> pts;

            pts.push_back(face.points[0]);
            pts.push_back(face.points[j]);
            pts.push_back(face.points[j+1]);

            faces.push_back(Face(pts));
        }
    }

    faces.erase(faces.begin(), faces.begin()+size);
}

Figure::Figure(const Figure &to_copy) {
    points = to_copy.points;
    color = to_copy.color;
    diffuseReflection = to_copy.diffuseReflection;
    specularReflection = to_copy.specularReflection;
    reflectionCoefficient = to_copy.reflectionCoefficient;
    for (int i=0; i<to_copy.faces.size(); i++){
        Face face = to_copy.faces[i];

        std::vector<int> pts = face.points;
        faces.push_back(Face(pts));
    }
}
