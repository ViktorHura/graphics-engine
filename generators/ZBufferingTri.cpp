//
// Created by viktor on 29.05.20.
//

#include "ZBufferingTri.h"
#include "Wireframe.h"
#include "ZBufferedWireframe.h"

img::EasyImage ZBufferingTri::parseConfig(std::string &type, const ini::Configuration &conf) {
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

    for (Figures3D::iterator it = figures.begin(); it != figures.end(); it++){
        it->triangulate();
    }

    Vector3D eyepoint = Vector3D::point(eye[0], eye[1], eye[2]);
    Matrix eyepmat= TransformMatrices::eyePointTrans(eyepoint);

    for (Figures3D::iterator it = figures.begin(); it != figures.end(); it++){
        it->applyTransformation(eyepmat);
    }

    Lines2D lines = ZBufferedWireframe::doProjection(figures);

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

    img::EasyImage image = img::EasyImage(std::round(imageX),std::round(imageY));
    Utils::fillColor(image,backgroundcolor);

    ZBuffer zbuf = ZBuffer(std::round(imageX),std::round(imageY));

    double d = 0.95 * imageX / xrange;

    double dcx = d*(xmin + xmax)/2;
    double dcy = d*(ymin + ymax)/2;

    double xoffset = imageX/2 - dcx;
    double yoffset = imageY/2 - dcy;

    for (Figures3D::iterator it = figures.begin(); it != figures.end(); it++){
        for (int i=0; i < it->faces.size(); i++){
            Face face = it->faces[i];
            Vector3D A = it->points[face.points[0]];
            Vector3D B = it->points[face.points[1]];
            Vector3D C = it->points[face.points[2]];

            /*std::vector<double> clr;
            clr.push_back(((double) rand() / (RAND_MAX)));
            clr.push_back(((double) rand() / (RAND_MAX)));
            clr.push_back(((double) rand() / (RAND_MAX)));
            Color col(clr);*/

            draw_zbuf_tri(image,zbuf,A,B,C,d,xoffset,yoffset, it->color);
        }
    }

    return image;
}

void ZBufferingTri::draw_zbuf_tri(img::EasyImage &image, ZBuffer &buf, Vector3D &A, Vector3D &B, Vector3D &C, double &d, double &dx, double &dy, Color &color, bool lighting, Color diffuseReflection, Color specularReflection, double reflectionCoeff, std::list<Light> lights) {
    double az;
    double bz;
    double cz;
    Point2D a = ZBufferedWireframe::projectPoint(A, d,az, dx, dy);
    Point2D b = ZBufferedWireframe::projectPoint(B, d,bz, dx, dy);
    Point2D c = ZBufferedWireframe::projectPoint(C, d,cz, dx, dy);

    Vector3D g;
    g.x = (a.x + b.x + c.x)/3.0;
    g.y = (a.y + b.y + c.y)/3.0;
    g.z = (1.0/(3.0*A.z)) + (1.0/(3.0*B.z)) + (1.0/(3.0*C.z));

    Vector3D u = B-A;
    Vector3D v = C-A;

    Vector3D normal = Vector3D::cross(u,v);

    double w1 = normal.x;
    double w2 = normal.y;
    double w3 = normal.z;
    double k = w1 * A.x + w2 * A.y + w3 * A.z;
    //if (k >= 0){
    //    return;
    //}
    double dzdx = w1 / (-d * k);
    double dzdy = w2 / (-d * k);

    normal.normalise();

    int ymin = std::round(min(a.y,b.y,c.y) + 0.5);
    int ymax = std::round(max(a.y,b.y,c.y) - 0.5);

    Color tricol = Color();

    for (Lights3D::iterator it = lights.begin(); it != lights.end(); it++){
        Lighting::applyLightTri(*it,color,diffuseReflection,tricol, normal);
    }

    for (int i=ymin; i < ymax+1; i++){
        double xlab = +INFINITY;
        double xlac = +INFINITY;
        double xlbc = +INFINITY;

        double xrab = -INFINITY;
        double xrac = -INFINITY;
        double xrbc = -INFINITY;

        calcLeftandRight(i, a,b,c, xlab,xlac,xlbc,xrab,xrac,xrbc);

        int xl = std::round(min(xlab,xlac,xlbc)+0.5);
        int xr = std::round(max(xrab,xrac,xrbc)-0.5);

        for (int j=xl; j < xr+1; j++){
            double z = (1.0001 * g.z) + (j - g.x)*dzdx + (i-g.y)*dzdy;
            double zinv = 1.0 / z;

            if (z < buf.at(j,i)){
                buf.set(j,i,z);
                if (lighting){
                    Color final = Color(tricol);

                    for (Lights3D::iterator it = lights.begin(); it != lights.end(); it++){
                        Lighting::applyLight(*it,color,diffuseReflection,specularReflection,reflectionCoeff,final, normal, j,i,dx,dy,d, zinv);
                    }

                    final.updateInts();
                    Utils::colorPixel(image, j, i, final);
                }else {
                    Utils::colorPixel(image, j, i, color);
                }
            }
        }

    }
}

double ZBufferingTri::min(double &y1, double &y2, double &y3) {
    double min = y1;

    if (y2 < min){
        min = y2;
    }
    if (y3 < min){
        min = y3;
    }

    return min;
}

double ZBufferingTri::max(double &y1, double &y2, double &y3) {
    double max = y1;

    if (y2 > max){
        max = y2;
    }
    if (y3 > max){
        max = y3;
    }

    return max;
}

bool crosses(int& i, Point2D& p, Point2D& q){
    return (p.y != q.y && ( ((double) i - p.y ) * ((double) i - q.y ) <= 0));
}

double Xi(int& i, Point2D& p, Point2D& q){
    return q.x + ( p.x - q.x )*( (double)i - q.y )/( p.y - q.y );
}

void ZBufferingTri::calcLeftandRight(int& i, Point2D& a, Point2D& b, Point2D& c, double& xlab, double& xlac, double& xlbc, double& xrab, double& xrac, double& xrbc) {
    if (crosses(i,a,b)){//ab
        double xi = Xi(i,a,b);
        Utils::setIflessOrMore(xlab,xrab, xi);
    }

    if (crosses(i,a,c)){//ac
        double xi = Xi(i,a,c);
        Utils::setIflessOrMore(xlac,xrac, xi);
    }

    if (crosses(i,b,c)){//bc
        double xi = Xi(i,b,c);
        Utils::setIflessOrMore(xlbc,xrbc, xi);
    }
}
