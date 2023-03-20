//
// Created by viktor on 30.05.20.
//

#include "Lighting.h"
#include "Wireframe.h"
#include "ZBufferedWireframe.h"
#include "ZBufferingTri.h"

img::EasyImage Lighting::parseConfig(std::string &type, const ini::Configuration &conf) {
    int size = conf["General"]["size"].as_int_or_die();
    std::vector<double> eye = conf["General"]["eye"].as_double_tuple_or_die();
    std::vector<double> bcol = conf["General"]["backgroundcolor"].as_double_tuple_or_die();

    Color backgroundcolor = Color(bcol);

    int nrfigures = conf["General"]["nrFigures"].as_int_or_die();
    int nrlights = conf["General"]["nrLights"].as_int_or_die();

    Figures3D figures;
    Lights3D lights;

    for (int i=0; i < nrfigures; i++){
        std::string figurename = "Figure" + std::to_string(i);

        if(!Wireframe::parseFigure(figurename, conf, figures, true)){
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

    for (int i=0; i < nrlights; i++){
        std::string name = "Light" + std::to_string(i);

        parseLight(name, conf, lights, eyepmat);

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

            ZBufferingTri::draw_zbuf_tri(image,zbuf,A,B,C,d,xoffset,yoffset, it->color, true, it->diffuseReflection, it->specularReflection, it->reflectionCoefficient, lights);
        }
    }

    return image;
}

void Lighting::applyLight(Light &light, Color &color, Color& diffuseReflection, Color& specularReflection, double& reflectionCoeff, Color &final, Vector3D& normal, int& x, int& y, double& dx, double& dy, double& d, double zr) {
    Vector3D l;
    double px = (x-dx)*(-zr)/d;
    double py = (y-dy)*(-zr)/d;
    double pz = zr;
    Vector3D p = Vector3D().point(px,py,pz);

    double cos;

    if (!light.inf){
        Color& difl = light.diffuseLight;

        l = light.location - p;
        l.normalise();

        double cosas = std::cos(light.angle);

        cos = normal.x * l.x + normal.y * l.y + normal.z * l.z;

        double intens = 1.0 - ((1.0 - cos) / (1.0 - cosas));

        if (cos <= cosas){
            intens = 0;
        }

        double rd = difl.R * diffuseReflection.R * intens;
        double gd = difl.G * diffuseReflection.G * intens;
        double bd = difl.B * diffuseReflection.B * intens;
        final.R += rd;
        final.G += gd;
        final.B += bd;
    }else{
        l = -1* light.ldVector;
        l.normalise();
        cos = normal.x * l.x + normal.y * l.y + normal.z * l.z;
    }

    Color& specl = light.specularLight;

    Vector3D r = (2*cos*normal) - l;
    r.normalise();

    Vector3D cam = Vector3D::point(0, 0, 0) - p;
    cam.normalise();

    double cB = r.x * cam.x + r.y * cam.y + r.z * cam.z;;
    if (cB < 0){
        cB = 0;
    }

    double specint = pow(cB,reflectionCoeff);

    double rs = specl.R * specularReflection.R * specint;
    double gs = specl.G * specularReflection.G * specint;
    double bs = specl.B * specularReflection.B * specint;
    final.R += rs;
    final.G += gs;
    final.B += bs;
}

void Lighting::applyLightTri(Light &light, Color &color, Color &diffuseReflection, Color &final, Vector3D & normal) {
    Color& ambl = light.ambientLight;
    double r = ambl.R * color.R;
    double g = ambl.G * color.G;
    double b = ambl.B * color.B;
    final.R += r;
    final.G += g;
    final.B += b;
    //diffuse
    if (light.inf){
        Color& difl = light.diffuseLight;
        Vector3D l = -1* light.ldVector;
        l.normalise();
        double cos = normal.x * l.x + normal.y * l.y + normal.z * l.z;

        if (cos<0){cos=0;}
        double rd = difl.R * diffuseReflection.R * cos;
        double gd = difl.G * diffuseReflection.G * cos;
        double bd = difl.B * diffuseReflection.B * cos;
        final.R += rd;
        final.G += gd;
        final.B += bd;
    }
}

void Lighting::parseLight(std::string &index, const ini::Configuration &conf, Lights3D &lights, Matrix& eyepmat) {
    std::vector<double> ambient = conf[index]["ambientLight"].as_double_tuple_or_die();
    Color amb = Color(ambient);
    Light l;
    l.ambientLight = amb;


    std::vector<double> difuse;
    if(conf[index]["diffuseLight"].as_double_tuple_if_exists(difuse)){
        Color dif = Color(difuse);
        l.diffuseLight = dif;
        if (conf[index]["infinity"].as_bool_or_die()){
            std::vector<double> dir = conf[index]["direction"].as_double_tuple_or_die();
            l.ldVector = Vector3D::vector(dir[0],dir[1], dir[2]);
            l.ldVector *= eyepmat;
        }else{
            std::vector<double> dir = conf[index]["location"].as_double_tuple_or_die();
            l.location = Vector3D::point(dir[0],dir[1], dir[2]);
            l.location *= eyepmat;
            l.angle = Utils::degTorad(conf[index]["spotAngle"].as_double_or_default(90));
            l.inf = false;
        }
    }

    std::vector<double> specular;
    if(conf[index]["specularLight"].as_double_tuple_if_exists(specular)){
        Color spec = Color(specular);
        l.specularLight = spec;
        if (conf[index]["infinity"].as_bool_or_die()){
            std::vector<double> dir = conf[index]["direction"].as_double_tuple_or_die();
            l.ldVector = Vector3D::vector(dir[0],dir[1], dir[2]);
            l.ldVector *= eyepmat;
        }else{
            std::vector<double> dir = conf[index]["location"].as_double_tuple_or_die();
            l.location = Vector3D::point(dir[0],dir[1], dir[2]);
            l.location *= eyepmat;
            l.angle = Utils::degTorad(conf[index]["spotAngle"].as_double_or_default(90));
            l.inf = false;
        }
    }

    lights.push_back(l);
}