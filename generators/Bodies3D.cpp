//
// Created by viktor on 08.05.20.
//

#include "Bodies3D.h"

Figure Bodies3D::Cube(Color &color) {
    std::vector<Vector3D> points;
    points.push_back(Vector3D::point(1,-1,-1));
    points.push_back(Vector3D::point(-1,1,-1));
    points.push_back(Vector3D::point(1,1,1));
    points.push_back(Vector3D::point(-1,-1,1));
    points.push_back(Vector3D::point(1,1,-1));
    points.push_back(Vector3D::point(-1,-1,-1));
    points.push_back(Vector3D::point(1,-1,1));
    points.push_back(Vector3D::point(-1,1,1));

    std::vector<Face> faces;

    std::vector<int> f1ptn;
    f1ptn.push_back(0);
    f1ptn.push_back(4);
    f1ptn.push_back(2);
    f1ptn.push_back(6);
    faces.push_back(Face(f1ptn));

    std::vector<int> f2ptn;
    f2ptn.push_back(4);
    f2ptn.push_back(1);
    f2ptn.push_back(7);
    f2ptn.push_back(2);
    faces.push_back(Face(f2ptn));

    std::vector<int> f3ptn;
    f3ptn.push_back(1);
    f3ptn.push_back(5);
    f3ptn.push_back(3);
    f3ptn.push_back(7);
    faces.push_back(Face(f3ptn));

    std::vector<int> f4ptn;
    f4ptn.push_back(5);
    f4ptn.push_back(0);
    f4ptn.push_back(6);
    f4ptn.push_back(3);
    faces.push_back(Face(f4ptn));

    std::vector<int> f5ptn;
    f5ptn.push_back(6);
    f5ptn.push_back(2);
    f5ptn.push_back(7);
    f5ptn.push_back(3);
    faces.push_back(Face(f5ptn));

    std::vector<int> f6ptn;
    f6ptn.push_back(0);
    f6ptn.push_back(5);
    f6ptn.push_back(1);
    f6ptn.push_back(4);
    faces.push_back(Face(f6ptn));

    return Figure(points,faces,color);
}

Figure Bodies3D::Tetrahedron(Color &color) {
    std::vector<Vector3D> points;
    points.push_back(Vector3D::point(1,-1,-1));
    points.push_back(Vector3D::point(-1,1,-1));
    points.push_back(Vector3D::point(1,1,1));
    points.push_back(Vector3D::point(-1,-1,1));

    std::vector<Face> faces;

    std::vector<int> f1ptn;
    f1ptn.push_back(0);
    f1ptn.push_back(1);
    f1ptn.push_back(2);
    faces.push_back(Face(f1ptn));

    std::vector<int> f2ptn;
    f2ptn.push_back(1);
    f2ptn.push_back(3);
    f2ptn.push_back(2);
    faces.push_back(Face(f2ptn));

    std::vector<int> f3ptn;
    f3ptn.push_back(0);
    f3ptn.push_back(3);
    f3ptn.push_back(1);
    faces.push_back(Face(f3ptn));

    std::vector<int> f4ptn;
    f4ptn.push_back(0);
    f4ptn.push_back(2);
    f4ptn.push_back(3);
    faces.push_back(Face(f4ptn));

    return Figure(points,faces,color);
}

Figure Bodies3D::Octahedron(Color &color) {
    std::vector<Vector3D> points;
    points.push_back(Vector3D::point(1,0,0));
    points.push_back(Vector3D::point(0,1,0));
    points.push_back(Vector3D::point(-1,0,0));
    points.push_back(Vector3D::point(0,-1,0));
    points.push_back(Vector3D::point(0,0,-1));
    points.push_back(Vector3D::point(0,0,1));

    std::vector<Face> faces;

    std::vector<int> f1ptn;
    f1ptn.push_back(0);
    f1ptn.push_back(1);
    f1ptn.push_back(5);
    faces.push_back(Face(f1ptn));

    std::vector<int> f2ptn;
    f2ptn.push_back(1);
    f2ptn.push_back(2);
    f2ptn.push_back(5);
    faces.push_back(Face(f2ptn));

    std::vector<int> f3ptn;
    f3ptn.push_back(2);
    f3ptn.push_back(3);
    f3ptn.push_back(5);
    faces.push_back(Face(f3ptn));

    std::vector<int> f4ptn;
    f4ptn.push_back(3);
    f4ptn.push_back(0);
    f4ptn.push_back(5);
    faces.push_back(Face(f4ptn));

    std::vector<int> f5ptn;
    f5ptn.push_back(1);
    f5ptn.push_back(0);
    f5ptn.push_back(4);
    faces.push_back(Face(f5ptn));

    std::vector<int> f6ptn;
    f6ptn.push_back(2);
    f6ptn.push_back(1);
    f6ptn.push_back(4);
    faces.push_back(Face(f6ptn));

    std::vector<int> f7ptn;
    f7ptn.push_back(3);
    f7ptn.push_back(2);
    f7ptn.push_back(4);
    faces.push_back(Face(f7ptn));

    std::vector<int> f8ptn;
    f8ptn.push_back(0);
    f8ptn.push_back(3);
    f8ptn.push_back(4);
    faces.push_back(Face(f8ptn));

    return Figure(points,faces,color);
}

Figure Bodies3D::Icosahedron(Color &color) {
    std::vector<Vector3D> points;

    double q = sqrt(5) / 2;

    points.push_back(Vector3D::point(0,0,q));

    for (int i=0; i < 5; i++){
        double p = i* 2*M_PI / 5;
        points.push_back(Vector3D::point(cos(p),sin(p), 0.5));
    }

    for (int i=0; i < 5; i++){
        double p = M_PI / 5 + i*2*M_PI / 5;
        points.push_back(Vector3D::point(cos(p),sin(p), -0.5));
    }

    points.push_back(Vector3D::point(0,0,-q));

    std::vector<Face> faces;

    std::vector<int> f1ptn;
    f1ptn.push_back(0);
    f1ptn.push_back(1);
    f1ptn.push_back(2);
    faces.push_back(Face(f1ptn));

    std::vector<int> f2ptn;
    f2ptn.push_back(0);
    f2ptn.push_back(2);
    f2ptn.push_back(3);
    faces.push_back(Face(f2ptn));

    std::vector<int> f3ptn;
    f3ptn.push_back(0);
    f3ptn.push_back(3);
    f3ptn.push_back(4);
    faces.push_back(Face(f3ptn));

    std::vector<int> f4ptn;
    f4ptn.push_back(0);
    f4ptn.push_back(4);
    f4ptn.push_back(5);
    faces.push_back(Face(f4ptn));

    std::vector<int> f5ptn;
    f5ptn.push_back(0);
    f5ptn.push_back(5);
    f5ptn.push_back(1);
    faces.push_back(Face(f5ptn));

    std::vector<int> f6ptn;
    f6ptn.push_back(1);
    f6ptn.push_back(6);
    f6ptn.push_back(2);
    faces.push_back(Face(f6ptn));

    std::vector<int> f7ptn;
    f7ptn.push_back(2);
    f7ptn.push_back(6);
    f7ptn.push_back(7);
    faces.push_back(Face(f7ptn));

    std::vector<int> f8ptn;
    f8ptn.push_back(2);
    f8ptn.push_back(7);
    f8ptn.push_back(3);
    faces.push_back(Face(f8ptn));

    std::vector<int> f9ptn;
    f9ptn.push_back(3);
    f9ptn.push_back(7);
    f9ptn.push_back(8);
    faces.push_back(Face(f9ptn));

    std::vector<int> f10ptn;
    f10ptn.push_back(3);
    f10ptn.push_back(8);
    f10ptn.push_back(4);
    faces.push_back(Face(f10ptn));

    std::vector<int> f11ptn;
    f11ptn.push_back(4);
    f11ptn.push_back(8);
    f11ptn.push_back(9);
    faces.push_back(Face(f11ptn));

    std::vector<int> f12ptn;
    f12ptn.push_back(4);
    f12ptn.push_back(9);
    f12ptn.push_back(5);
    faces.push_back(Face(f12ptn));

    std::vector<int> f13ptn;
    f13ptn.push_back(5);
    f13ptn.push_back(9);
    f13ptn.push_back(10);
    faces.push_back(Face(f13ptn));

    std::vector<int> f14ptn;
    f14ptn.push_back(5);
    f14ptn.push_back(10);
    f14ptn.push_back(1);
    faces.push_back(Face(f14ptn));

    std::vector<int> f15ptn;
    f15ptn.push_back(1);
    f15ptn.push_back(10);
    f15ptn.push_back(6);
    faces.push_back(Face(f15ptn));

    std::vector<int> f16ptn;
    f16ptn.push_back(11);
    f16ptn.push_back(7);
    f16ptn.push_back(6);
    faces.push_back(Face(f16ptn));

    std::vector<int> f17ptn;
    f17ptn.push_back(11);
    f17ptn.push_back(8);
    f17ptn.push_back(7);
    faces.push_back(Face(f17ptn));

    std::vector<int> f18ptn;
    f18ptn.push_back(11);
    f18ptn.push_back(9);
    f18ptn.push_back(8);
    faces.push_back(Face(f18ptn));

    std::vector<int> f19ptn;
    f19ptn.push_back(11);
    f19ptn.push_back(10);
    f19ptn.push_back(9);
    faces.push_back(Face(f19ptn));

    std::vector<int> f20ptn;
    f20ptn.push_back(11);
    f20ptn.push_back(6);
    f20ptn.push_back(10);
    faces.push_back(Face(f20ptn));

    return Figure(points,faces,color);
}

Figure Bodies3D::Dodecahedron(Color &color) {

    Figure ico = Icosahedron(color);

    std::vector<Vector3D> points;

    for (int i=0; i < ico.faces.size(); i++){
        Vector3D p1 = ico.points[ico.faces[i].points[0]];
        Vector3D p2 = ico.points[ico.faces[i].points[1]];
        Vector3D p3 = ico.points[ico.faces[i].points[2]];

        Vector3D average = Utils::averagePoint(p1,p2,p3);

        points.push_back(average);
    }

    std::vector<Face> faces;

    std::vector<int> f1ptn;
    f1ptn.push_back(0);
    f1ptn.push_back(1);
    f1ptn.push_back(2);
    f1ptn.push_back(3);
    f1ptn.push_back(4);
    faces.push_back(Face(f1ptn));

    std::vector<int> f2ptn;
    f2ptn.push_back(0);
    f2ptn.push_back(5);
    f2ptn.push_back(6);
    f2ptn.push_back(7);
    f2ptn.push_back(1);
    faces.push_back(Face(f2ptn));

    std::vector<int> f3ptn;
    f3ptn.push_back(1);
    f3ptn.push_back(7);
    f3ptn.push_back(8);
    f3ptn.push_back(9);
    f3ptn.push_back(2);
    faces.push_back(Face(f3ptn));

    std::vector<int> f4ptn;
    f4ptn.push_back(2);
    f4ptn.push_back(9);
    f4ptn.push_back(10);
    f4ptn.push_back(11);
    f4ptn.push_back(3);
    faces.push_back(Face(f4ptn));

    std::vector<int> f5ptn;
    f5ptn.push_back(3);
    f5ptn.push_back(11);
    f5ptn.push_back(12);
    f5ptn.push_back(13);
    f5ptn.push_back(4);
    faces.push_back(Face(f5ptn));

    std::vector<int> f6ptn;
    f6ptn.push_back(4);
    f6ptn.push_back(13);
    f6ptn.push_back(14);
    f6ptn.push_back(5);
    f6ptn.push_back(0);
    faces.push_back(Face(f6ptn));

    std::vector<int> f7ptn;
    f7ptn.push_back(19);
    f7ptn.push_back(18);
    f7ptn.push_back(17);
    f7ptn.push_back(16);
    f7ptn.push_back(15);
    faces.push_back(Face(f7ptn));

    std::vector<int> f8ptn;
    f8ptn.push_back(19);
    f8ptn.push_back(14);
    f8ptn.push_back(13);
    f8ptn.push_back(12);
    f8ptn.push_back(18);
    faces.push_back(Face(f8ptn));

    std::vector<int> f9ptn;
    f9ptn.push_back(18);
    f9ptn.push_back(12);
    f9ptn.push_back(11);
    f9ptn.push_back(10);
    f9ptn.push_back(17);
    faces.push_back(Face(f9ptn));

    std::vector<int> f10ptn;
    f10ptn.push_back(17);
    f10ptn.push_back(10);
    f10ptn.push_back(9);
    f10ptn.push_back(8);
    f10ptn.push_back(16);
    faces.push_back(Face(f10ptn));

    std::vector<int> f11ptn;
    f11ptn.push_back(16);
    f11ptn.push_back(8);
    f11ptn.push_back(7);
    f11ptn.push_back(6);
    f11ptn.push_back(15);
    faces.push_back(Face(f11ptn));

    std::vector<int> f12ptn;
    f12ptn.push_back(15);
    f12ptn.push_back(6);
    f12ptn.push_back(5);
    f12ptn.push_back(14);
    f12ptn.push_back(19);
    faces.push_back(Face(f12ptn));

    return Figure(points,faces,color);
}

Figure Bodies3D::Sphere(int n, Color &color) {
    Figure ico = Icosahedron(color);

    for (int i=0; i<n; i++){
        std::vector<Vector3D> points;
        std::vector<Face> faces;

        for (int j=0; j<ico.faces.size(); j++){
            Face oldface = ico.faces[j];

            Vector3D A = ico.points[oldface.points[0]];

            Vector3D B = ico.points[oldface.points[1]];

            Vector3D C = ico.points[oldface.points[2]];

            Vector3D D = Utils::averagePoint(A, B);

            Vector3D F = Utils::averagePoint(B, C);

            Vector3D E = Utils::averagePoint(C, A);

            points.push_back(A); //A
            points.push_back(D); // D
            points.push_back(B); // B
            points.push_back(F); //F
            points.push_back(C); // C
            points.push_back(E); //E

            int pindex = j*6;

            std::vector<int> f1ptn; //ADE
            f1ptn.push_back(pindex);
            f1ptn.push_back(pindex + 1);
            f1ptn.push_back(pindex + 5);
            faces.push_back(Face(f1ptn));

            std::vector<int> f4ptn; //BFD
            f4ptn.push_back(pindex + 2);
            f4ptn.push_back(pindex + 3);
            f4ptn.push_back(pindex + 1);
            faces.push_back(Face(f4ptn));

            std::vector<int> f2ptn; //CEF
            f2ptn.push_back(pindex + 4);
            f2ptn.push_back(pindex + 5);
            f2ptn.push_back(pindex + 3);
            faces.push_back(Face(f2ptn));

            std::vector<int> f3ptn; //DFE
            f3ptn.push_back(pindex + 1);
            f3ptn.push_back(pindex + 3);
            f3ptn.push_back(pindex + 5);
            faces.push_back(Face(f3ptn));

        }

        ico = Figure(points,faces,color);
    }

    for (int i =0; i<ico.points.size(); i++){
        ico.points[i].normalise();
    }

    return ico;
}

Figure Bodies3D::Cone(int n, double h, Color &color) {
    std::vector<Vector3D> points;
    std::vector<Face> faces;

    points.push_back(Vector3D::point(0,0, h));
    for (int i=0; i<= n;i++){
        double q = 2*i*M_PI/n;
        points.push_back(Vector3D::point(cos(q),sin(q), 0));
    }

    int size = points.size();
    for (int i=1; i< size;i++){
        std::vector<int> fptn;
        fptn.push_back((i+1)%size);
        fptn.push_back(0);
        fptn.push_back(i%size);
        faces.push_back(Face(fptn));
    }

    std::vector<int> fbptn;
    for (int i=0; i <n;i++){
        fbptn.push_back(i);
    }
    faces.push_back(Face(fbptn));

    return Figure(points,faces,color);
}

Figure Bodies3D::Cylinder(int n, double h, Color &color, bool tube) {
    std::vector<Vector3D> points;
    std::vector<Face> faces;

    for (int i=0; i< n;i++){
        double q = 2*i*M_PI/n;
        points.push_back(Vector3D::point(cos(q),sin(q), 0));
    }
    for (int i=0; i< n;i++){
        double q = 2*i*M_PI/n;
        points.push_back(Vector3D::point(cos(q),sin(q), h));
    }

    for (int i=0; i< n;i++){
        std::vector<int> fptn;
        fptn.push_back(i);
        fptn.push_back((i+1)%n);
        fptn.push_back(n + (i+1)%n);
        fptn.push_back(n + i);
        faces.push_back(Face(fptn));
    }

    if (!tube) {
        std::vector<int> fbptn;
        for (int i = 0; i < n; i++) {
            fbptn.push_back(i);
        }
        faces.push_back(Face(fbptn));

        std::vector<int> fuptn;
        for (int i = n; i < 2 * n; i++) {
            fuptn.push_back(i);
        }
        faces.push_back(Face(fuptn));
    }

    return Figure(points,faces,color);
}

int torusIndex(int i, int j, int m){
    return i * m + j;
}

Figure Bodies3D::Torus(double R, double r, int n, int m, Color &color) {
    std::vector<Vector3D> points;
    std::vector<Face> faces;

    for (int i=0; i < n; i++){
        for (int j=0; j<m; j++){
            double u = 2*i*M_PI / n;
            double v = 2*j*M_PI / m;

            points.push_back(Vector3D::point( (R+r*cos(v))*cos(u) , (R+r*cos(v))*sin(u) , r*sin(v) ));
        }
    }

    for (int i=0; i < n; i++){
        for (int j=0; j<m; j++){
            std::vector<int> fptn;
            fptn.push_back(torusIndex(i,j,m));
            fptn.push_back(torusIndex((i+1)%n,j,m));
            fptn.push_back(torusIndex((i+1)%n,(j+1)%m,m));
            fptn.push_back(torusIndex(i,(j+1)%m,m));
            faces.push_back(Face(fptn));
        }
    }

    return Figure(points,faces,color);
}

void Bodies3D::generateThickFigures(Figure &figure, Figures3D &figures, const double r, const int n, const int m) {
    Color color = figure.color;
    Color diffuseReflection = figure.diffuseReflection;
    Color specularReflection = figure.specularReflection;
    double reflectionCoefficient = figure.reflectionCoefficient;

    Matrix scale = TransformMatrices::scaleFigure(r);

    for(int i=0; i<figure.points.size(); i++)
    {
        Figure sphere = Sphere(m,color);
        sphere.applyTransformation(scale);

        Vector3D translate = figure.points[i] - Vector3D::point(0,0,0);
        Matrix transl = TransformMatrices::translate(translate);
        sphere.applyTransformation(transl);

        sphere.diffuseReflection = diffuseReflection;
        sphere.specularReflection = specularReflection;
        sphere.reflectionCoefficient = reflectionCoefficient;
        figures.push_back(sphere);
    }
    for(int i=0; i<figure.faces.size(); i++)
    {
        Face face = figure.faces[i];

        for(int j = 0; j < face.points.size(); j++)
        {
            Vector3D p1 = figure.points[face.points[j]];
            Vector3D p2;
            if (j == face.points.size()-1){
                p2 = figure.points[face.points[0]];
            }else{
                p2 = figure.points[face.points[j+1]];
            }

            double h = Vector3D::vector(p2-p1).length()/(double)r;

            Figure cylinder = Cylinder(n, h, color, true);
            cylinder.diffuseReflection = diffuseReflection;
            cylinder.specularReflection = specularReflection;
            cylinder.reflectionCoefficient = reflectionCoefficient;

            cylinder.applyTransformation(scale);

            Vector3D pp = Vector3D::vector(p2-p1);
            Vector3D pr = Vector3D::point(0,0,0) + pp;
            double r;
            double phi;
            double theta;

            TransformMatrices::toPolar(pr,theta,phi,r);

            Matrix rotY = TransformMatrices::rotY(phi);
            Matrix rotZ = TransformMatrices::rotZ(theta);
            Matrix trans = TransformMatrices::translate(p1);

            cylinder.applyTransformation(rotY);
            cylinder.applyTransformation(rotZ);
            cylinder.applyTransformation(trans);

            figures.push_back(cylinder);
        }
    }
}
