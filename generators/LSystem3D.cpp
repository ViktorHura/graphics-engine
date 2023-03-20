//
// Created by viktor on 11.05.20.
//

#include <stack>
#include "LSystem3D.h"

void parseChar(char& c,const LParser::LSystem3D &lsystem, std::vector<Vector3D>& points, std::vector<Face>& faces, Vector3D& pos, Vector3D& posprev, Vector3D& h, Vector3D& l, Vector3D& u,const double& angle, std::stack<std::vector<Vector3D>>& stack){
    if (c == '+'){
        Vector3D hn = h*cos(angle) + l*sin(angle);
        Vector3D ln = -h*sin(angle) + l*cos(angle);
        h = hn;
        l = ln;
    }else if (c == '-'){
        Vector3D hn = h*cos(-angle) + l*sin(-angle);
        Vector3D ln = -h*sin(-angle) + l*cos(-angle);
        h = hn;
        l = ln;
    }else if (c == '^'){
        Vector3D hn = h*cos(angle) + u*sin(angle);
        Vector3D un = -h*sin(angle) + u*cos(angle);
        h = hn;
        u = un;
    }else if (c == '&'){
        Vector3D hn = h*cos(-angle) + u*sin(-angle);
        Vector3D un = -h*sin(-angle) + u*cos(-angle);
        h = hn;
        u = un;
    }else if (c == '\\'){
        Vector3D ln = l*cos(angle) - u*sin(angle);
        Vector3D un = l*sin(angle) + u*cos(angle);
        l = ln;
        u = un;
    }else if (c == '/'){
        Vector3D ln = l*cos(-angle) - u*sin(-angle);
        Vector3D un = l*sin(-angle) + u*cos(-angle);
        l = ln;
        u = un;
    }else if (c == '|'){
        h = -h;
        l = -l;
    }else if (c == '('){
        std::vector<Vector3D> vec;
        vec.push_back(pos);
        vec.push_back(posprev);
        vec.push_back(h);
        vec.push_back(l);
        vec.push_back(u);
        stack.push(vec);
    }else if (c == ')'){
        std::vector<Vector3D> vec = stack.top();
        pos = vec[0];
        posprev = vec[1];
        h = vec[2];
        l = vec[3];
        u = vec[4];
        stack.pop();
    }else{
        posprev = pos;
        pos = pos + h;

        if (lsystem.draw(c)){
            points.push_back(posprev);
            points.push_back(pos);
            std::vector<int> pts;
            pts.push_back(points.size() -2);
            pts.push_back(points.size() -1);
            Face face = Face(pts);
            faces.push_back(face);
        }
    }
}

void parseStringRecursive(const std::string& str,const LParser::LSystem3D &lsystem, std::vector<Vector3D>& points, int i, int imax, std::vector<Face>& faces, Vector3D& pos, Vector3D& posprev, Vector3D& h, Vector3D& l, Vector3D& u,const double& angle, std::stack<std::vector<Vector3D>>& stack) {
    for (char c: str) {
        if (c == '-' || c == '+' || c == '^' || c == '&' || c == '\\' || c == '/' || c == '|' || c == '(' || c == ')') {
            parseChar(c, lsystem, points, faces, pos, posprev,h,l,u, angle,stack);
        } else {
            if (i < imax){
                std::string replacement = lsystem.get_replacement(c);
                parseStringRecursive(replacement, lsystem, points, i+1, imax, faces, pos, posprev,h,l,u, angle,stack);
            }else{
                parseChar(c, lsystem, points, faces, pos, posprev,h,l,u, angle,stack);
            }
        }
    }
}

Figure LSystem3D::drawLSystem(const LParser::LSystem3D &lsystem, Color &color) {

    int iterations = lsystem.get_nr_iterations();
    std::string init = lsystem.get_initiator();

    Vector3D pos = Vector3D::point(0,0,0);
    Vector3D posprev = Vector3D::point(0,0,0);

    Vector3D h = Vector3D::vector(1,0,0);
    Vector3D l = Vector3D::vector(0,1,0);
    Vector3D u = Vector3D::vector(0,0,1);

    std::stack<std::vector<Vector3D>> stack;

    double angle = lsystem.get_angle() * M_PI / 180.0;

    std::vector<Vector3D> points;
    std::vector<Face> faces;

    parseStringRecursive(init,lsystem,points,0,iterations,faces,pos,posprev,h,l,u,angle,stack);

    return Figure(points,faces,color);
}
