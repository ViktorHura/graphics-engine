#include "LSystem2D.h"


img::EasyImage LSystem2D::parseConfig(std::string &type, const ini::Configuration &conf) {
    int size = conf["General"]["size"].as_int_or_die();
    std::vector<double> colorB = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    std::vector<double> colorL = conf["2DLSystem"]["color"].as_double_tuple_or_die();

    Color linec = Color(colorL);
    Color bcolor = Color(colorB);

    std::string lfile = conf["2DLSystem"]["inputfile"].as_string_or_die();

    LParser::LSystem2D lsystem;

    std::ifstream input_stream(lfile);
    input_stream >> lsystem;
    input_stream.close();

    Lines2D lines = drawLSystem(lsystem, linec);

    return Line2D::draw2DLines(lines, size, bcolor);
}

void parseChar(char& c, const LParser::LSystem2D &lsystem, Color& color, Lines2D& lines, Point2D& point, Point2D& point_prev, double& angle, double& addangle, std::stack<std::tuple<Point2D,double>>& stack){
    if (c == '-'){
        angle -= addangle;
    }else if (c == '+'){
        angle += addangle;
    }else if (c == '('){
        stack.push(std::make_tuple(point,angle));
    }else if (c == ')'){
        std::tuple<Point2D,double> tuple = stack.top();
        point = std::get<0>(tuple);
        angle = std::get<1>(tuple);
        stack.pop();
    }else{
        point_prev.x = point.x;
        point_prev.y = point.y;
        point.x += cos(angle);
        point.y += sin(angle);

        if (lsystem.draw(c)){
            Line2D line = Line2D(point_prev, point, color);
            lines.push_back(line);
        }
    }
}

void parseStringRecursive(const std::string& str,const LParser::LSystem2D &lsystem, Color& color, Lines2D& lines, int i, int imax, Point2D& point, Point2D& point_prev, double& angle, double& addangle, std::stack<std::tuple<Point2D,double>>& stack){
    for (char c: str) {
        if (c == '-' || c == '+' || c == '(' || c == ')') {
            parseChar(c, lsystem,color, lines, point, point_prev, angle, addangle, stack);
        } else {
            if (i < imax){
                std::string replacement = lsystem.get_replacement(c);
                parseStringRecursive(replacement, lsystem, color, lines, i+1, imax, point, point_prev, angle, addangle, stack);
            }else{
                parseChar(c, lsystem,color, lines, point, point_prev, angle, addangle, stack);
            }
        }
    }
}

Lines2D LSystem2D::drawLSystem(const LParser::LSystem2D &lsystem, Color& color) {
    int iterations = lsystem.get_nr_iterations();

    std::string init = lsystem.get_initiator();

    Lines2D lines = Lines2D();

    Point2D point = Point2D(0,0);
    Point2D point_prev = Point2D(0,0);
    double angle = lsystem.get_starting_angle() * M_PI / 180.0;

    double addangle = lsystem.get_angle() * M_PI / 180.0;

    std::stack<std::tuple<Point2D,double>> stack;


    parseStringRecursive(init, lsystem, color, lines, 0, iterations, point, point_prev, angle, addangle, stack);


    return lines;
}
