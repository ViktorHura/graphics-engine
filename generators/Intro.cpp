//
// Created by viktor on 20.03.20.
//

#include <cmath>
#include "Intro.h"

img::EasyImage Intro::IntroColorRectangle(int& w, int& h) {
    img::EasyImage image(w, h);

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            image(i, j).red = i % 255;
            image(i, j).green = j % 255;
            image(i, j).blue = (i % 255 + j % 255) % 256;
        }
    }

    return image;
}

void colorWhite_Black(img::EasyImage& image, int& x, int& y, std::vector<double>& colorW, std::vector<double>& colorB, bool white){
    if (white){
        image(x, y).red = Color::colorDouble_to_Int(colorW[0]);
        image(x, y).green = Color::colorDouble_to_Int(colorW[1]);
        image(x, y).blue = Color::colorDouble_to_Int(colorW[2]);
    }else{
        image(x, y).red = Color::colorDouble_to_Int(colorB[0]);
        image(x, y).green = Color::colorDouble_to_Int(colorB[1]);
        image(x, y).blue = Color::colorDouble_to_Int(colorB[2]);
    }
}

img::EasyImage Intro::IntroBlocks(int& w, int& h, int& nrXB, int& nrYB, std::vector<double>& colorW, std::vector<double>& colorB, bool invert) {
    if (w % nrXB != 0 || h % nrYB != 0){
        return img::EasyImage();
    }

    img::EasyImage image(w, h);
    int wB = w / nrXB;
    int hB = h / nrYB;

    for (int x = 0; x < h; x++) {
        for (int y = 0; y < w; y++) {
            int bX = x / hB;
            int bY = y / wB;

            bool white = (bX + bY) % 2 == 0;
            if (!invert){
                colorWhite_Black(image, x,y, colorW, colorB, white);
            }else{
                colorWhite_Black(image, x,y, colorW, colorB, !white);
            }
        }
    }

    return image;
}

void Intro::quarterCircleLU(img::EasyImage &image, int& xO, int& yO, int &w, int &h, Color &colorB, Color &colorL,int &nrL) {
    int x = 0;
    int y = 0;
    Color color = Color(colorL);

    double Hs = (double)(h-1) / (double)(nrL - 1);
    double Ws = (double)(w-1) / (double)(nrL - 1);
    int x1 = xO;
    int y2 = yO + h-1;

    for (int i =0; i < nrL; i++ ){
        y = yO + std::floor((double)i * Hs);
        x = xO + std::floor((double)i * Ws);

        Line2D::drawLine(image, x1, y,  x, y2, color);
    }
}

void Intro::quarterCircleLD(img::EasyImage &image, int& xO, int& yO, int &w, int &h, Color &colorB, Color &colorL,int &nrL) {
    int x = 0;
    int y = 0;
    Color color = Color(colorL);

    double Hs = (double)(h-1) / (double)(nrL - 1);
    double Ws = (double)(w-1) / (double)(nrL - 1);
    int x1 = xO;
    int y2 = yO;

    for (int i =0; i < nrL; i++ ){
        y = yO + std::floor((double)(nrL - i - 1) * Hs);
        x = xO + std::floor((double)i * Ws);

        Line2D::drawLine(image, x1, y,  x, y2, color);
    }
}

void Intro::quarterCircleRU(img::EasyImage &image, int& xO, int& yO, int &w, int &h, Color &colorB, Color &colorL,int &nrL) {
    int x = 0;
    int y = 0;
    Color color = Color(colorL);

    double Hs = (double)(h-1) / (double)(nrL - 1);
    double Ws = (double)(w-1) / (double)(nrL - 1);
    int x1 = xO + w-1;
    int y2 = yO + h-1;

    for (int i =0; i < nrL; i++ ){
        y = yO + std::floor((double)i * Hs);
        x = xO + std::floor((double)(nrL - i - 1) * Ws);

        Line2D::drawLine(image, x1, y,  x, y2, color);
    }
}

void Intro::quarterCircleRD(img::EasyImage &image, int& xO, int& yO, int &w, int &h, Color &colorB, Color &colorL,int &nrL) {
    int x = 0;
    int y = 0;
    Color color = Color(colorL);

    double Hs = (double)(h-1) / (double)(nrL - 1);
    double Ws = (double)(w-1) / (double)(nrL - 1);
    int x1 = xO + w-1;
    int y2 = yO;

    for (int i =0; i < nrL; i++ ){
        y = yO + std::floor((double)(nrL - i - 1) * Hs);
        x = xO + std::floor((double)(nrL - i - 1) * Ws);

        Line2D::drawLine(image, x1, y,  x, y2, color);
    }
}

img::EasyImage Intro::QuarterCircle(int &w, int &h, Color& colorB, Color& colorL, int& nrL) {
    img::EasyImage image(w, h);
    Utils::fillColor(image,colorB);

    int xO = 0;
    int yO = 0;

    quarterCircleLU(image,xO, yO, w,h,colorB,colorL,nrL);

    return image;
}

img::EasyImage Intro::Eye(int &w, int &h, Color& colorB, Color& colorL, int& nrL) {
    img::EasyImage image(w, h);
    Utils::fillColor(image,colorB);

    int xO = 0;
    int yO = 0;

    quarterCircleLU(image,xO, yO, w,h,colorB,colorL,nrL);
    quarterCircleRD(image,xO, yO, w,h,colorB,colorL,nrL);

    return image;
}

img::EasyImage Intro::Diamond(int &w, int &h, Color& colorB, Color& colorL, int& nrL) {
    img::EasyImage image(w, h);
    Utils::fillColor(image,colorB);

    int xO = 0;
    int yO = 0;

    int cW = w / 2;
    int cH = h / 2;

    quarterCircleRU(image,xO, yO, cW,cH,colorB,colorL,nrL);

    yO = cH;
    quarterCircleRD(image,xO, yO, cW,cH,colorB,colorL,nrL);


    xO = cW;
    quarterCircleLD(image,xO, yO, cW,cH,colorB,colorL,nrL);

    yO = 0;
    quarterCircleLU(image,xO, yO, cW,cH,colorB,colorL,nrL);


    return image;
}

img::EasyImage Intro::parseConfig(std::string &type, const ini::Configuration &conf) {
    int w = conf["ImageProperties"]["width"].as_int_or_die();
    int h = conf["ImageProperties"]["height"].as_int_or_die();

    if (type == "IntroColorRectangle") {
        return Intro::IntroColorRectangle(w, h);

    }else if (type == "IntroBlocks") {
        int nrXB = conf["BlockProperties"]["nrXBlocks"].as_int_or_die();
        int nrYB = conf["BlockProperties"]["nrYBlocks"].as_int_or_die();

        std::vector<double> colorW = conf["BlockProperties"]["colorWhite"].as_double_tuple_or_die();
        std::vector<double> colorB = conf["BlockProperties"]["colorBlack"].as_double_tuple_or_die();

        bool invert = false;

        conf["BlockProperties"]["invertColors"].as_bool_if_exists(invert);

        return Intro::IntroBlocks(w, h, nrXB, nrYB, colorW, colorB, invert);
    }else if (type == "IntroLines"){
        std::string figure = conf["LineProperties"]["figure"].as_string_or_die();

        std::vector<double> colorB = conf["LineProperties"]["backgroundcolor"].as_double_tuple_or_die();
        std::vector<double> colorL = conf["LineProperties"]["lineColor"].as_double_tuple_or_die();

        int nrL = conf["LineProperties"]["nrLines"].as_int_or_die();

        Color cB = Color(colorB);
        Color cL = Color(colorL);

        if (figure == "QuarterCircle"){
            return QuarterCircle(w,h, cB, cL, nrL);
        }

        if (figure == "Eye") {
            return Eye(w, h, cB, cL, nrL);
        }
        if (figure == "Diamond") {
            return Diamond(w, h, cB, cL, nrL);
        }

    }
}
