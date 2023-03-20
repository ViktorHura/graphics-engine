//
// Created by viktor on 20.03.20.
//

#include "../easy_image.h"
#include "../ini_configuration.h"
#include "Utils.h"
#include <string>

#ifndef ENGINE_INTRO_H
#define ENGINE_INTRO_H


class Intro {
public:
    static img::EasyImage parseConfig(std::string& type, const ini::Configuration &conf);
    static void quarterCircleLU(img::EasyImage& image, int& xO, int& yO, int& w, int& h, Color& colorB, Color& colorL, int& nrL);
    static void quarterCircleLD(img::EasyImage& image, int& xO, int& yO, int& w, int& h, Color& colorB, Color& colorL, int& nrL);
    static void quarterCircleRU(img::EasyImage& image, int& xO, int& yO, int& w, int& h, Color& colorB, Color& colorL, int& nrL);
    static void quarterCircleRD(img::EasyImage& image, int& xO, int& yO, int& w, int& h, Color& colorB, Color& colorL, int& nrL);
private:
    static img::EasyImage IntroColorRectangle(int& w, int& h);
    static img::EasyImage IntroBlocks(int& w, int& h, int& nrXB, int& nrYB, std::vector<double>& colorW, std::vector<double>& colorB, bool invert);
    static img::EasyImage QuarterCircle(int& w, int& h,Color& colorB, Color& colorL, int& nrL);
    static img::EasyImage Eye(int& w, int& h, Color& colorB, Color& colorL, int& nrL);
    static img::EasyImage Diamond(int& w, int& h, Color& colorB, Color& colorL, int& nrL);
};


#endif //ENGINE_INTRO_H
