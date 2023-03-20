//
// Created by viktor on 29.05.20.
//

#ifndef ENGINE_ZBUFFER_H
#define ENGINE_ZBUFFER_H


#include <vector>
#include <limits>

class ZBuffer{
    std::vector<double> data;
    int rows;
    int cols;
public:
    //Constructor: maakt een Z-Buffer van de correcte
    //grootte aan en initialiseert alle velden op +inf
    ZBuffer(const int width, const int height){
        rows = height;
        cols = width;
        for (int i =0; i < rows*cols; i++){
            data.push_back(std::numeric_limits<double>::infinity());
        }
    }
    void set(int row, int col, double input){
        int nrow = rows - col;
        int ncol = row;
        data[cols * nrow + ncol] = input;
    }
    double at(int row, int col){
        int nrow = rows - col;
        int ncol = row;
        return data[cols * nrow + ncol];
    }
    int width(){
        return cols;
    }
    int height(){
        return rows;
    }
};


#endif //ENGINE_ZBUFFER_H
