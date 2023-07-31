#ifndef GRAPHICSDRAWER_HPP
#define GRAPHICSDRAWER_HPP

#include <SFML/Graphics.hpp>

class Graphic
{
    public:
        Graphic(int W, int H);
        void drawPoints();
    private:
        int weigth;
        int heigth;
        int x0;
        int y0;

};

#endif