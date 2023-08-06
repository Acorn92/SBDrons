#ifndef GRAPHICSDRAWER_HPP
#define GRAPHICSDRAWER_HPP

#include <SFML/Graphics.hpp>
#include <message.hpp>


class Graphic
{
    public:
        Graphic(int W, int H);
        void drawPoints(StateVector data);
        void Run();
        ~Graphic();
    private:
        int weigth;
        int heigth;
        int x0;
        int y0;
        sf::RenderWindow *window;

};

#endif