#include "graphicsDrawer.hpp"


using namespace sf;

Graphic::Graphic(int W, int H)
{
    this->weigth = W;
    this->heigth = H;
    x0 = this->weigth / 2;
    y0 = this->heigth / 2;

}

void Graphic::drawPoints()
{
    RenderWindow window(VideoMode(this->weigth, this->heigth), "Grap!");
}