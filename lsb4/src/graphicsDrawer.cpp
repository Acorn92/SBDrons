#include "graphicsDrawer.hpp"

#include <boost/thread/thread.hpp>
#include <boost/thread/xtime.hpp>

using namespace sf;

Graphic::Graphic(int W, int H)
{
    this->weigth = W;
    this->heigth = H;
    x0 = this->weigth / 2;
    y0 = this->heigth / 2;
    this->window = new RenderWindow(VideoMode(this->weigth, this->heigth), "Grap!");
    
}

void Graphic::Run() {
    while (window->isOpen()) {
        sf::Event event;
        while (window->pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                            window->close();
        }
    }
    window->clear(sf::Color::White);
    window->display();
    
}

Graphic::~Graphic() {
    if (window != nullptr)
        delete window;
}

void Graphic::drawPoints(StateVector data)
{
    CircleShape point(5.f);
    point.setFillColor(Color::Blue);
    point.setPosition(this->y0 + data.Z, this->x0 + data.timeStamp);
    window->draw(point);
}