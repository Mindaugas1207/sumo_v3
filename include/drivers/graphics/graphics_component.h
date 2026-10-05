#ifndef GRAPHICS_COMPONENT_H
#define GRAPHICS_COMPONENT_H

#include "graphics.h"
#include <string>

class GraphicsComponent
{
public:
    virtual void draw(Graphics& g) = 0;

    virtual const std::string& getText() const = 0;

    virtual void setPosition(int x, int y) = 0;
    virtual void setSize(int width, int height) = 0;
    virtual void setHeight(int height) = 0;
    virtual void setWidth(int width) = 0;
    virtual int getX() const = 0;
    virtual int getY() const = 0;
    virtual int getWidth() const = 0;
    virtual int getHeight() const = 0;
};

#endif // GRAPHICS_COMPONENT_H