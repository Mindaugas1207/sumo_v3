#ifndef TEXT_BOX_H
#define TEXT_BOX_H

#include "graphics_component.h"
#include <string>

class TextBox : public GraphicsComponent
{
    int x;
    int y;
    int width;
    int height;
    std::string text;
    Color color;
public:
    constexpr static int AUTO_SIZE = -1;

    TextBox(int x, int y, const std::string& text = "", Color color = Color::White(), int width = AUTO_SIZE, int height = AUTO_SIZE) : x(x), y(y), width(width), height(height), text(text), color(color) {}

    const std::string& getText() const override { return text; }
    void setText(const std::string& newText) { text = newText; }
    const Color& getColor() const { return color; }
    void setColor(const Color& newColor) { color = newColor; }

    void draw(Graphics& g) override
    {
        g.drawText(x, y, text.c_str(), width, height, color);
    }

    void setPosition(int x, int y) override { this->x = x; this->y = y; }
    void setSize(int width, int height) override { this->width = width; this->height = height; }
    void setHeight(int height) override { this->height = height; }
    void setWidth(int width) override { this->width = width; }
    int getX() const override { return x; }
    int getY() const override { return y; }
    int getWidth() const override { return width == AUTO_SIZE ? Graphics::getDefaultFontWidth() * static_cast<int>(text.length()) : width; }
    int getHeight() const override { return height == AUTO_SIZE ? Graphics::getDefaultFontHeight() : height; }
};

#endif // TEXT_BOX_H