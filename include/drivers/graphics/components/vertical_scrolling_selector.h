#ifndef VERTICAL_SCROLLING_SELECTOR_H
#define VERTICAL_SCROLLING_SELECTOR_H

#include "graphics_component.h"
#include <vector>
#include <string>

class VerticalScrollingSelector : public GraphicsComponent
{
    int x;
    int y;
    int width;
    int height;
    int currentTopIndex = 0;
public:
    std::vector<GraphicsComponent*> items;
    bool drawBorder = true;
    int selectedIndex = 0;

    VerticalScrollingSelector(int x, int y, int width, int height) : x(x), y(y), width(width), height(height), items()
    {
    }

    VerticalScrollingSelector(int x, int y, int width, int height, const std::vector<GraphicsComponent*>& items) : x(x), y(y), width(width), height(height), items(items)
    {
    }

    int getX() const override { return x; }
    int getY() const override { return y; }
    int getWidth() const override { return width; }
    int getHeight() const override { return height; }
    void setHeight(int height) override { this->height = height; }
    void setWidth(int width) override { this->width = width; }

    const std::string& getText() const override
    {
        auto selectedItem = getSelectedItem();
        if (selectedItem)
        {
            return selectedItem->getText();
        }
        static const std::string emptyString;
        return emptyString;
    }

    GraphicsComponent* getSelectedItem() const
    {
        if (selectedIndex >= 0 && selectedIndex < static_cast<int>(items.size()))
        {
            return items[selectedIndex];
        }
        return nullptr;
    }

    void setPosition(int x, int y) override
    {
        this->x = x;
        this->y = y;
    }

    void setSize(int width, int height) override
    {
        this->width = width;
        this->height = height;
    }

    void draw(Graphics& g) override
    {
        if (drawBorder)
        {
            g.drawRectangle(x, y, width, height, 1, Color::White());
        }

        if (items.empty())
        {
            return;
        }

        int item_height;
        int offset_x = x + g.getFontGlyphAdvance('>') + 1;
        int offset_y = 2;

        if (selectedIndex < 0)
        {
            selectedIndex = 0;
        }
        else if (selectedIndex >= static_cast<int>(items.size()))
        {
            selectedIndex = static_cast<int>(items.size()) - 1;
        }

        if (selectedIndex < currentTopIndex)
        {
            currentTopIndex = selectedIndex;
        }

        bool indexDrawn = false;

        for (size_t i = currentTopIndex; i < items.size(); i++)
        {
            auto *item = items[i];
            item->setPosition(offset_x, y + offset_y);
            item->setWidth(width - offset_x - 1);
            item->draw(g);

            if (i == selectedIndex)
            {
                g.drawCharacter(x + 1, y + offset_y, '>', -1, -1, Color::White());
                indexDrawn = true;
            }
            offset_y += item->getHeight();
            if (offset_y >= height)
            {
                break;
            }
        }

        if (!indexDrawn)
        {
            currentTopIndex ++;
        }
    }
};

#endif // VERTICAL_SCROLLING_SELECTOR_H