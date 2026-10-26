
#include "ssd1306.h"
#include "lcd.h"
#include "graphics.h"
#include "display.h"
#include "graphics_component.h"
#include "components/vertical_scrolling_selector.h"
#include "components/text_box.h"
#include "app/motion.h"
#include "app/sensors.h"

constexpr int TOP_RIBON_SIZE = 6;
constexpr int REFRESH_RATE = 10; // Refresh rate for the display in Hz
constexpr int REFRESH_PERIOD_US = 1000000 / REFRESH_RATE; // Refresh period for the display in microseconds

//TextBox MenuItem_Settings(0, 0, "SETTINGS");
//TextBox MenuItem_tactics(0, 0, "TACTICS");
TextBox MenuItem_home(0, 0, "HOME");
TextBox MenuItem_sensors(0, 0, "SENSORS");

std::vector<GraphicsComponent*> menuItems = 
{
    &MenuItem_home,
    //&MenuItem_Settings,
    //&MenuItem_tactics,
    &MenuItem_sensors
};

class View
{
public:
    virtual void draw(Graphics& g) = 0;
    virtual bool allowOverlay() const = 0;
    virtual ~View() = default;
};

class MainMenuView : public View
{
    VerticalScrollingSelector selector;
    bool _isOpen = false;

public:
    MainMenuView(Graphics& graphics, std::vector<GraphicsComponent*>& items) : selector(0, 4, graphics.getWidth(), graphics.getHeight()-4, items)
    {
    }

    void draw(Graphics& g) override
    {
        if (_isOpen)
        {
            // Draw the menu items using the selector
            selector.draw(g);
            // Draw the menu background and title
            g.fillRectangle(0, 0, g.getWidth(), TOP_RIBON_SIZE - 1, Color::White());
            g.drawText(5, 0, "MENU", -1, -1, Color::None());
            //draw the arrow in place of the menu icon
            g.drawPixel(0, 1, Color::None());
            g.drawPixel(0, 3, Color::None());
            g.drawPixel(1, 0, Color::None());
            g.drawPixel(1, 1, Color::None());
            g.drawPixel(1, 3, Color::None());
            g.drawPixel(1, 4, Color::None());
            g.drawPixel(2, 1, Color::None());
            g.drawPixel(2, 3, Color::None());
            g.drawPixel(3, 2, Color::None());
            
        }
        else
        {
            auto selectedItem = selector.getText();
            g.drawText(g.getWidth() - g.getTextWidth(selectedItem.c_str()), 0, selectedItem.c_str(), -1, -1, Color::White());
            //Draw a simple "closed" menu icon (three horizontal lines)
            g.drawLine(0, 0, 3, 0, 1, Color::White());
            g.drawLine(0, 2, 3, 2, 1, Color::White());
            g.drawLine(0, 4, 3, 4, 1, Color::White());
        }
    }

    void menuToggle()
    {
        _isOpen = !_isOpen;
    }

    int currentSelection() const
    {
        return selector.selectedIndex;
    }

    void updateSelection(int newIndex)
    {
        selector.selectedIndex = newIndex;
    }

    bool allowOverlay() const override
    {
        return true;
    }

    bool isOpen() const
    {
        return _isOpen;
    }
};

class SensorView : public View
{
public:
    

    void draw(Graphics& g) override
    {
        int x = 0;
        int y = TOP_RIBON_SIZE;
        int textHeight = g.getFontHeight();
        char buffer[50];

        SensorData sensorData = get_sensor_data();
        snprintf(buffer, sizeof(buffer), "%.4d %.4d %.4d %.4d %.4d %.4d %.4d", 
                sensorData.distanceSensorValues[0],
                sensorData.distanceSensorValues[1],
                sensorData.distanceSensorValues[2],
                sensorData.distanceSensorValues[3],
                sensorData.distanceSensorValues[4],
                sensorData.distanceSensorValues[5],
                sensorData.distanceSensorValues[6]);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;

        /*
        snprintf(buffer, sizeof(buffer), "Delta Heading: %.2f", delta_heading * 180.0 / M_PI);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "Target Heading: %.2f", target_heading * 180.0 / M_PI);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "Total Heading: %.2f", total_heading * 180.0 / M_PI);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "Line Avoidance State: %d", static_cast<int>(line_avoidance_state));
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "Motion State: %d", static_cast<int>(motion_state));
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        */
    }

    bool allowOverlay() const override
    {
        return true;
    }
};

class HomeView : public View
{
public:
    void draw(Graphics& g) override
    {
        g.drawText(0, TOP_RIBON_SIZE, "Home View", -1, -1, Color::White());
    }

    bool allowOverlay() const override
    {
        return true;
    }
};



MainMenuView mainMenu(graphics, menuItems);
SensorView sensorView;
HomeView homeView;
View* selectedView = &sensorView;

void menuNavigate(int direction)
{
    if (!mainMenu.isOpen()) return;
    mainMenu.updateSelection(mainMenu.currentSelection() + direction);
}


void menuToggle()
{
    mainMenu.menuToggle();
}

void menuSelect()
{
    if (!mainMenu.isOpen()) return;
    switch (mainMenu.currentSelection())
    {
        case 0:
            selectedView = &homeView;
            break;
        case 1:
            selectedView = &sensorView;
            break;
        default:
            selectedView = nullptr;
            break;
        // Add more cases for other menu items if needed
    }
    mainMenu.menuToggle();
}

void display_init()
{
    // Initialize display-related resources here
}

void handle_display()
{
    static utils::time_t last_refresh_time = 0;
    if (!utils::hasElapsed_us(last_refresh_time, REFRESH_PERIOD_US))
    {
        return;
    }
    last_refresh_time = utils::now();
    graphics.clear();

    if (!mainMenu.isOpen())
    {
        auto currentView = selectedView;
        if (currentView)
        {
            currentView->draw(graphics);
        }
        else
        {
            graphics.drawText(0, TOP_RIBON_SIZE, "No view selected", -1, -1, Color::White());
        }
    }

    mainMenu.draw(graphics);
    graphics.display();
}
