
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
        MotionData motionData = get_motion_data();
        SensorData sensorData = get_sensor_data();
        char buffer[32];
        
        snprintf(buffer, sizeof(buffer), "a: %.2f, %.2f, %.2f", motionData.linear_acceleration.X, motionData.linear_acceleration.Y, motionData.linear_acceleration.Z);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "w: %.2f, %.2f, %.2f", motionData.angular_velocity.X, motionData.angular_velocity.Y, motionData.angular_velocity.Z);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "O: %.2f, %.2f, %.2f", motionData.orientation.X * 180.0 / M_PI, motionData.orientation.Y * 180.0 / M_PI, motionData.orientation.Z * 180.0 / M_PI);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "Stationary: %s", motionData.stationary ? "Yes" : "No");
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "R: %.2f L: %.2f Df: %.2f", motionData.right_wheel_angle * 180.0 / M_PI, motionData.left_wheel_angle * 180.0 / M_PI, motionData.forward_distance);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;
        snprintf(buffer, sizeof(buffer), "vl: %.2f RPM vr: %.2f RPM", motionData.left_wheel_velocity * 60.0 / (2.0 * M_PI), motionData.right_wheel_velocity * 60.0 / (2.0 * M_PI));
        g.drawText(x, y, buffer, -1, -1, Color::White());
        y += textHeight;

        x = 0;
        y = g.getHeight() - textHeight * 2;

        if (sensorData.distanceSensorValues.size() != 7)
        {
            g.drawText(x, y, "Invalid sensor data", -1, -1, Color::White());
            return;
        }
        snprintf(buffer, sizeof(buffer), "%d", sensorData.distanceSensorValues[5]);
        g.drawText(x, y, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%d", sensorData.distanceSensorValues[4]);
        g.drawText(x + 28, y, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%d", sensorData.distanceSensorValues[3]);
        g.drawText(x + 56, y, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%d", sensorData.distanceSensorValues[2]);
        g.drawText(x + 84, y, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%d", sensorData.distanceSensorValues[1]);
        g.drawText(x + 112, y, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%d", sensorData.distanceSensorValues[6]);
        g.drawText(x, y + textHeight, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%s", sensorData.lineSensorLeft ? "WHITE" : "BLACK");
        g.drawText(x + 28, y + textHeight, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%s", sensorData.lineSensorRight ? "WHITE" : "BLACK");
        g.drawText(x + 84, y + textHeight, buffer, -1, -1, Color::White());
        snprintf(buffer, sizeof(buffer), "%d", sensorData.distanceSensorValues[0]);
        g.drawText(x + 112, y + textHeight, buffer, -1, -1, Color::White());
        if (sensorData.targetDetected)
        {
            g.drawText(x + 62, y + textHeight, "|", -1, -1, Color::White());
        }
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
        char buffer[32];
        switch (combat_state)
        {
            case STATE_SEARCH:
                snprintf(buffer, sizeof(buffer), "Combat State: SEARCH");
                break;
            case STATE_APPROACH:
                snprintf(buffer, sizeof(buffer), "Combat State: APPROACH");
                break;
            case STATE_PREPARE_ATTACK:
                snprintf(buffer, sizeof(buffer), "Combat State: PREPARE_ATTACK");
                break;
            case STATE_ATTACK:
                snprintf(buffer, sizeof(buffer), "Combat State: ATTACK");
                break;
            default:
                snprintf(buffer, sizeof(buffer), "Combat State: UNKNOWN");
                break;
        }
        g.drawText(0, TOP_RIBON_SIZE + 16, buffer, -1, -1, Color::White());
        
    }

    bool allowOverlay() const override
    {
        return true;
    }
};



MainMenuView mainMenu(graphics, menuItems);
SensorView sensorView;
HomeView homeView;
View* selectedView = nullptr;

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
