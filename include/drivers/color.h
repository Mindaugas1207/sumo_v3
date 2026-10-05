#ifndef COLOR_H
#define COLOR_H

#include <cstdint>

/*! @brief Class representing an RGBW color.
 *  This class provides constructors for different color formats and predefined color constants.
 */
class Color
{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
public:
    /*! @brief Default constructor. Initializes the color to black (0,0,0,0). */
    Color() : r(0), g(0), b(0), w(0)
    {
    }

    /*! @brief Constructor with specified RGBW values.
     *  @param r Red component (0-255)
     *  @param g Green component (0-255)
     *  @param b Blue component (0-255)
     *  @param w White component (0-255), default is 0
     */
    Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) : r(r), g(g), b(b), w(w)
    {
    }

    /*! @brief Constructor from a 32-bit GRBW value.
     *  The input format is 0xWWRRGGBB, where WW is the white component, RR is red, GG is green, and BB is blue.
     *  @param grbw 32-bit GRBW color value
     */
    Color(uint32_t grbw) : r((grbw >> 8) & 0xFF), g((grbw >> 16) & 0xFF), b(grbw & 0xFF), w((grbw >> 24) & 0xFF)
    {
    }

    /*! @brief Predefined color constants. */
    static Color Red(void) {return Color(255,0,0);}
    static Color Green(void) {return Color(0,255,0);}
    static Color Blue(void) {return Color(0,0,255);}
    static Color White(void) {return Color(255,255,255);}
    static Color Black(void) {return Color(0,0,0);}
    static Color None(void) {return Color(0,0,0);}
    static Color Yellow(void) {return Color(255,255,0);}
    static Color Cyan(void) {return Color(0,255,255);}
    static Color Magenta(void) {return Color(255,0,255);}
    static Color Orange(void) {return Color(255,165,0);}
    static Color Purple(void) {return Color(128,0,128);}
    static Color Pink(void) {return Color(255,192,203);}
    static Color Lime(void) {return Color(0,255,0);}
    static Color Gray(void) {return Color(128,128,128);}
    static Color Brown(void) {return Color(165,42,42);}
    static Color LightBlue(void) {return Color(173,216,230);}
    static Color DarkBlue(void) {return Color(0,0,139);}
    static Color Gold(void) {return Color(255,215,0);}
    static Color Silver(void) {return Color(192,192,192);}

    /*! @brief Convert the color to a 32-bit GRBW value.
     *  @return The 32-bit GRBW representation of the color.
     */
    uint32_t toGRBW_u32(void)
    {
        return ((uint32_t)w << 24) | ((uint32_t)g << 16) | ((uint32_t)r << 8) | (b);
    }

    /*! @brief Equality operator for comparing two colors.
     *  @param other The other color to compare with.
     *  @return True if the colors are equal, false otherwise.
     */
    bool operator==(const Color& other) const
    {
        return r == other.r && g == other.g && b == other.b && w == other.w;
    }
};

#endif // COLOR_H


