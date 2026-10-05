#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "lcd.h"
#include "image.h"
#include "font.h"
#include "color.h"
#include "math.h"
#include "tiny_hex_font.h"

class Graphics
{
    constexpr static FontAsset defaultFont = tinyhex_3x5;
    LCD& lcd;
public:
    Graphics(LCD& lcd) : lcd(lcd) {}

    int getWidth() const
    {
        return lcd.getWidth();
    }

    int getHeight() const
    {
        return lcd.getHeight();
    }

    void clear()
    {
        lcd.clear();
    }

    void display()
    {
        lcd.display();
    }

    int getFontHeight(const FontAsset& font = defaultFont) const
    {
        return font.line_height;
    }

    constexpr static FontAsset getDefaultFont() { return defaultFont; }
    constexpr static int getDefaultFontHeight() { return defaultFont.line_height; }
    constexpr static int getDefaultFontWidth() { return defaultFont.glyphs[0].width; }

    int getFontGlyphWidth(char c, const FontAsset& font = defaultFont) const
    {
        if (c < font.first_char || c > font.last_char)
        {
            return 0;
        }

        const uint16_t glyph_index = static_cast<uint16_t>(c - font.first_char);
        const FontGlyph& glyph = font.glyphs[glyph_index];
        return glyph.width;
    }

    int getFontGlyphAdvance(char c, const FontAsset& font = defaultFont) const
    {
        if (c < font.first_char || c > font.last_char)
        {
            return 0;
        }

        const uint16_t glyph_index = static_cast<uint16_t>(c - font.first_char);
        const FontGlyph& glyph = font.glyphs[glyph_index];
        return glyph.advance;
    }

    int getTextWidth(const char* text, const FontAsset& font = defaultFont) const
    {
        int width = 0;
        for (const char* p = text; *p; ++p)
        {
            width += getFontGlyphAdvance(*p, font);
        }
        return width;
    }

    void drawPixel(int x, int y, Color color = Color::White())
    {
        lcd.setPixel(x, y, color);
    }

    void drawPixel(int x, int y, bool on)
    {
        lcd.setPixel(x, y, on);
    }

    void setPixel(int x, int y)
    {
        lcd.setPixel(x, y, true);
    }

    void clearPixel(int x, int y)
    {
        lcd.setPixel(x, y, false);
    }

    void _drawLine(int x0, int y0, int x1, int y1, Color color)
    {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            drawPixel(x0, y0, color);

            if (x0 == x1 && y0 == y1)
                break;

            int err2 = err * 2;
            if (err2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (err2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    void drawLine(int x0, int y0, int x1, int y1, int thickness = 1, Color color = Color::White())
    {
        if (thickness <= 1)
        {
            _drawLine(x0, y0, x1, y1, color);
            return;
        }

        int center_offset = (thickness % 2 == 0) ? 1 : 0;
        int radius_scaled_sq = thickness * thickness;
        int brush_extent = thickness / 2 + 1;

        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            for (int oy = -brush_extent; oy <= brush_extent; ++oy)
            {
                int sy_scaled = 2 * oy + center_offset;
                for (int ox = -brush_extent; ox <= brush_extent; ++ox)
                {
                    int sx_scaled = 2 * ox + center_offset;
                    if (sx_scaled * sx_scaled + sy_scaled * sy_scaled <= radius_scaled_sq)
                    {
                        drawPixel(x0 + ox, y0 + oy, color);
                    }
                }
            }

            if (x0 == x1 && y0 == y1)
                break;

            int err2 = err * 2;
            if (err2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (err2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    void drawRectangle(int x, int y, int width, int height, int thickness = 1, Color color = Color::White())
    {
        drawLine(x, y, x + width - 1, y, thickness, color);
        drawLine(x, y, x, y + height - 1, thickness, color);
        drawLine(x + width - 1, y, x + width - 1, y + height - 1, thickness, color);
        drawLine(x, y + height - 1, x + width - 1, y + height - 1, thickness, color);
    }

    void fillRectangle(int x, int y, int width, int height, Color color = Color::White())
    {
        for (int j = 0; j < height; j++)
        {
            _drawLine(x, y + j, x + width - 1, y + j, color);
        }
    }

    void drawCircle(int x0, int y0, int radius, Color color = Color::White())
    {
        int x = radius;
        int y = 0;
        int err = 0;

        while (x >= y)
        {
            drawPixel(x0 + x, y0 + y, color);
            drawPixel(x0 + y, y0 + x, color);
            drawPixel(x0 - y, y0 + x, color);
            drawPixel(x0 - x, y0 + y, color);
            drawPixel(x0 - x, y0 - y, color);
            drawPixel(x0 - y, y0 - x, color);
            drawPixel(x0 + y, y0 - x, color);
            drawPixel(x0 + x, y0 - y, color);

            if (err <= 0)
            {
                err += 2 * y + 1;
                y++;
            }
            if (err > 0)
            {
                err -= 2 * x + 1;
                x--;
            }
        }
    }

    void fillCircle(int x0, int y0, int radius, Color color = Color::White())
    {
        int x = radius;
        int y = 0;
        int err = 0;

        while (x >= y)
        {
            _drawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
            _drawLine(x0 - x, y0 - y, x0 + x, y0 - y, color);
            _drawLine(x0 - y, y0 + x, x0 + y, y0 + x, color);
            _drawLine(x0 - y, y0 - x, x0 + y, y0 - x, color);

            if (err <= 0)
            {
                err += 2 * y + 1;
                y++;
            }
            if (err > 0)
            {
                err -= 2 * x + 1;
                x--;
            }
        }
    }

    void drawCircle(int x0, int y0, int radius, int thickness = 1, Color color = Color::White())
    {
        if (thickness <= 1)
        {
            drawCircle(x0, y0, radius, color);
            return;
        }

        int inner_radius = radius - thickness / 2;
        int outer_radius = radius + thickness / 2;

        for (int r = inner_radius; r <= outer_radius; r++)
        {
            drawCircle(x0, y0, r, color);
        }
    }

    void drawCharacter(int x, int y, char c, int maxWidth = -1, Color color = Color::White(), const FontAsset& font = defaultFont)
    {
        if (c < font.first_char || c > font.last_char || (maxWidth >= 0 && x > maxWidth))
        {
            return;
        }

        const uint16_t glyph_index = static_cast<uint16_t>(c - font.first_char);
        const FontGlyph& glyph = font.glyphs[glyph_index];
        if (glyph.width == 0 || glyph.height == 0)
        {
            return;
        }

        const uint8_t* glyph_bitmap = font.bitmap + glyph.bitmap_offset;
        const uint16_t stride_bytes = (glyph.width + 7) / 8;

        for (uint16_t row = 0; row < glyph.height; ++row)
        {
            for (uint16_t column = 0; column < glyph.width; ++column)
            {
                const uint16_t byte_index = row * stride_bytes + (column / 8);
                const uint8_t bit_index = 7 - (column % 8);
                const bool pixel_on = ((glyph_bitmap[byte_index] >> bit_index) & 0x01) != 0;

                if (maxWidth >= 0 && x + glyph.x_offset + column >= x + maxWidth)
                {
                    continue;
                }

                lcd.setPixel(x + glyph.x_offset + column, y + glyph.y_offset + row, pixel_on ? color : Color::None());
            }
        }
    }
    
    void drawText(int x, int y, const char* text, int maxWidth = -1, Color color = Color::White(), const FontAsset& font = defaultFont)
    {
        if (text == nullptr || *text == '\0' || (maxWidth >= 0 && x >= x + maxWidth))
        {
            return;
        }

        int cursor_x = x;
        int cursor_y = y;

        for (const char* p = text; *p != '\0'; ++p)
        {
            const char c = *p;

            if (c == '\n')
            {
                cursor_x = x;
                cursor_y += font.line_height;
                continue;
            }

            if (maxWidth >= 0 && cursor_x >= x + maxWidth)
            {
                continue;
            }

            if (c < font.first_char || c > font.last_char)
            {
                cursor_x += static_cast<int>(font.pixel_size) / 2;
                continue;
            }

            const uint16_t glyph_index = static_cast<uint16_t>(c - font.first_char);
            const FontGlyph& glyph = font.glyphs[glyph_index];
            drawCharacter(cursor_x, cursor_y, c, maxWidth >= 0 ? maxWidth - (cursor_x - x) : -1, color, font);
            cursor_x += glyph.advance;
        }
    }

    void drawCharacter(int x, int y, char c, int maxWidth = -1, int maxHeight = -1, Color color = Color::White(), const FontAsset& font = defaultFont)
    {
        if (c < font.first_char || c > font.last_char || (maxWidth >= 0 && x > maxWidth) || (maxHeight >= 0 && y > maxHeight))
        {
            return;
        }

        const uint16_t glyph_index = static_cast<uint16_t>(c - font.first_char);
        const FontGlyph& glyph = font.glyphs[glyph_index];
        if (glyph.width == 0 || glyph.height == 0)
        {
            return;
        }

        const uint8_t* glyph_bitmap = font.bitmap + glyph.bitmap_offset;
        const uint16_t stride_bytes = (glyph.width + 7) / 8;

        for (uint16_t row = 0; row < glyph.height; ++row)
        {
            for (uint16_t column = 0; column < glyph.width; ++column)
            {
                const uint16_t byte_index = row * stride_bytes + (column / 8);
                const uint8_t bit_index = 7 - (column % 8);
                const bool pixel_on = ((glyph_bitmap[byte_index] >> bit_index) & 0x01) != 0;

                if ((maxWidth >= 0 && x + glyph.x_offset + column >= x + maxWidth) || (maxHeight >= 0 && y + glyph.y_offset + row >= y + maxHeight))
                {
                    continue;
                }

                if (pixel_on)
                {
                    lcd.setPixel(x + glyph.x_offset + column, y + glyph.y_offset + row, color);
                }
            }
        }
    }
    
    void drawText(int x, int y, const char* text, int maxWidth = -1, int maxHeight = -1, Color color = Color::White(), const FontAsset& font = defaultFont)
    {
        if (text == nullptr || *text == '\0' || (maxWidth >= 0 && x >= x + maxWidth) || (maxHeight >= 0 && y >= y + maxHeight))
        {
            return;
        }

        int cursor_x = x;
        int cursor_y = y;

        for (const char* p = text; *p != '\0'; ++p)
        {
            const char c = *p;

            if (c == '\n')
            {
                cursor_x = x;
                cursor_y += font.line_height;
                continue;
            }

            if (maxWidth >= 0 && cursor_x >= x + maxWidth)
            {
                continue;
            }

            if (maxHeight >= 0 && cursor_y >= y + maxHeight)
            {
                continue;
            }

            if (c < font.first_char || c > font.last_char)
            {
                cursor_x += static_cast<int>(font.pixel_size) / 2;
                continue;
            }

            const uint16_t glyph_index = static_cast<uint16_t>(c - font.first_char);
            const FontGlyph& glyph = font.glyphs[glyph_index];
            drawCharacter(cursor_x, cursor_y, c, maxWidth >= 0 ? maxWidth - (cursor_x - x) : -1, maxHeight >= 0 ? maxHeight - (cursor_y - y) : -1, color, font);
            cursor_x += glyph.advance;
        }
    }

    void drawImage(int x, int y, const ImageAsset& image)
    {
        for (uint16_t j = 0; j < image.height; j++)
        {
            for (uint16_t i = 0; i < image.width; i++)
            {
                uint16_t byte_index = j * image.stride_bytes + (i / 8);
                uint8_t bit_index = 7 - (i % 8);
                bool pixel_on = (image.data[byte_index] >> bit_index) & 0x01;
                lcd.setPixel(x + i, y + j, pixel_on ? Color::White() : Color::None());
            }
        }
    }
};

#endif // GRAPHICS_H