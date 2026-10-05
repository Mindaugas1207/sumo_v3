#ifndef FONT_H
#define FONT_H

#include <cstddef>
#include <cstdint>

struct FontGlyph
{
    uint32_t bitmap_offset;
    uint16_t width;
    uint16_t height;
    int16_t x_offset;
    int16_t y_offset;
    uint16_t advance;
};

struct FontAsset
{
    uint16_t pixel_size;
    uint16_t line_height;
    char first_char;
    char last_char;
    const FontGlyph* glyphs;
    const uint8_t* bitmap;
    size_t bitmap_size;
};

#endif // FONT_H
