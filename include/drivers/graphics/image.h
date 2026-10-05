#ifndef IMAGE_H
#define IMAGE_H

#include <cstddef>
#include <cstdint>

enum class ImageFormat : uint8_t
{
    Monochrome1Bpp = 1,
};

struct ImageAsset
{
    uint16_t width;
    uint16_t height;
    uint16_t stride_bytes;
    size_t data_size;
    ImageFormat format;
    const uint8_t* data;
};

#endif // IMAGE_H