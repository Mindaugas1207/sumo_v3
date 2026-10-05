#!/usr/bin/env python3

from __future__ import annotations

import argparse
import struct
import zlib
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ImageData:
    name: str
    width: int
    height: int
    stride_bytes: int
    payload: bytes


def sanitize_identifier(value: str) -> str:
    identifier = re.sub(r"[^0-9A-Za-z]+", "_", value).strip("_")
    if not identifier:
        identifier = "image"
    if identifier[0].isdigit():
        identifier = f"image_{identifier}"
    return identifier.lower()


def decode_png(path: Path) -> ImageData:
    raw = path.read_bytes()
    png_signature = b"\x89PNG\r\n\x1a\n"
    if not raw.startswith(png_signature):
        raise RuntimeError(f"{path.name}: unsupported image format; use PNG or PBM/PGM/PPM")

    offset = len(png_signature)
    width = height = bit_depth = color_type = None
    idat_chunks: list[bytes] = []

    while offset + 8 <= len(raw):
        chunk_length = struct.unpack_from(">I", raw, offset)[0]
        offset += 4
        chunk_type = raw[offset : offset + 4]
        offset += 4
        chunk_data = raw[offset : offset + chunk_length]
        offset += chunk_length
        offset += 4  # CRC

        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, compression, filter_method, interlace = struct.unpack(
                ">IIBBBBB", chunk_data
            )
            if compression != 0 or filter_method != 0 or interlace != 0:
                raise RuntimeError(f"{path.name}: unsupported PNG compression, filter, or interlace mode")
        elif chunk_type == b"IDAT":
            idat_chunks.append(chunk_data)
        elif chunk_type == b"IEND":
            break

    if width is None or height is None or bit_depth is None or color_type is None:
        raise RuntimeError(f"{path.name}: missing IHDR chunk")
    if bit_depth != 8:
        raise RuntimeError(f"{path.name}: only 8-bit PNG files are supported")

    channels_by_color_type = {
        0: 1,
        2: 3,
        4: 2,
        6: 4,
    }
    if color_type not in channels_by_color_type:
        raise RuntimeError(f"{path.name}: unsupported PNG color type {color_type}")

    channels = channels_by_color_type[color_type]
    bytes_per_pixel = channels
    stride = width * channels
    inflated = zlib.decompress(b"".join(idat_chunks))
    expected_size = height * (1 + stride)
    if len(inflated) < expected_size:
        raise RuntimeError(f"{path.name}: truncated PNG image data")

    def unfilter_scanline(filter_type: int, current: bytearray, previous: bytes) -> None:
        if filter_type == 0:
            return
        if filter_type == 1:
            for index in range(len(current)):
                left = current[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
                current[index] = (current[index] + left) & 0xFF
            return
        if filter_type == 2:
            for index in range(len(current)):
                current[index] = (current[index] + previous[index]) & 0xFF
            return
        if filter_type == 3:
            for index in range(len(current)):
                left = current[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
                up = previous[index]
                current[index] = (current[index] + ((left + up) >> 1)) & 0xFF
            return
        if filter_type == 4:
            for index in range(len(current)):
                left = current[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
                up = previous[index]
                up_left = previous[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
                predictor = paeth_predictor(left, up, up_left)
                current[index] = (current[index] + predictor) & 0xFF
            return
        raise RuntimeError(f"{path.name}: unsupported PNG filter {filter_type}")

    grayscale_pixels: list[int] = []
    cursor = 0
    previous_row = bytes(stride)
    for _row in range(height):
        filter_type = inflated[cursor]
        cursor += 1
        row_data = bytearray(inflated[cursor : cursor + stride])
        cursor += stride
        unfilter_scanline(filter_type, row_data, previous_row)
        previous_row = bytes(row_data)

        if color_type == 0:
            grayscale_pixels.extend(row_data)
        elif color_type == 2:
            for index in range(0, len(row_data), 3):
                red = row_data[index]
                green = row_data[index + 1]
                blue = row_data[index + 2]
                grayscale_pixels.append((red * 299 + green * 587 + blue * 114) // 1000)
        elif color_type == 4:
            for index in range(0, len(row_data), 2):
                grayscale_pixels.append(row_data[index])
        elif color_type == 6:
            for index in range(0, len(row_data), 4):
                red = row_data[index]
                green = row_data[index + 1]
                blue = row_data[index + 2]
                alpha = row_data[index + 3]
                gray = (red * 299 + green * 587 + blue * 114) // 1000
                grayscale_pixels.append(255 if alpha < 128 else gray)

    stride_bytes = (width + 7) // 8
    payload = bytearray(stride_bytes * height)
    for row in range(height):
        for column in range(width):
            pixel = grayscale_pixels[row * width + column]
            if pixel < 128:
                payload[row * stride_bytes + (column // 8)] |= 1 << (7 - (column % 8))

    return ImageData(path.stem, width, height, stride_bytes, bytes(payload))


def paeth_predictor(left: int, up: int, up_left: int) -> int:
    estimate = left + up - up_left
    left_distance = abs(estimate - left)
    up_distance = abs(estimate - up)
    up_left_distance = abs(estimate - up_left)
    if left_distance <= up_distance and left_distance <= up_left_distance:
        return left
    if up_distance <= up_left_distance:
        return up
    return up_left


def load_image(path: Path) -> ImageData:
    suffix = path.suffix.lower()
    if suffix == ".png":
        return decode_png(path)

    if suffix in {".pbm", ".pgm", ".ppm"}:
        raise RuntimeError(f"{path.name}: Netpbm inputs are no longer supported; use PNG")

    raise RuntimeError(f"{path.name}: unsupported image format; use PNG")


def emit_header(images: list[ImageData], output_path: Path) -> None:
    lines: list[str] = [
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "#include \"image.h\"",
        "",
    ]

    if not images:
        lines.append("// No image assets were found under resources/images.")
        lines.append("")

    for image in images:
        symbol = sanitize_identifier(image.name)
        data_symbol = f"{symbol}_data"

        lines.append(f"inline constexpr std::uint8_t {data_symbol}[] = {{")
        for offset in range(0, len(image.payload), 12):
            chunk = image.payload[offset : offset + 12]
            hex_values = ", ".join(f"0x{byte:02X}" for byte in chunk)
            lines.append(f"    {hex_values},")
        lines.append("};")
        lines.append(
            f"inline constexpr ImageAsset {symbol}{{{image.width}, {image.height}, {image.stride_bytes}, sizeof({data_symbol}), ImageFormat::Monochrome1Bpp, {data_symbol}}};"
        )
        lines.append("")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate C++ image headers")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("inputs", nargs="*", type=Path)
    args = parser.parse_args()

    images = [load_image(path) for path in sorted(args.inputs)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    emit_header(images, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())