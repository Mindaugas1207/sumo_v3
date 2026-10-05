#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path


ASCII_FIRST = 32
ASCII_LAST = 126


@dataclass(frozen=True)
class GlyphData:
    bitmap_offset: int
    width: int
    height: int
    x_offset: int
    y_offset: int
    advance: int


@dataclass(frozen=True)
class FontData:
    symbol: str
    display_name: str
    pixel_size: int
    line_height: int
    glyphs: list[GlyphData]
    bitmap: bytes


def make_tight_variant(font: FontData) -> FontData:
    non_empty = [g for g in font.glyphs if g.width > 0 and g.height > 0]
    if not non_empty:
        return FontData(
            symbol=f"{font.symbol}_tight",
            display_name=f"{font.display_name}_tight",
            pixel_size=font.pixel_size,
            line_height=font.line_height,
            glyphs=font.glyphs,
            bitmap=font.bitmap,
        )

    min_y = min(g.y_offset for g in non_empty)
    max_bottom = max(g.y_offset + g.height for g in non_empty)
    tight_line_height = max(1, max_bottom - min_y)

    tight_glyphs = [
        GlyphData(
            bitmap_offset=g.bitmap_offset,
            width=g.width,
            height=g.height,
            x_offset=g.x_offset,
            y_offset=g.y_offset - min_y,
            advance=g.advance,
        )
        for g in font.glyphs
    ]

    return FontData(
        symbol=f"{font.symbol}_tight",
        display_name=f"{font.display_name}_tight",
        pixel_size=font.pixel_size,
        line_height=tight_line_height,
        glyphs=tight_glyphs,
        bitmap=font.bitmap,
    )


def sanitize_identifier(value: str) -> str:
    identifier = re.sub(r"[^0-9A-Za-z]+", "_", value).strip("_")
    if not identifier:
        identifier = "font"
    if identifier[0].isdigit():
        identifier = f"font_{identifier}"
    return identifier.lower()


def pack_bitmap(mask_pixels: list[int], width: int, height: int) -> bytes:
    stride_bytes = (width + 7) // 8
    packed = bytearray(stride_bytes * height)

    for y in range(height):
        row_offset = y * stride_bytes
        pixel_row = y * width
        for x in range(width):
            if mask_pixels[pixel_row + x] >= 128:
                packed[row_offset + (x // 8)] |= 1 << (7 - (x % 8))

    return bytes(packed)


def load_font(path: Path, pixel_size: int) -> FontData:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise RuntimeError(
            "Pillow is required for font conversion. Install it with: pip install pillow"
        ) from exc

    font = ImageFont.truetype(str(path), size=pixel_size)
    ascent, descent = font.getmetrics()
    line_height = max(1, ascent + descent)

    glyphs: list[GlyphData] = []
    packed_bitmap = bytearray()

    for codepoint in range(ASCII_FIRST, ASCII_LAST + 1):
        character = chr(codepoint)
        advance = max(1, int(math.ceil(font.getlength(character))))

        # Render with horizontal padding to preserve negative left bearings.
        horizontal_padding = pixel_size
        canvas_width = max(pixel_size * 3, advance + horizontal_padding * 2 + pixel_size)
        image = Image.new("L", (canvas_width, line_height), 0)
        draw = ImageDraw.Draw(image)
        draw.text((horizontal_padding, 0), character, fill=255, font=font)

        bbox = image.getbbox()
        if bbox is None:
            glyphs.append(
                GlyphData(
                    bitmap_offset=len(packed_bitmap),
                    width=0,
                    height=0,
                    x_offset=0,
                    y_offset=0,
                    advance=advance,
                )
            )
            continue

        cropped = image.crop(bbox)
        glyph_width = cropped.width
        glyph_height = cropped.height
        glyph_bitmap = pack_bitmap(list(cropped.tobytes()), glyph_width, glyph_height)

        glyphs.append(
            GlyphData(
                bitmap_offset=len(packed_bitmap),
                width=glyph_width,
                height=glyph_height,
                x_offset=bbox[0] - horizontal_padding,
                y_offset=bbox[1],
                advance=advance,
            )
        )
        packed_bitmap.extend(glyph_bitmap)

    symbol = f"{sanitize_identifier(path.stem)}_{pixel_size}"
    return FontData(
        symbol=symbol,
        display_name=f"{path.stem}_{pixel_size}",
        pixel_size=pixel_size,
        line_height=line_height,
        glyphs=glyphs,
        bitmap=bytes(packed_bitmap),
    )


def emit_fonts_header(output_path: Path, fonts: list[FontData]) -> None:
    lines: list[str] = [
        "#pragma once",
        "",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "#include \"font.h\"",
        "",
    ]

    if not fonts:
        lines.append("// No font assets were found under resources/fonts.")
        lines.append("")

    for font in fonts:
        bitmap_symbol = f"{font.symbol}_bitmap"
        glyph_symbol = f"{font.symbol}_glyphs"

        lines.append(f"inline constexpr std::uint8_t {bitmap_symbol}[] = {{")
        if font.bitmap:
            for offset in range(0, len(font.bitmap), 16):
                chunk = font.bitmap[offset : offset + 16]
                values = ", ".join(f"0x{byte:02X}" for byte in chunk)
                lines.append(f"    {values},")
        lines.append("};")
        lines.append("")

        lines.append(f"inline constexpr FontGlyph {glyph_symbol}[] = {{")
        for glyph in font.glyphs:
            lines.append(
                "    {"
                f"{glyph.bitmap_offset}, {glyph.width}, {glyph.height}, {glyph.x_offset}, {glyph.y_offset}, {glyph.advance}"
                "},"
            )
        lines.append("};")
        lines.append("")

        lines.append(
            f"inline constexpr FontAsset {font.symbol}{{"
            f"{font.pixel_size}, {font.line_height}, {ASCII_FIRST}, {ASCII_LAST}, {glyph_symbol}, {bitmap_symbol}, sizeof({bitmap_symbol})"
            "};"
        )
        lines.append("")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate C++ bitmap font headers")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--sizes", default="6,12,18,32")
    parser.add_argument("--tight-sizes", default="")
    parser.add_argument("inputs", nargs="*", type=Path)
    args = parser.parse_args()

    size_tokens = [token.strip() for token in str(args.sizes).split(",") if token.strip()]
    sizes = sorted({int(size) for size in size_tokens if int(size) > 0})
    tight_size_tokens = [token.strip() for token in str(args.tight_sizes).split(",") if token.strip()]
    tight_sizes = {int(size) for size in tight_size_tokens if int(size) > 0}

    fonts: list[FontData] = []
    for input_path in sorted(args.inputs):
        for pixel_size in sizes:
            loaded = load_font(input_path, pixel_size)
            fonts.append(loaded)
            if pixel_size in tight_sizes:
                fonts.append(make_tight_variant(loaded))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    emit_fonts_header(args.output, fonts)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
