#!/usr/bin/env python3
"""
Generate QR codes for all grid nodes
Each QR code contains the node label and has the node name displayed as text below it
"""

import qrcode
from PIL import Image, ImageDraw, ImageFont
import os
from pathlib import Path


def get_grid_nodes(grid_size: int = 4):
    """
    Generate list of all grid nodes
    Mimics the structure from GridPathPlanner
    """
    nodes = []
    cols = [chr(65 + i) for i in range(grid_size)]  # A, B, C, D
    rows = [str(i + 1) for i in range(grid_size)]   # 1, 2, 3, 4
    
    # Main grid nodes (intersections)
    for col in cols:
        for row in rows:
            nodes.append(f"{col}{row}")
    
    # Intermediate nodes on vertical lines (between rows)
    for col in cols:
        for j in range(grid_size - 1):
            row1 = str(j + 1)
            row2 = str(j + 2)
            nodes.append(f"{col}{row1}{row2}")
    
    # Intermediate nodes on horizontal lines (between columns)
    for i in range(grid_size - 1):
        col1 = chr(65 + i)
        col2 = chr(65 + i + 1)
        for row in rows:
            nodes.append(f"{col1}{col2}{row}")
    
    # Dock nodes (extending upward from horizontal intermediate nodes, except row 1)
    for i in range(grid_size - 1):
        col1 = chr(65 + i)
        col2 = chr(65 + i + 1)
        for j in range(grid_size - 1):  # Skip row 1
            row = str(j + 2)
            horiz_label = f"{col1}{col2}{row}"
            nodes.append(f"DOC-{horiz_label}")
    
    # Home nodes extending to the right
    for row in rows:
        nodes.append(f"HOME-{row}")
    
    return sorted(set(nodes))


def generate_qr_code_with_label(node_name: str, output_path: str):
    """
    Generate a QR code with the node name embedded and as text label
    
    Args:
        node_name: The node identifier (e.g., 'A1', 'B2')
        output_path: Path to save the image
    """
    # QR code parameters
    qr_size = 10  # Size in pixels per module
    border_size = 2
    
    # Generate QR code with the node name embedded
    qr = qrcode.QRCode(
        version=1,  # Controls the size of the QR code
        error_correction=qrcode.constants.ERROR_CORRECT_H,  # High error correction
        box_size=qr_size,
        border=border_size,
    )
    qr.add_data(node_name)
    qr.make(fit=True)
    
    # Create QR image - PIL Image object in RGB mode
    qr_img = qr.make_image(fill_color="black", back_color="white").convert('RGB')
    
    # Calculate dimensions for final image with text
    text_height = 100  # Space for text at bottom
    final_width = qr_img.width
    final_height = qr_img.height + text_height
    
    # Create final image with white background
    final_img = Image.new('RGB', (final_width, final_height), color='white')
    
    # Paste QR code at the top
    final_img.paste(qr_img, box=(0, 0))
    
    # Add text label below QR code
    draw = ImageDraw.Draw(final_img)
    
    # Try to use a larger font, fallback to default if not available
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf", 60)
    except:
        font = ImageFont.load_default()
    
    # Draw black text with node name
    text_bbox = draw.textbbox((0, 0), node_name, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_x = (final_width - text_width) // 2  # Center horizontally
    text_y = qr_img.height + 15  # Below QR code
    
    draw.text((text_x, text_y), node_name, fill='black', font=font)
    
    # Save image
    final_img.save(output_path)
    print(f"✓ Generated: {output_path}")


def main():
    """main function to generate all QR codes"""
    
    # Create output directory
    output_dir = Path(__file__).parent / "qr_codes_nodes"
    output_dir.mkdir(exist_ok=True)
    
    print(f"📁 QR codes will be saved in: {output_dir}\n")
    
    # Get all nodes
    nodes = get_grid_nodes(grid_size=4)
    
    print(f"🔄 Generating QR codes for {len(nodes)} nodes...\n")
    
    # Generate QR code for each node
    for node_name in nodes:
        output_file = output_dir / f"{node_name}.png"
        try:
            generate_qr_code_with_label(node_name, str(output_file))
        except Exception as e:
            print(f"✗ Error generating QR for {node_name}: {e}")
    
    print(f"\n✅ All QR codes generated successfully!")
    print(f"📂 Total files: {len(list(output_dir.glob('*.png')))}")
    print(f"📍 Location: {output_dir}")


if __name__ == '__main__':
    main()
