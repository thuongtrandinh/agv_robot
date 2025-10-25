#!/usr/bin/env python3

import os
import random
from PIL import Image, ImageDraw

def generate_random_map(output_path, width=400, height=400, num_obstacles=15):
    """
    Tạo một file bản đồ .pgm ngẫu nhiên với các chướng ngại vật.

    :param output_path: Đường dẫn để lưu file my_map.pgm
    :param width: Chiều rộng ảnh (pixels)
    :param height: Chiều cao ảnh (pixels)
    :param num_obstacles: Số lượng chướng ngại vật hình chữ nhật
    """
    # Tạo ảnh mới với nền trắng (không gian trống)
    img = Image.new('L', (width, height), color=255) # 'L' là chế độ grayscale, 255 là màu trắng
    draw = ImageDraw.Draw(img)

    # Vẽ các chướng ngại vật ngẫu nhiên
    for _ in range(num_obstacles):
        # Kích thước ngẫu nhiên cho chướng ngại vật
        obs_width = random.randint(20, 80)
        obs_height = random.randint(20, 80)
        
        # Vị trí ngẫu nhiên
        x1 = random.randint(0, width - obs_width)
        y1 = random.randint(0, height - obs_height)
        x2 = x1 + obs_width
        y2 = y1 + obs_height
        
        # Vẽ hình chữ nhật màu đen (chướng ngại vật)
        draw.rectangle([x1, y1, x2, y2], fill=0)

    # Lưu ảnh dưới định dạng PGM
    img.save(output_path)
    print(f"Đã tạo bản đồ ngẫu nhiên và lưu tại: {output_path}")

if __name__ == '__main__':
    map_file_path = os.path.join(os.path.dirname(__file__), 'maps', 'my_map.pgm')
    generate_random_map(map_file_path)