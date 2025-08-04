import os
import re
import shutil

def move_matched_images():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 定义images文件夹路径
    parent_images_dir = os.path.join(script_dir, '..', 'images')
    local_images_dir = os.path.join(script_dir, 'images')
    
    # 确保本地images文件夹存在
    os.makedirs(local_images_dir, exist_ok=True)
    
    # 正则表达式匹配图片引用
    img_pattern = re.compile(r'!\[img\]\(images/([^)]+)\)')
    
    # 遍历脚本目录下的所有.md文件
    for filename in os.listdir(script_dir):
        if filename.endswith('.md'):
            filepath = os.path.join(script_dir, filename)
            
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 查找所有匹配的图片引用
            matches = img_pattern.findall(content)
            
            # 移动匹配的图片文件
            for img_name in matches:
                src_path = os.path.join(parent_images_dir, img_name)
                dst_path = os.path.join(local_images_dir, img_name)
                
                if os.path.exists(src_path):
                    shutil.move(src_path, dst_path)
                    print(f'Moved: {src_path} -> {dst_path}')
                else:
                    print(f'Warning: Source image not found: {src_path}')

if __name__ == '__main__':
    move_matched_images()
    print("Image moving process completed.")