#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import argparse
from tqdm import tqdm

def extract_files(source_dir, reference_dir, target_dir, extension=None):
    """
    根据参考文件夹中的文件名，从源文件夹中提取文件到目标文件夹
    
    参数:
        source_dir (str): 源文件夹路径，包含要提取的文件
        reference_dir (str): 参考文件夹路径，包含文件名列表
        target_dir (str): 目标文件夹路径，提取的文件将被复制到这里
        extension (str): 可选，指定要提取的文件扩展名
    """
    # 确保目标文件夹存在
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)
        print(f"创建目标文件夹: {target_dir}")
    
    # 获取参考文件夹中的文件名（不包含扩展名）
    reference_files = []
    for filename in os.listdir(reference_dir):
        if os.path.isfile(os.path.join(reference_dir, filename)):
            # 获取文件名（不含扩展名）
            base_name = os.path.splitext(filename)[0]
            reference_files.append(base_name)
    
    print(f"参考文件夹中找到 {len(reference_files)} 个文件")
    
    # 在源文件夹中查找匹配的文件
    matched_files = []
    for root, _, files in os.walk(source_dir):
        for filename in files:
            # 如果指定了扩展名，则只处理该扩展名的文件
            if extension and not filename.endswith(extension):
                continue
                
            base_name = os.path.splitext(filename)[0]
            
            # 检查文件名是否在参考列表中
            if base_name in reference_files:
                source_path = os.path.join(root, filename)
                matched_files.append((source_path, filename))
    
    print(f"在源文件夹中找到 {len(matched_files)} 个匹配文件")
    
    # 复制匹配的文件到目标文件夹
    copied_count = 0
    for source_path, filename in tqdm(matched_files, desc="复制文件"):
        target_path = os.path.join(target_dir, filename)
        try:
            shutil.copy2(source_path, target_path)
            copied_count += 1
        except Exception as e:
            print(f"复制文件 {filename} 时出错: {e}")
    
    print(f"成功复制 {copied_count} 个文件到 {target_dir}")

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="根据参考文件夹中的文件名从源文件夹中提取文件")
    parser.add_argument("-s", "--source", required=True, help="源文件夹路径")
    parser.add_argument("-r", "--reference", required=True, help="参考文件夹路径")
    parser.add_argument("-t", "--target", required=True, help="目标文件夹路径")
    parser.add_argument("-e", "--extension", help="要提取的文件扩展名（例如 .jpg）")
    
    args = parser.parse_args()
    
    # 执行文件提取
    extract_files(args.source, args.reference, args.target, args.extension)



source_dir = 'extracted_images/raw/left'
reference_dir='extracted_images/right'
target_dir='extracted_images/left'

extract_files(source_dir, reference_dir, target_dir, extension=None)

# if __name__ == "__main__":
#     main()
