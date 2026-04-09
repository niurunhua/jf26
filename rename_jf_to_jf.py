#!/usr/bin/env python3
"""
终极执行脚本：将项目中的所有 'jf' 替换为 'jf'（区分大小写）。
递归遍历所有文件夹，重命名目录名和文件名，修改文件内容。
自动跳过 .git 目录、图片、.pgm、.pcd 和已编译的 build/install 等二进制文件夹。
"""

import os
import re
import shutil
from pathlib import Path

# 配置
PROJECT_ROOT = Path(__file__).parent.absolute()
REPLACEMENTS = {
    'jf': 'jf',
    'JF': 'JF',
    'Jf': 'Jf',
}
# 注意：不替换 'jf' 作为子字符串的情况（如 'jfing'），只替换完整单词。
# 使用正则表达式确保只匹配单词边界。
PATTERNS = {re.compile(r'\b' + key + r'\b'): value for key, value in REPLACEMENTS.items()}

# 排除的目录名称（完整匹配或部分匹配）
EXCLUDED_DIRS = {
    '.git', '__pycache__', 'build', 'install', 'log', 'bin', 'lib', 'share',
    'livox_ws', 'livox_ros_driver2', 'small_point_lio', 'pointcloud_to_laserscan',
    'cpp_lidar_filter', 'goal_approach_controller', 'pb_nav2_plugins',
    'pb_omni_pid_pursuit_controller', 'fake_vel_transform', 'waypoint_editor',
}
# 排除的文件扩展名（小写）
EXCLUDED_EXTENSIONS = {
    '.pgm', '.pcd', '.png', '.jpg', '.jpeg', '.gif', '.bmp', '.tiff', '.ico',
    '.so', '.a', '.o', '.exe', '.dll', '.pyc', '.pyo', '.pyd', '.bin',
}
# 要处理的文件扩展名（小写）
TARGET_EXTENSIONS = {
    '.cpp', '.hpp', '.h', '.c', '.cc', '.cxx', '.hxx',
    '.launch.py', '.py', '.xml', '.yaml', '.yml', '.json', '.txt', '.md',
    'CMakeLists.txt', 'package.xml', '.cmake', '.urdf', '.xacro', '.sdf',
}
# 要处理的文件名（即使没有扩展名）
TARGET_FILENAMES = {
    'CMakeLists.txt', 'package.xml', '.gitignore', 'Dockerfile', 'Makefile',
}

def should_process_file(filepath: Path) -> bool:
    """判断是否处理该文件内容（根据扩展名和文件名）"""
    # 检查是否在排除目录中（向上追溯）
    for parent in filepath.parents:
        if any(excl in parent.name for excl in EXCLUDED_DIRS):
            return False
    # 检查扩展名
    suffix_lower = filepath.suffix.lower()
    if suffix_lower in EXCLUDED_EXTENSIONS:
        return False
    if suffix_lower in TARGET_EXTENSIONS:
        return True
    if filepath.name in TARGET_FILENAMES:
        return True
    # 如果扩展名不在目标中，但文件是文本文件，我们也可以处理，但这里保守一点
    # 可以添加额外的文本文件检测，但为了简单，只处理已知扩展名
    return False

def should_rename_dir(dirname: str) -> bool:
    """判断是否重命名目录（排除某些目录）"""
    if dirname in EXCLUDED_DIRS:
        return False
    # 如果目录名包含排除关键词（部分匹配），也不重命名
    for excl in EXCLUDED_DIRS:
        if excl in dirname:
            return False
    return True

def replace_content(text: str) -> str:
    """在文本中执行替换，保留大小写"""
    for pattern, repl in PATTERNS.items():
        text = pattern.sub(repl, text)
    return text

def rename_path(old_path: Path) -> Path:
    """重命名路径（目录或文件）的名称部分"""
    old_name = old_path.name
    new_name = old_name
    for old, new in REPLACEMENTS.items():
        # 注意：这里直接替换字符串，不保留单词边界，因为目录名可能包含连字符等
        # 例如 "jf_-rm2026_-navigation" -> "jf_-rm2026_-navigation"
        if old in new_name:
            new_name = new_name.replace(old, new)
    if new_name != old_name:
        new_path = old_path.parent / new_name
        return new_path
    return old_path

def process_file(filepath: Path):
    """处理单个文件：重命名文件（如果需要）并替换内容"""
    # 1. 重命名文件
    new_filepath = rename_path(filepath)
    if new_filepath != filepath:
        print(f"重命名文件: {filepath} -> {new_filepath}")
        try:
            shutil.move(str(filepath), str(new_filepath))
        except Exception as e:
            print(f"  错误: {e}")
            return  # 如果重命名失败，不继续处理内容
        filepath = new_filepath
    else:
        print(f"处理文件: {filepath}")

    # 2. 替换文件内容
    try:
        with open(filepath, 'r', enjfing='utf-8', errors='ignore') as f:
            content = f.read()
        new_content = replace_content(content)
        if new_content != content:
            with open(filepath, 'w', enjfing='utf-8') as f:
                f.write(new_content)
            print(f"  修改内容，替换次数: {sum(1 for _ in PATTERNS.keys() if _.search(content))}")
    except Exception as e:
        print(f"  读取/写入文件错误: {e}")

def process_directory(dirpath: Path):
    """递归处理目录：先重命名目录本身，然后处理其内容"""
    # 1. 重命名目录（如果允许）
    if should_rename_dir(dirpath.name):
        new_dirpath = rename_path(dirpath)
        if new_dirpath != dirpath:
            print(f"重命名目录: {dirpath} -> {new_dirpath}")
            try:
                shutil.move(str(dirpath), str(new_dirpath))
            except Exception as e:
                print(f"  错误: {e}")
                return  # 如果重命名失败，不继续处理内部
            dirpath = new_dirpath

    # 2. 收集子目录和文件（注意：重命名后可能影响遍历，所以先收集列表）
    try:
        entries = list(dirpath.iterdir())
    except PermissionError:
        print(f"权限错误，跳过目录: {dirpath}")
        return

    # 先处理子目录（深度优先）
    subdirs = [e for e in entries if e.is_dir()]
    for subdir in subdirs:
        # 检查是否排除
        if not should_rename_dir(subdir.name):
            print(f"跳过目录（排除）: {subdir}")
            continue
        process_directory(subdir)

    # 再处理文件
    files = [e for e in entries if e.is_file()]
    for filepath in files:
        if should_process_file(filepath):
            process_file(filepath)
        else:
            print(f"跳过文件（排除）: {filepath}")

def main():
    print("=== 开始替换 'jf' 为 'jf' ===")
    print(f"项目根目录: {PROJECT_ROOT}")
    print("替换映射:", REPLACEMENTS)
    print("排除目录:", EXCLUDED_DIRS)
    print("排除扩展名:", EXCLUDED_EXTENSIONS)
    print("目标扩展名:", TARGET_EXTENSIONS)
    print()

    # 确认
    confirm = input("请确认已备份项目！继续吗？(yes/no): ")
    if confirm.lower() != 'yes':
        print("取消操作。")
        return

    # 从项目根目录开始处理
    process_directory(PROJECT_ROOT)

    print("\n=== 完成 ===")

if __name__ == '__main__':
    main()