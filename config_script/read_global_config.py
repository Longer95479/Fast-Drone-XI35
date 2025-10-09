# coding=UTF-8                                                                                                                                                                                                                         

# Example:
# python3 read_global_config.py global_config_outdoor.yaml

import os
import re
import yaml
import argparse

def load_replacements_from_yaml_as_tuples(yaml_file):
    """从 YAML 文件中加载路径和替换内容，以指定格式返回"""
    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)
    
    # 获取路径前缀
    path_prefix = config.get('path_prefix', '') 

    path_replacements = {}
    for path, replacements_list in config.get('paths', {}).items():
        # 使用路径前缀拼接完整路径
        full_path = os.path.join(path_prefix, path)
    
        # 初始化每个路径的替换列表
        path_replacements[full_path] = []
    
        # 遍历替换字典并将其转换为 (search_pattern, replace_template) 的元组形式
        for replacement_dict in replacements_list:
            for search_pattern, replace_template in replacement_dict.items():
                path_replacements[full_path].append((search_pattern, replace_template))
    
    return path_replacements



def replace_in_file(file_path, search_pattern, replace_template):
    """在指定文件中查找符合模式的文本并替换，同时输出原配置和新配置"""
    with open(file_path, 'r') as file:
        file_data = file.read()
    
    # 查找匹配的所有项
    matches = re.findall(search_pattern, file_data)
    if matches:
        # 输出原配置
        for match in matches:
            print("  {" + match + "}")
        
        # # 替换匹配项
        # new_data = re.sub(search_pattern, replace_template, file_data)
        
        # # 输出修改后的配置
        # new_matches = re.findall(replace_template, new_data)
        # for new_match in new_matches:
        #     print("Modified configuration in {" + file_path + "}: {" + new_match + "}")
        
        # # 如果文件内容有更改，重新写入文件
        # if new_data != file_data:
        #     with open(file_path, 'w') as file:
        #         file.write(new_data)
        #     print("Replaced in {" + file_path + "}\n")
        # else:
        #     print("No changes made in {" + file_path + "}\n")
    else:
        print("No matches found in {" + file_path + "}\n")

def batch_replace_in_paths(path_replacements):
    # 遍历指定路径中的文件，查找符合模式的文本并替换
    for path, replacements in path_replacements.items():
        if os.path.isfile(path):
            print("Original configuration in {" + path + "}:")
            # 如果是文件，直接替换内容
            for search_pattern, replace_template in replacements:
                replace_in_file(path, search_pattern, replace_template)
            print("\n")
        elif os.path.isdir(path):
            # 如果是目录，递归查找其中的文件
            for root, _, files in os.walk(path):
                for file_name in files:
                    file_path = os.path.join(root, file_name)
                    print("Original configuration in {" + file_path + "}:")
                    for search_pattern, replace_template in replacements:
                        replace_in_file(file_path, search_pattern, replace_template)
                    print("\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="加载 YAML 文件并解析路径和替换内容")
    parser.add_argument("yaml_file", type=str, help="YAML 文件路径")
    args = parser.parse_args()

    # 从命令行参数读取 YAML 文件路径
    yaml_file = args.yaml_file
    path_replacements = load_replacements_from_yaml_as_tuples(yaml_file)

    # print(path_replacements)
    # print("\n")

    # 执行批量替换
    batch_replace_in_paths(path_replacements)