import open3d as o3d
import os

def merge_pcd_files(directory_path, output_file):
    # 初始化一个空的点云对象
    merged_cloud = o3d.geometry.PointCloud()

    # 遍历文件夹中的所有 PCD 文件
    for filename in os.listdir(directory_path):
        if filename.endswith('.pcd'):
            # 读取每个点云文件
            file_path = os.path.join(directory_path, filename)
            cloud = o3d.io.read_point_cloud(file_path)

            # 将当前点云加入到累积的点云中
            merged_cloud += cloud

    # 保存合并后的点云
    o3d.io.write_point_cloud(output_file, merged_cloud)
    print(f"Merged point cloud saved as '{output_file}'.")

# 使用函数
directory_path = '/path/to/your/pcd_files'
output_file = '/path/to/your/output/merged_cloud.pcd'
merge_pcd_files(directory_path, output_file)
