import numpy as np
import open3d as o3d
# from open3d import *

def main():
    # cloud = read_point_cloud("part1.ply") # Read the point cloud
    # cloud2 = open3d.io.read_point_cloud("part1.ply")
    # open3d.io.raw_geometries([cloud2]) # Visualize the point cloud     
    # open3d.visualization.draw_geometries(cloud2)


    # cloud = o3d.data.PLYPointCloud()
    # cloud2 = o3d.io.read_point_cloud(cloud.path)
    # o3d.visualization.draw_geometries([cloud2],
    #                                     zoom=0.3412,
    #                                     front=[0.4257, -0.2125, -0.8795],
    #                                     lookat=[2.6172, 2.0475, 1.532],
    #                                     up=[-0.0694, -0.9768, 0.2024])
    

    # cloud2 = o3d.io.read_point_cloud("Part1.ply")
    # o3d.visualization.draw_geometries([cloud2],
    #                                     zoom=0.3412,
    #                                     front=[0.4257, -0.2125, -0.8795],
    #                                     lookat=[2.6172, 2.0475, 1.532],
    #                                     up=[-0.0694, -0.9768, 0.2024])
    

    pcd = o3d.io.read_point_cloud("Part2.PLY")
    print(o3d.io.read_point_cloud("Part2.PLY"))
    for x in pcd.points[0:20]:
        print(x)
    o3d.visualization.draw_geometries([pcd], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
    

    pcd = o3d.io.read_point_cloud("Part2.PLY")
    print(o3d.io.read_point_cloud("Part2.PLY"))
    for x in pcd.points:
        print(x)
        
    pointss = np.asarray(pcd.points)
    
    center = np.array([0, 0, 0])
    radius = 21
    
    distances = np.linalg.norm(pointss - center, axis=1)
    pcd.points = o3d.utility.Vector3dVector(pointss[distances <= radius])

    o3d.io.write_point_cloud("out.ply", pcd)
    
    pcd2 = o3d.io.read_point_cloud("out.ply")
    print(pcd2)
    for x in pcd2.points:
        print(x)

    o3d.visualization.draw_geometries([pcd2], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])

if __name__ == "__main__":
    main()