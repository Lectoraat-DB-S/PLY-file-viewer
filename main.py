import numpy as np
import open3d as o3d

def main():
    # Look at a PLY file
    pcd = o3d.io.read_point_cloud("Part1.PLY")                      # Read in the PLY file
    print(pcd)                                                      # Print the amount of points in the point cloud
    for x in pcd.points[0:20]:                                      # Print the first 20 points of the point cloud
        print(x)
        
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])
    

    
    # Look at a PLY file and show a section (sphere shape) of the point cloud
    pcd = o3d.io.read_point_cloud("Part1.PLY")                              # Read in the PLY file
    print(pcd)                                                              # Print the amount of points in the point cloud
    for x in pcd.points:                                                    # Print the points of the point cloud
        print(x)
        
    pointss = np.asarray(pcd.points)                                        # Safe the points of the old point cloud
    
    center = np.array([0, 0, 0])                                            # Centre point of the sphere
    radius = 21                                                             # Radius of the sphere
    
    distances = np.linalg.norm(pointss - center, axis=1)                    # Find all the points that fall within the sphere
    pcd.points = o3d.utility.Vector3dVector(pointss[distances <= radius])   # Overwrite the old point cloud with the new point cloud

    o3d.io.write_point_cloud("out.ply", pcd)                                # Write the new point cloud to a PLY file
    
    pcd2 = o3d.io.read_point_cloud("out.ply")                               # Read the new PLY file
    print(pcd2)                                                             # Print the amount of points in the point cloud
    for x in pcd2.points:                                                   # Print the points of the point cloud
        print(x)

    # Visualize the section of the point cloud
    o3d.visualization.draw_geometries([pcd2], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])

if __name__ == "__main__":
    main()
