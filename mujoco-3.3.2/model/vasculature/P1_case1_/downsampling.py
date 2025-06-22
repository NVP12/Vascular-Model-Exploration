import open3d as o3d

# 1. Read
mesh = o3d.io.read_triangle_mesh('P1_case1_.stl')

# 2. Decide target face count
target_faces = min(len(mesh.triangles), 200_000)

# 3. Simplify
dec_mesh = mesh.simplify_quadric_decimation(target_faces)

# 4. Write out
o3d.io.write_triangle_mesh('P1_case1_decimated.stl', dec_mesh)

print("Original face count:", len(mesh.triangles))
print("New   face count:", len(dec_mesh.triangles))

