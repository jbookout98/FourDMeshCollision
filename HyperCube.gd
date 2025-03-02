@tool
extends Mesh4D
class_name HyperCube

# Scale factors for each axis (x, y, z, and w).
@export var scale: Vector4 = Vector4(4, 4, 8, 8)

# Array to hold the 4D vertices of the hypercube (tesseract).
var vertices: Array = []
# Array to hold edge connectivity (each edge is an Array of two indices)
var indices: Array = []

# Initializes the hypercube vertices.
# 'scale' is used to define the vertex positions as ±scale.x, ±scale.y, etc.
func init_hypercube() -> void:
	vertices.clear()
	# Generate vertices at all combinations of -scale and scale for each axis.
	for x in [-1,1]:
		for y in [-1,1]:
			for z in [-1,1]:
				for w in [-1,1]:
					var v = Vector4(x*scale.x, y*scale.y, z*scale.z, w*scale.w)
					vertices.append(v)
	createIndices() 

func createIndices():
	
	for i in range(vertices.size()):
		for j in range(i + 1, vertices.size()):
			var diff_count=0
			if abs(vertices[i].x - vertices[j].x) > 0: diff_count += 1
			if abs(vertices[i].y - vertices[j].y) > 0: diff_count += 1
			if abs(vertices[i].z - vertices[j].z) > 0: diff_count += 1
			if abs(vertices[i].w - vertices[j].w) > 0: diff_count += 1
			if diff_count == 1:
					indices.append([i, j])

func get_vertices():
	return vertices
func rotate_hypercube(rotor: Rotor) -> Array:
	var rotated_vertices = []
	for v in vertices:
		rotated_vertices.append(rotor.rotate(v))
	return rotated_vertices
func perspective_project_4d_to_3d(point_4d: Vector4, d: float = 500.0) -> Vector3:

	var scale_factor = d / (d - point_4d.w) 
	return Vector3(
		point_4d.x * scale_factor,
		point_4d.y * scale_factor,
		point_4d.z * scale_factor
	)
func apply_perspective_scaling(point3d: Vector4, effective_w: float, d: float) -> Vector3:
	# Here, we choose a parameter M that controls the effect.
	# When |effective_w| is zero, scale is maximal, and it decreases symmetrically as |effective_w| increases.
	var M = 100.0  # You can tweak this constant for your desired zoom effect.
	var scale_factor = M / (M + abs(effective_w))
	return Vector3(
		point3d.x * scale_factor,
		point3d.y * scale_factor,
		point3d.z * scale_factor
	)

func hyperplane_project_4d_to_3d(point_4d: Vector4, eye_w: float = 50.0, w_proj: float = 0.0) -> Vector3:
	var t = (w_proj - eye_w) / (point_4d.w - eye_w)
	return Vector3(
		point_4d.x * t,
		point_4d.y * t,
		point_4d.z * t
	)
	

func get_projected_vertices(wSlice:float, rot:Rotor):
	
	var rotatedVerts = rotate_hypercube(rot)
	
	var actualVerts=[]
	for i in rotatedVerts:
		actualVerts.append(hyperplane_project_4d_to_3d(i,500.0,wSlice))
	
	return actualVerts
func get_sliced_vertices(w_proj: float, rot: Rotor) -> Array:
	var rotatedVerts = rotate_hypercube(rot)  # Rotate the hypercube first
	var new_vertices = []  # Store intersection points

	for edge in indices:
		var v1 = rotatedVerts[edge[0]]
		var v2 = rotatedVerts[edge[1]]

		# Check if the edge crosses the hyperplane at w_proj
		if (v1.w > w_proj and v2.w < w_proj) or (v1.w < w_proj and v2.w > w_proj):
			# Compute the interpolation factor t where w_proj is reached
			var t = (w_proj - v1.w) / (v2.w - v1.w)
			
			# Linearly interpolate to find the exact intersection point
			var intersection = Vector4(
				lerp(v1.x, v2.x, t),
				lerp(v1.y, v2.y, t),
				lerp(v1.z, v2.z, t),
				w_proj  # Ensure it lies exactly on the hyperplane
			)

			# Project to 3D
			var projected = hyperplane_project_4d_to_3d(intersection, 500.0, w_proj)
			new_vertices.append(projected)

	return new_vertices

func get_slice_intersection(v1: Vector4, v2: Vector4, w_proj: float) -> Dictionary:
	var t = (w_proj - v1.w) / (v2.w - v1.w)
	var intersect = Vector4(
		lerp(v1.x, v2.x, t),
		lerp(v1.y, v2.y, t),
		lerp(v1.z, v2.z, t),
		w_proj  # Force the w coordinate to be the slicing value.
	)
	return {"point": intersect, "t": t}

func get_transformed_vertices(rotor: Rotor) -> Array:
	return rotate_hypercube(rotor)  # Only apply rotation, no translation


func get_sliced_vertices_with_indices(w_proj: float, rot: Rotor) -> Dictionary:
	var rotatedVerts = rotate_hypercube(rot)  # Rotate the hypercube first
	var new_vertices = []  # Store unique intersection points
	var vertex_map = {}    # Map 3D position -> index in new_vertices

	for edge in indices:
		var v1 = rotatedVerts[edge[0]]
		var v2 = rotatedVerts[edge[1]]
		# Check if the edge crosses the hyperplane at w_proj
		if (v1.w > w_proj and v2.w < w_proj) or (v1.w < w_proj and v2.w > w_proj):
			# Compute interpolation factor t
			var t = (w_proj - v1.w) / (v2.w - v1.w)
			
			# Compute intersection point
			var intersection = Vector4(
				lerp(v1.x, v2.x, t),
				lerp(v1.y, v2.y, t),
				lerp(v1.z, v2.z, t),
				w_proj  # Ensure it lies on the hyperplane
			)

			# Project to 3D
			var projected = apply_perspective_scaling(intersection, w_proj,500)

			# Ensure uniqueness using a string key
			var key = str(snapped(projected.x, 0.01)) + "," + str(snapped(projected.y, 0.01)) + "," + str(snapped(projected.z, 0.01))
			if not vertex_map.has(key):
				vertex_map[key] = new_vertices.size()
				new_vertices.append(Vector3(projected.x, projected.y, projected.z))

	var triangle_indices = compute_convex_hull(new_vertices)

	
	return {"vertices": new_vertices, "indices": triangle_indices}



func compute_convex_hull(points: Array) -> Array:
	var hull_faces = []
	var face_keys = {}  # Used to avoid duplicates
	var n = points.size()
	
	# Brute-force: check every combination of three points
	for i in range(n):
		for j in range(i + 1, n):
			for k in range(j + 1, n):
				var p = points[i]
				var q = points[j]
				var r = points[k]
				
				# Compute the plane normal (if points are nearly colinear, skip)
				var normal = (q - p).cross(r - p)
				if normal.length() < 0.0001:
					continue
				normal = normal.normalized()
				
				# Check the sign of the distance for all other points
				var all_positive = true
				var all_negative = true
				for l in range(n):
					if l == i or l == j or l == k:
						continue
					var d = (points[l] - p).dot(normal)
					if d > 0.001:
						all_negative = false
					if d < -0.001:
						all_positive = false
				# If all points lie on one side (or on the plane), then (i,j,k) is a hull face.
				if all_positive or all_negative:
					# Choose ordering so that the face normal points outward.
					# (If all points are below the plane (all_negative), reverse the order.)
					var face = []
					if all_negative:
						face = [i, k, j]  # reversed order to flip normal
					else:
						face = [i, j, k]
					
					# Create a duplicate sorted copy for duplicate checking.
					var sorted_face = face.duplicate()
					sorted_face.sort()
					var key = str(sorted_face)
					if not face_keys.has(key):
						face_keys[key] = true
						hull_faces.append(face)
	return hull_faces
