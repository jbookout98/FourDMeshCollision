@tool
extends Mesh4D
class_name HyperPlane

# Scale factors for the grid defining the hyperplane
@export var scale: Vector4 = Vector4(4, 4, 0, 4)
@export var resolution: int = 5  # Number of subdivisions along each axis

# Stores the vertices and indices of the hyperplane
var vertices: Array = []
var indices: Array = []

# Initializes the hyperplane by generating a grid of points
func init_mesh() -> void:
	vertices.clear()
	indices.clear()
	generate_vertices()
	generate_indices()

func generate_vertices() -> void:
	vertices = [
		Vector4(-scale.x, 0, -scale.z, -scale.w),
		Vector4(scale.x, 0, -scale.z, -scale.w),
		Vector4(scale.x, 0, scale.z, -scale.w),
		Vector4(-scale.x, 0, scale.z, -scale.w)
	]


func generate_indices() -> void:
	var size = vertices.size()
	for i in range(size):
		for j in range(i + 1, size):
			var diff_count = 0
			if abs(vertices[i].x - vertices[j].x) > 0: diff_count += 1
			if abs(vertices[i].y - vertices[j].y) > 0: diff_count += 1
			if abs(vertices[i].w - vertices[j].w) > 0: diff_count += 1
			if diff_count == 1:
				indices.append([i, j])

func get_vertices() -> Array:
	return vertices

func rotate_hyperplane(rotor: Rotor) -> Array:
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
	var M = 100.0
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

func get_projected_vertices(wSlice: float, rot: Rotor):
	var rotatedVerts = rotate_hyperplane(rot)
	var actualVerts = []
	for i in rotatedVerts:
		actualVerts.append(hyperplane_project_4d_to_3d(i, 500.0, wSlice))
	return actualVerts

func get_sliced_vertices(w_proj: float, rot: Rotor) -> Array:
	return get_projected_vertices(w_proj, rot)

func get_transformed_vertices(rotor: Rotor) -> Array:
	return rotate_hyperplane(rotor)

func get_sliced_vertices_with_indices(w_proj: float, rot: Rotor) -> Dictionary:
	var rotatedVerts = rotate_hyperplane(rot)
	var new_vertices = []
	var vertex_map = {}

	for v in rotatedVerts:
		var projected = apply_perspective_scaling(v, w_proj, 500)
		var key = str(snapped(projected.x, 0.01)) + "," + str(snapped(projected.y, 0.01)) + "," + str(snapped(projected.z, 0.01))
		if not vertex_map.has(key):
			vertex_map[key] = new_vertices.size()
			new_vertices.append(Vector3(projected.x, projected.y, projected.z))

	var triangle_indices = compute_convex_hull(new_vertices)
	return {"vertices": new_vertices, "indices": triangle_indices}

func compute_convex_hull(points: Array) -> Array:
	var hull_faces = []
	var face_keys = {}
	var n = points.size()

	for i in range(n):
		for j in range(i + 1, n):
			for k in range(j + 1, n):
				var p = points[i]
				var q = points[j]
				var r = points[k]
				var normal = (q - p).cross(r - p)
				if normal.length() < 0.0001:
					continue
				normal = normal.normalized()
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
				if all_positive or all_negative:
					var face = [i, j, k] if all_positive else [i, k, j]
					var sorted_face = face.duplicate()
					sorted_face.sort()
					var key = str(sorted_face)
					if not face_keys.has(key):
						face_keys[key] = true
						hull_faces.append(face)
	return hull_faces
