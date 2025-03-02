# CollisionShape4D.gd
extends Node3D
class_name CollisionShape4D

var vertices: Array = []  # Original 4D vertices of the hypercube.
var indices: Array = []   # Connectivity info (edges, etc).
var edges: Array = []

# Preload the debug sphere scene (make sure DebugSphere.tscn exists in your project)
var debug_sphere_scene: PackedScene = preload("res://small_sphere.tscn")
func _ready():
	vertices = get_parent().get_node("Transform4D").mesh.get_vertices()
func get_transformed_vertices(rotor: Rotor) -> Array:
	
	var transformed = []
	
	var parent = get_parent()
	
	var pos3d = parent.global_position
	
	var trans4d = parent.get_node("Transform4D").translation.w
	
	var translation4d = Vector4(pos3d.x, pos3d.y, pos3d.z, trans4d)
		
	for v in vertices:
		
		var rotated = rotor.rotate(v)+translation4d
		# Apply the translation to the rotated vertex.
		transformed.append(rotated)
		
	return transformed

func get_candidate_axes(vertsA: Array, vertsB: Array) -> Array:
	var axes = []
	# Add the four primary global axes.
	axes.append(Vector4(1, 0, 0, 0))
	axes.append(Vector4(0, 1, 0, 0))
	axes.append(Vector4(0, 0, 1, 0))
	axes.append(Vector4(0, 0, 0, 1))
	
	# Optionally add candidate axes based on differences between vertices.
	# This helps capture directions that correspond to edges or face normals.
	for shape in [vertsA, vertsB]:
		for i in range(shape.size()):
			for j in range(i + 1, shape.size()):
				var diff = shape[j] - shape[i]
				if diff.length() > 0.0001:
					var axis = diff.normalized()
					axes.append(axis)
					
	# Remove nearly duplicate axes (axes pointing in nearly the same direction)
	axes = unique_axes(axes)
	return axes

func unique_axes(axes: Array) -> Array:
	var unique = []
	for a in axes:
		var found = false
		for u in unique:
			# If the dot product is close to ±1, then the axes are almost parallel.
			if abs(a.dot(u)) > 0.99:
				found = true
				break
		if not found:
			unique.append(a)
	return unique

func project_to_3d(v4: Vector4) -> Vector3:
	# No perspective scaling; simply return the 3D part.
	return Vector3(v4.x, v4.y, v4.z)


func debug_draw_vertices_as_spheres(rotor: Rotor):
	
	for child in get_children():
		child.queue_free()
	
	var transformed_vertices = get_transformed_vertices(rotor)
	
	for i in range(transformed_vertices.size()):
		var v4 = transformed_vertices[i]
		var sphere_instance = debug_sphere_scene.instantiate() as Node3D
		if sphere_instance == null:
			push_error("Failed to instantiate debug sphere for vertex %d!" % i)
			continue
		
		sphere_instance.position = project_to_3d(v4)
		
		sphere_instance.name = "debug_sphere_%d" % i
		add_child(sphere_instance)
func _process(_delta):
	# Use the rotor from Transform4D (assuming it's a direct child of the Rigidbody4D)
	debug_draw_vertices_as_spheres(get_parent().get_node("Transform4D").rotor)
