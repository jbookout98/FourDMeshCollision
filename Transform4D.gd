@tool
extends Node3D

class_name Transform4D

@export var rotor: Rotor
@export var translation: Vector4 = Vector4()
@export var mesh: HyperCube = HyperCube.new()
@export var mesh_instance: MeshInstance3D = null

var last_translation: Vector4 = Vector4()
@export var last_rotation: Rotor = Rotor.new()
var rotTest:Rotor
# Constructor
func _init(rotor: Rotor = Rotor.new(), translation: Vector4 = Vector4()):
	self.rotor = rotor
	rotTest=Rotor.new()
	self.translation = translation
	
# Apply the transform to a 4D vector
func transform_vector(vector: Vector4) -> Vector4:
	return rotor.rotate(vector) + translation
func _ready():
	mesh.init_hypercube()
# Inverse transform: inverse rotor and inverse translation
func inverse() -> Transform4D:
	var inv_rotation = Rotor.new(rotor.s, -rotor.xy, -rotor.xz, -rotor.xw, -rotor.yz, -rotor.yw, -rotor.zw)
	var inv_translation = -inv_rotation.rotate(translation)
	return Transform4D.new(inv_rotation, inv_translation)

# Combine with another Transform4D
func multiply(other: Transform4D) -> Transform4D:
	var new_rotation = rotor.multiply(other.rotor)
	var new_translation = rotor.rotate(other.translation) + translation
	return Transform4D.new(new_rotation, new_translation)

# Static method to create a Transform4D from an angle, plane of rotor, and translation
static func from_angle_plane_translation(angle: float, plane: String, translation: Vector4) -> Transform4D:
	var rotor_instance = Rotor.new()
	var rotor = rotor_instance.from_angle_plane(angle, plane)
	return Transform4D.new(rotor, translation)

# Utility function to apply the transform to an array of 4D vectors
func transform_vectors(vectors: Array) -> Array:
	var transformed = []
	for vector in vectors:
		transformed.append(transform_vector(vector))
	return transformed

# Check if two rotors are equivalent when normalized
func is_equal_rotor(rotor1: Rotor, rotor2: Rotor, tol: float = 1e-6) -> bool:
	
	if rotor1.equals(rotor2):
		return true
	return false

# Interpolate between two 4D vectors along the w-axis (for slicing)
func interpolate(v1: Vector4, v2: Vector4, w_plane: float) -> Vector4:
	if abs(v2.w - v1.w) < 1e-6:
		return v1
	var t = (w_plane - v1.w) / (v2.w - v1.w)
	return v1.lerp(v2, t)

# Projects a 4D vector into 3D space using perspective division (ignoring further projection math)
func project_to_3d(v: Vector4, distance: float) -> Vector3:
	var w_factor = 1.0 / (distance - v.w)
	return Vector3(v.x * w_factor, v.y * w_factor, v.z * w_factor)

# Helper function: get or create a vertex index from a 4D vector.
# It adds the vertex (projected to 3D) into the positions array if not already present.



# Generates a mesh using ImmediateMesh from the projected vertices and edges.
# Dictionary to track spawned spheres
var spawned_spheres = []





# A vertex is visible if its w coordinate is less than or equal to w_threshold.
# Returns an ImmediateMesh representing the visible portion of the hypercube,
# Helper function to compute the normal of a triangle (using v0, v1, v2)
func compute_normal(v0: Vector3, v1: Vector3, v2: Vector3) -> Vector3:
	return (v0-v1).cross(v2 - v0).normalized()
	

	
func generate_mesh_from_sliced_vertices(slice_data: Dictionary):
	# Remove previous mesh instance if it exists
	if mesh_instance:
		remove_child(mesh_instance)
		mesh_instance.queue_free()

	# Create a new MeshInstance3D
	mesh_instance = MeshInstance3D.new()
	add_child(mesh_instance)

	# Extract data from slice_data
	var vertices_3d = slice_data["vertices"]
	var triangle_indices = slice_data["indices"]
	if vertices_3d.size()>2:

		# Prepare mesh arrays
		var mesh = ArrayMesh.new()
		var arrays = []
		arrays.resize(Mesh.ARRAY_MAX)

		var vertex_array = PackedVector3Array()
		var normal_array = PackedVector3Array()
		var index_array = PackedInt32Array()

		# For each triangle, compute a normal and add vertices
		for tri in triangle_indices:
			var v0 = vertices_3d[tri[0]]
			var v1 = vertices_3d[tri[1]]
			var v2 = vertices_3d[tri[2]]
			var normal = (v1 - v0).cross(v2 - v0).normalized()
			
			vertex_array.append(v0)
			vertex_array.append(v1)
			vertex_array.append(v2)
			
			normal_array.append(normal)
			normal_array.append(normal)
			normal_array.append(normal)
			
			index_array.append(vertex_array.size() - 3)
			index_array.append(vertex_array.size() - 2)
			index_array.append(vertex_array.size() - 1)

		arrays[Mesh.ARRAY_VERTEX] = vertex_array
		arrays[Mesh.ARRAY_NORMAL] = normal_array
		arrays[Mesh.ARRAY_INDEX] = index_array

		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		mesh_instance.mesh = mesh

# Update the mesh if translation or rotation changes.
func update_mesh():
	if mesh_instance == null:
		print("Generating MeshInstance3D")
		mesh_instance = MeshInstance3D.new()
		add_child(mesh_instance)
	
	# Check if translation or rotation changed
	if translation != last_translation or not rotor.betterEquals(last_rotation):

		var sliced_vertices = mesh.get_sliced_vertices_with_indices(translation.w, rotor)  # Get updated vertices
		generate_mesh_from_sliced_vertices(sliced_vertices) 
		
		self.position = Vector3(translation.x, translation.y, translation.z)
		
		last_translation = translation
		last_rotation = rotor.duplicate()
		





func _process(delta):
	update_mesh()
	
