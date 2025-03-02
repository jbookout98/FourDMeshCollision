@tool
extends Resource


class_name MeshGenerator4D

# Export mesh to see changes in the editor
@export var mesh: Mesh = null


# Generate vertices of a 4D hypercube
func generate_hypercube_vertices(transform: Transform4D, size: float = 1.0) -> Array:
	var vertices = []
	for x in [-size, size]:
		for y in [-size, size]:
			for z in [-size, size]:
				for w in [-size, size]:
					var vertex = Vector4(x, y, z, w)
					vertices.append(transform.transform_vector(vertex))
	return vertices

# Generate a mesh from 4D vertices based on the w dimension of the Transform's translation
