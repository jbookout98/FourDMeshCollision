extends Node3D



var rigidbodies=[]

const MAX_ANGULAR_IMPULSE = 50.0  # Clamp maximum angular impulse (tweak as needed)
const ANGULAR_DAMPING = 0.3    # Damping factor per frame

func _ready():
	rigidbodies = get_children()
	for i in rigidbodies:
		i.randomizeVelocity()
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	# Update all rigid bodies’ positions, rotations, etc.
	for body in rigidbodies:
		body.update_state(delta)

	# Check collisions for each pair.
	for i in range(rigidbodies.size()):
		for j in range(i + 1, rigidbodies.size()):
			var collision_info = rigidbodies[i].check_collision_with_mtv(rigidbodies[j])
			if collision_info["colliding"]:
				print("Penetration depth: ", collision_info["penetration"])
				resolve_collision(rigidbodies[i], rigidbodies[j],collision_info["normal"], collision_info["penetration"])



func clamp_bivector(biv: Bivector) -> Bivector:
		return Bivector.new(
			clamp(biv.xy, -MAX_ANGULAR_IMPULSE, MAX_ANGULAR_IMPULSE),
			clamp(biv.xz, -MAX_ANGULAR_IMPULSE, MAX_ANGULAR_IMPULSE),
			clamp(biv.xw, -MAX_ANGULAR_IMPULSE, MAX_ANGULAR_IMPULSE),
			clamp(biv.yz, -MAX_ANGULAR_IMPULSE, MAX_ANGULAR_IMPULSE),
			clamp(biv.yw, -MAX_ANGULAR_IMPULSE, MAX_ANGULAR_IMPULSE),
			clamp(biv.zw, -MAX_ANGULAR_IMPULSE, MAX_ANGULAR_IMPULSE)
		)


# Converts a Bivector into a 6-element array.
func bivector_to_vector(biv: Bivector) -> Array:
	# Order: [xy, xz, xw, yz, yw, zw]
	return [biv.xy, biv.xz, biv.xw, biv.yz, biv.yw, biv.zw]

# Converts a 6-element array back to a Bivector.
func bivector_from_vector(arr: Array) -> Bivector:
	return Bivector.new(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5])

# Multiply a 6x6 matrix (as an Array of 6 Arrays) by a 6-element vector.
func multiply_matrix_vector(mat: Array, vec: Array) -> Array:
	var result = []
	for i in range(mat.size()):
		var sum = 0.0
		for j in range(vec.size()):
			sum += mat[i][j] * vec[j]
		result.append(sum)
	return result

# Assume each Rigidbody4D has a property "dimensions" of type Vector4,
# representing the lengths along each axis (L_x, L_y, L_z, L_w).
func compute_inertia_inverse(body: Rigidbody4D) -> Array:
	# If the body doesn't have dimensions defined, fall back to a default.
	var d = body.transform4d.mesh.scale 
	
	# For a 4D hypercube, a rough approximation is to assume the inertia about
	# each independent rotation (each bivector component) is proportional to m*(L²)/12.
	# We compute an average of the squares of the dimensions.
	var L_avg_sq = (d.x*d.x + d.y*d.y + d.z*d.z + d.w*d.w) / 4.0
	var I_val = body.mass * L_avg_sq / 12.0  # Approximate inertia for each rotation plane.
	
	# The inverse inertia for a diagonal tensor is just 1/I_val for each diagonal entry.
	var I_inv = []
	for i in range(6):
		var row = []
		for j in range(6):
			if i == j:
				row.append(1.0 / I_val if I_val != 0 else 0.0)
  # 
			else:
				row.append(0.0)
		I_inv.append(row)
	return I_inv


func resolve_collision(r1: Rigidbody4D, r2: Rigidbody4D, collision_normal: Vector4, penetration_depth: float) -> void:
	var restitution: float = 0.5
	var inv_mass1 = 1.0 / r1.mass
	var inv_mass2 = 1.0 / r2.mass
	
	# Approximate contact point.
	var pos1 = r1.get_position4d()
	var pos2 = r2.get_position4d()
	var contact_point = (pos1 + pos2) * 0.5
	
	# Compute relative velocity (including rotational contributions).
	var vA = r1.velocity + r1.get_point_velocity(contact_point)
	var vB = r2.velocity + r2.get_point_velocity(contact_point)
	var rel_vel = vB - vA
	var vel_along_normal = rel_vel.dot(collision_normal)
	if vel_along_normal > 0:
		return
	
	var effective_mass = inv_mass1 + inv_mass2
	var impulse_scalar = -(1 + restitution) * vel_along_normal / effective_mass
	var impulse = collision_normal * impulse_scalar
	print("Impulse: "+ str(impulse_scalar))
	# Update linear velocities.
	r1.velocity -= impulse * inv_mass1
	r2.velocity += impulse * inv_mass2
	
	# Compute lever arms.
	var rA = contact_point - pos1
	var rB = contact_point - pos2
	
	# Compute torque as wedge product.
	var torqueA = Bivector.wedge(rA, impulse)
	var torqueB = Bivector.wedge(rB, impulse)
	
	# Compute the inverse inertia tensor for each body.
	var I_inv1 = compute_inertia_inverse(r1)  # 6x6 matrix for r1
	var I_inv2 = compute_inertia_inverse(r2)  # 6x6 matrix for r2
	
	# Convert the torque bivectors to 6-element vectors.
	var torqueA_vec = bivector_to_vector(torqueA)
	var torqueB_vec = bivector_to_vector(torqueB)
	
	# Map torque through the inverse inertia tensor to get angular impulse vectors.
	var ang_impulseA_vec = multiply_matrix_vector(I_inv1, torqueA_vec)
	var ang_impulseB_vec = multiply_matrix_vector(I_inv2, torqueB_vec)
	
	# Convert the resulting 6-element vectors back to Bivectors.
	var angular_impulseA = bivector_from_vector(ang_impulseA_vec)
	var angular_impulseB = bivector_from_vector(ang_impulseB_vec)
	
	# Update angular velocities.
	r1.angularVelocity = r1.angularVelocity.add(angular_impulseA)
	r2.angularVelocity = r2.angularVelocity.subtract(angular_impulseB)
	
	# Positional correction (if needed).
	var percent: float = 0.2
	var slop: float = 0.01
	var correction_mag = max(penetration_depth - slop, 0) / (inv_mass1 + inv_mass2) * percent
	var correction = collision_normal * correction_mag
	r1.set_position4d(pos1 - correction * inv_mass1)
	r2.set_position4d(pos2 + correction * inv_mass2)
