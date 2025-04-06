extends Node3D



var rigidbodies=[]

const MAX_ANGULAR_IMPULSE = 50.0  # Clamp maximum angular impulse (tweak as needed)
const ANGULAR_DAMPING = 0.3    # Damping factor per frame

func _ready():
	rigidbodies=get_children()
	for body in rigidbodies:
		body.randomizeVelocity()
	pass
	
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	var dynamic_bodies = rigidbodies.filter(func(b): return not b.staticRigidbody)
	var static_bodies = rigidbodies.filter(func(b): return b.staticRigidbody)
	
	for body in dynamic_bodies:
		body.resolved_this_frame = false  # Reset flag
		addGravityToEachCollider(body, delta)
		body.update_state(delta)
	
	# Dynamic vs. dynamic collisions
	for i in range(dynamic_bodies.size()):
		for j in range(i + 1, dynamic_bodies.size()):
			var collision_info = dynamic_bodies[i].check_collision_with_mtv(dynamic_bodies[j])
			if collision_info["colliding"] and not dynamic_bodies[i].resolved_this_frame:
				resolve_collision(dynamic_bodies[i], dynamic_bodies[j], collision_info["normal"], collision_info["penetration"])
				dynamic_bodies[i].resolved_this_frame = true
	
	# Dynamic vs. static collisions
	for d_body in dynamic_bodies:
		for s_body in static_bodies:
			var collision_info = d_body.check_collision_with_mtv(s_body)
			if collision_info["colliding"] and not d_body.resolved_this_frame:
				resolve_collision(d_body, s_body, collision_info["normal"], collision_info["penetration"])
				d_body.resolved_this_frame = true
				
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
func multiply_matrix_vector(matrix: Array, vector: Array) -> Array:
	if matrix.size() != 36 or vector.size() != 6:
		return [0, 0, 0, 0, 0, 0]
	var result = []
	for i in range(6):
		var sum = 0.0
		for j in range(6):
			sum += matrix[i * 6 + j] * vector[j]
		result.append(sum)
	return result
	
func addGravityToEachCollider(body,delta):
	var gravity = Vector4(0, -98, 0, 0)  # Gravity along negative Y-axis
	body.velocity += gravity *delta
# Assume each Rigidbody4D has a property "dimensions" of type Vector4,
func compute_inertia_inverse(body: Rigidbody4D) -> Array:
	if body.staticRigidbody:
		return []  # Zero matrix for static bodies
	var mass = body.mass
	var dim = body.dimensions  # Vector4(L_x, L_y, L_z, L_w)
	# For a hypercuboid, diagonal inertia components (simplified):
	var I_xx = (1.0/12.0) * mass * (dim.y*dim.y + dim.z*dim.z + dim.w*dim.w)
	var I_yy = (1.0/12.0) * mass * (dim.x*dim.x + dim.z*dim.z + dim.w*dim.w)
	var I_zz = (1.0/12.0) * mass * (dim.x*dim.x + dim.y*dim.y + dim.w*dim.w)
	var I_ww = (1.0/12.0) * mass * (dim.x*dim.x + dim.y*dim.y + dim.z*dim.z)
	var I_inv_xx = 1.0 / I_xx if I_xx > 0 else 0.0
	var I_inv_yy = 1.0 / I_yy if I_yy > 0 else 0.0
	var I_inv_zz = 1.0 / I_zz if I_zz > 0 else 0.0
	var I_inv_ww = 1.0 / I_ww if I_ww > 0 else 0.0
	# Simplified 6x6 matrix (assuming symmetry and neglecting off-diagonal terms)
	return [
		I_inv_xx, 0, 0, 0, 0, 0,
		0, I_inv_xx, 0, 0, 0, 0,  # xy, xz, etc., use I_xx for simplicity
		0, 0, I_inv_xx, 0, 0, 0,
		0, 0, 0, I_inv_yy, 0, 0,
		0, 0, 0, 0, I_inv_yy, 0,
		0, 0, 0, 0, 0, I_inv_zz
	]

func dot_product(v1: Array, v2: Array) -> float:
	var sum = 0.0
	for i in range(v1.size()):
		sum += v1[i] * v2[i]
	return sum

# Resolves a collision between two 4D rigid bodies
func resolve_collision(r1: Rigidbody4D, r2: Rigidbody4D, collision_normal: Vector4, penetration_depth: float) -> void:
	var restitution = 0.1
	var mu_s = 0.8
	var mu_k = 0.2
	var velocity_threshold = 0.001

	var inv_mass1 = 0.0 if r1.staticRigidbody else 1.0 / r1.mass
	var inv_mass2 = 0.0 if r2.staticRigidbody else 1.0 / r2.mass

	var pos1 = r1.get_position4d()
	var pos2 = r2.get_position4d()
	var contact_point = pos1 + collision_normal * (penetration_depth * 0.5)
	var rA = contact_point - pos1
	var rB = contact_point - pos2

	var vA = r1.velocity + r1.get_point_velocity(contact_point)
	var vB = r2.velocity + r2.get_point_velocity(contact_point)
	var rel_vel = vB - vA
	var vel_along_normal = rel_vel.dot(collision_normal)

	if vel_along_normal > 0:
		return

	var I_inv1 = compute_inertia_inverse(r1)
	var I_inv2 = compute_inertia_inverse(r2)

	var r1_star_n = compute_r_star_n(rA, collision_normal)
	var r2_star_n = compute_r_star_n(rB, collision_normal)
	var term1 = dot_product(r1_star_n, multiply_matrix_vector(I_inv1, r1_star_n))
	var term2 = dot_product(r2_star_n, multiply_matrix_vector(I_inv2, r2_star_n))
	var K_normal = inv_mass1 + inv_mass2 + term1 + term2
	if K_normal <= 0:
		return

	var j_n = -(1 + restitution) * vel_along_normal / K_normal
	var impulse_normal = collision_normal * j_n

	apply_impulse(r1, -impulse_normal, rA, inv_mass1, I_inv1)
	apply_impulse(r2, impulse_normal, rB, inv_mass2, I_inv2)

	# Clamp velocity along normal
	vA = r1.velocity + r1.get_point_velocity(contact_point)
	vB = r2.velocity + r2.get_point_velocity(contact_point)
	rel_vel = vB - vA
	vel_along_normal = rel_vel.dot(collision_normal)
	if vel_along_normal < 0 and not r1.staticRigidbody:
		r1.velocity -= collision_normal * vel_along_normal

	# Friction (unchanged for brevity, adjust if needed)
	var v_tan = rel_vel - (rel_vel.dot(collision_normal)) * collision_normal
	var v_tan_mag = v_tan.length()
	if v_tan_mag > velocity_threshold:
		var tangent_basis = compute_tangent_basis(collision_normal)
		if tangent_basis.size() < 3:
			return
		var j_friction = Vector4(0, 0, 0, 0)
		for t_i in tangent_basis:
			var v_tan_i = v_tan.dot(t_i)
			var r1_star_t_i = compute_r_star_n(rA, t_i)
			var r2_star_t_i = compute_r_star_n(rB, t_i)
			var term1_i = dot_product(r1_star_t_i, multiply_matrix_vector(I_inv1, r1_star_t_i))
			var term2_i = dot_product(r2_star_t_i, multiply_matrix_vector(I_inv2, r2_star_t_i))
			var K_i = inv_mass1 + inv_mass2 + term1_i + term2_i
			if K_i > 0:
				var j_i = -v_tan_i / K_i
				j_friction += t_i * j_i
		var j_friction_mag = j_friction.length()
		var max_static_friction = mu_s * abs(j_n)
		var impulse_friction: Vector4
		if j_friction_mag <= max_static_friction:
			impulse_friction = j_friction
		else:
			impulse_friction = -mu_k * abs(j_n) * (v_tan / v_tan_mag)
		apply_impulse(r1, -impulse_friction, rA, inv_mass1, I_inv1)
		apply_impulse(r2, impulse_friction, rB, inv_mass2, I_inv2)

	# Enhanced penetration correction
	var correction_factor = 1.0
	if not r1.staticRigidbody:
		var correction = collision_normal * (penetration_depth * correction_factor)
		r1.set_position4d(r1.get_position4d() - correction)
	if not r2.staticRigidbody:
		var correction = collision_normal * (penetration_depth * correction_factor)
		r2.set_position4d(r2.get_position4d() + correction)
# Helper function to compute [r]_* n (bivector components in 4D)
func compute_r_star_n(r: Vector4, n: Vector4) -> Array:
	var x = r.x
	var y = r.y
	var z = r.z
	var w = r.w
	var nx = n.x
	var ny = n.y
	var nz = n.z
	var nw = n.w
	# Bivector components: xy, xz, xw, yz, yw, zw
	var b_xy = x * ny - y * nx
	var b_xz = x * nz - z * nx
	var b_xw = x * nw - w * nx
	var b_yz = y * nz - z * ny
	var b_yw = y * nw - w * ny
	var b_zw = z * nw - w * nz
	return [b_xy, b_xz, b_xw, b_yz, b_yw, b_zw]

# Helper function to compute dot product of two arrays
# Computes an orthonormal basis for the 3D tangent hyperplane using Gram-Schmidt
func compute_tangent_basis(normal: Vector4) -> Array:
	var basis = []
	# Choose an initial vector not parallel to normal
	var v1 = Vector4(1, 0, 0, 0) if abs(normal.x) < 0.9 else Vector4(0, 1, 0, 0)
	# Subtract the projection of v1 onto normal
	var t1 = v1 - project_vector4(v1, normal)
	if t1.length() > 0.001:
		t1 = t1.normalized()
		basis.append(t1)
		
		# Second initial vector
		var v2 = Vector4(0, 1, 0, 0) if abs(normal.y) < 0.9 else Vector4(0, 0, 1, 0)
		# Subtract projections onto normal and t1
		var t2 = v2 - project_vector4(v2, normal) - project_vector4(v2, t1)
		if t2.length() > 0.001:
			t2 = t2.normalized()
			basis.append(t2)
			
			# Third initial vector
			var v3 = Vector4(0, 0, 1, 0) if abs(normal.z) < 0.9 else Vector4(0, 0, 0, 1)
			# Subtract projections onto normal, t1, and t2
			var t3 = v3 - project_vector4(v3, normal) - project_vector4(v3, t1) - project_vector4(v3, t2)
			if t3.length() > 0.001:
				t3 = t3.normalized()
				basis.append(t3)
	return basis
	
func apply_impulse(body: Rigidbody4D, impulse: Vector4, r: Vector4, inv_mass: float, I_inv: Array) -> void:
	if body.staticRigidbody:
		return
	body.velocity += impulse * inv_mass
	var torque = Bivector.wedge(r, impulse)
	var delta_omega = apply_inertia_inverse(I_inv, torque)
	body.angularVelocity = clamp_bivector(body.angularVelocity.add(delta_omega))
func apply_inertia_inverse(I_inv: Array, biv: Bivector) -> Bivector:
	if I_inv.size() != 36:  # 6x6 matrix
		return Bivector.new(0, 0, 0, 0, 0, 0)
	var biv_vec = bivector_to_vector(biv)  # [xy, xz, xw, yz, yw, zw]
	var result_vec = multiply_matrix_vector(I_inv, biv_vec)
	return bivector_from_vector(result_vec)
# Helper function to project one Vector4 onto another
func project_vector4(a: Vector4, b: Vector4) -> Vector4:
	# Avoid division by zero if b is a zero vector
	if b.length_squared() == 0:
		return Vector4(0, 0, 0, 0)
	# Compute the scalar projection and scale b
	var scalar = a.dot(b) / b.dot(b)
	return b * scalar
