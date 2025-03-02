
extends Node3D
class_name Rigidbody4D

@onready var transform4d = $Transform4D
@export var mass =1.0
var velocity:Vector4 = Vector4(0,0,0,0)
@onready var angularVelocity :Bivector =Bivector.new(0,0,0,0,0,0)

@export var debug = false

func project_vertices(vertices: Array, axis: Vector4) -> Dictionary:
	var min_proj = INF
	var max_proj = -INF
	for v in vertices:
		var proj = v.dot(axis)
		if proj < min_proj:
			min_proj = proj
		if proj > max_proj:
			max_proj = proj
	return {"min": min_proj, "max": max_proj}


func unique_axes(axes: Array) -> Array:
	var unique = []
	for a in axes:
		var found = false
		for u in unique:
			if abs(a.dot(u)) > 0.99:
				found = true
				break
		if not found:
			unique.append(a)
	return unique


func get_candidate_axes(vertsA: Array, vertsB: Array) -> Array:
	var axes = []

	axes.append(Vector4(1, 0, 0, 0))
	axes.append(Vector4(0, 1, 0, 0))
	axes.append(Vector4(0, 0, 1, 0))
	axes.append(Vector4(0, 0, 0, 1))

	for i in range(vertsA.size()):
		for j in range(i + 1, vertsA.size()):
			var diff = vertsA[j] - vertsA[i]
			if diff.length() > 0.001:
				axes.append(diff.normalized())

	for i in range(vertsB.size()):
		for j in range(i + 1, vertsB.size()):
			var diff = vertsB[j] - vertsB[i]
			if diff.length() > 0.001:
				axes.append(diff.normalized())
	
	return unique_axes(axes)


# Main collision check function.
# Retrieves the transformed 4D vertices from our mesh (via Transform4d)
# and from another Rigidbody4D, then checks all candidate axes.
# If a separating axis is found, the objects are not colliding.
func check_collision_with_mtv(other: Rigidbody4D) -> Dictionary:

	var vertsA = $CollisionShape4D.get_transformed_vertices(transform4d.rotor)
	var vertsB = other.get_node("CollisionShape4D").get_transformed_vertices(other.transform4d.rotor)
	
	var axes = get_candidate_axes(vertsA, vertsB)

	var min_overlap = INF
	var collision_normal = Vector4(0, 0, 0, 0)

	for axis in axes:
		var projA = project_vertices(vertsA, axis)
		var projB = project_vertices(vertsB, axis)
	
		if projA["max"] < projB["min"] or projB["max"] < projA["min"]:
			return {"colliding": false}

		var overlap = min(projA["max"], projB["max"]) - max(projA["min"], projB["min"])

		if overlap < min_overlap:
			min_overlap = overlap
			collision_normal = axis.normalized()

	var centroidA = get_centroid(vertsA)
	var centroidB = get_centroid(vertsB)
	var center_diff = (centroidB - centroidA).normalized()
	# If the collision normal is pointing opposite the vector from A to B, flip it.
	if collision_normal.dot(center_diff) < 0:
		collision_normal = -collision_normal
	
	return {"colliding": true, "normal": collision_normal, "penetration": min_overlap}

func get_centroid(vertices: Array) -> Vector4:
	var centroid = Vector4(0, 0, 0, 0)
	for v in vertices:
		centroid += v
	return centroid / vertices.size()

func get_position4d() -> Vector4:
	var pos3 = global_position
	var trans4d = get_node("Transform4D").translation.w
	return Vector4(pos3.x, pos3.y, pos3.z, trans4d)

func set_position4d(new_pos: Vector4) -> void:
	global_position = Vector3(new_pos.x, new_pos.y, new_pos.z)
	get_node("Transform4D").translation.w = new_pos.w

func get_contact_offset(contact_point: Vector4) -> Vector4:

	return contact_point - Vector4(global_position.x,global_position.y,global_position.z,$Transform4D.translation.w)

func get_point_velocity(contact_point: Vector4) -> Vector4:

	var offset = get_contact_offset(contact_point) 

	return left_contraction(offset, angularVelocity)
	

func left_contraction(v: Vector4, biv: Bivector) -> Vector4:
	var res_x = -(biv.xy * v.y + biv.xz * v.z + biv.xw * v.w)
	var res_y = biv.xy * v.x - biv.yz * v.z - biv.yw * v.w
	var res_z = biv.xz * v.x + biv.yz * v.y - biv.zw * v.w
	var res_w = biv.xw * v.x + biv.yw * v.y + biv.zw * v.z
	return Vector4(res_x, res_y, res_z, res_w)

func randomizeVelocity():
	if global_position.x>0:
		velocity.x = -48
		velocity.w=16
	else:
		velocity.x = 64

func computeAngularVelocityFromRotor(current_rotor: Rotor, prev_rotor: Rotor, delta: float) -> Bivector:
	var delta_rotor = current_rotor.multiply(prev_rotor.inverse())

	var rotation_bivector = delta_rotor.logarithm()
	
	var angular_velocity = Bivector.new(
		rotation_bivector["xy"] / delta,
		rotation_bivector["xz"] / delta,
		rotation_bivector["xw"] / delta,
		rotation_bivector["yz"] / delta,
		rotation_bivector["yw"] / delta,
		rotation_bivector["zw"] / delta
	)
	return angular_velocity
func rotor_exp(biv: Bivector) -> Rotor:
	var eps = 1e-6
	var omega_mag = biv.magnitude()
	if omega_mag < eps:
		return Rotor.new(1.0, 0, 0, 0, 0, 0, 0)
	var cos_val = cos(omega_mag)
	var sin_val = sin(omega_mag)
	var u_xy = biv.xy / omega_mag
	var u_xz = biv.xz / omega_mag
	var u_xw = biv.xw / omega_mag
	var u_yz = biv.yz / omega_mag
	var u_yw = biv.yw / omega_mag
	var u_zw = biv.zw / omega_mag
	return Rotor.new(cos_val,
					   sin_val * u_xy,
					   sin_val * u_xz,
					   sin_val * u_xw,
					   sin_val * u_yz,
					   sin_val * u_yw,
					   sin_val * u_zw)

const ANGULAR_DAMPING = 0.8  # Lower this value for more damping
const MAX_ANG_VEL = 100.0 

func clamp_angular_velocity(ang: Bivector, max_val: float) -> Bivector:
	var comps = [ang.xy, ang.xz, ang.xw, ang.yz, ang.yw, ang.zw]
	var mag = sqrt(comps[0]*comps[0] + comps[1]*comps[1] + comps[2]*comps[2] +
				   comps[3]*comps[3] + comps[4]*comps[4] + comps[5]*comps[5])
	if mag > max_val and mag > 0:
		var scale_factor = max_val / mag
		return ang.scale(scale_factor)
	return ang

func update_state(delta: float) -> void:
	angularVelocity = angularVelocity.scale(ANGULAR_DAMPING)
	if angularVelocity.magnitude() < 1e-6:
		angularVelocity = Bivector.new(0, 0, 0, 0, 0, 0)
	
	angularVelocity = clamp_angular_velocity(angularVelocity, MAX_ANG_VEL)
	
	var delta_rotor: Rotor = rotor_exp(angularVelocity.scale(delta))
	
	transform4d.rotor = delta_rotor.multiply(transform4d.rotor,true)
	transform4d.rotor.update_exported_angles()

	global_position += Vector3(velocity.x, velocity.y, velocity.z) * delta
	$Transform4D.translation.w += velocity.w * delta
	if debug:
		print("Rotor Data: " + transform4d.rotor.getListOfAngles())

