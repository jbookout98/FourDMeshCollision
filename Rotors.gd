@tool
extends Resource
class_name Rotor


#@export_range(0.0, 360.0, 0.1) var angle_xy: float = 0.0
#@export_range(0.0, 360.0, 0.1) var angle_xz: float = 0.0
#@export_range(0.0, 360.0, 0.1) var angle_xw: float = 0.0
#@export_range(0.0, 360.0, 0.1) var angle_yz: float = 0.0
#@export_range(0.0, 360.0, 0.1) var angle_yw: float = 0.0
#@export_range(0.0, 360.0, 0.1) var angle_zw: float = 0.0

var is_dirty: bool = true  

# Internal rotor components (computed from the angles)
var s: float = 1.0
var xy: float = 0.0
var xz: float = 0.0
var xw: float = 0.0
var yz: float = 0.0
var yw: float = 0.0
var zw: float = 0.0
var components = []
# Compute the rotor from the exported angles.
func update_rotor(angular_velocity: Dictionary, delta: float) -> void:
	# Angular velocity as a bivector, e.g., {"xy": 10.0, "xz": 0.0, ...} in degrees/sec
	var delta_rotor = Rotor.new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
	for plane in ["xy", "xz", "xw", "yz", "yw", "zw"]:
		var angle = angular_velocity.get(plane, 0.0) * delta
		if angle != 0.0:
			delta_rotor = delta_rotor.multiply(Rotor.from_angle_plane_degrees(angle, plane))
	delta_rotor.normalize()
	# Apply the delta rotation
	var new_rotor = delta_rotor.multiply(self)
	s = new_rotor.s
	xy = new_rotor.xy
	xz = new_rotor.xz
	xw = new_rotor.xw
	yz = new_rotor.yz
	yw = new_rotor.yw
	zw = new_rotor.zw
	normalize()

# Constructor remains the same if needed.
func _init(s: float = 1.0, xy: float = 0.0, xz: float = 0.0, xw: float = 0.0, yz: float = 0.0, yw: float = 0.0, zw: float = 0.0):
	self.s = s
	self.xy = xy
	self.xz = xz
	self.xw = xw
	self.yz = yz
	self.yw = yw
	self.zw = zw

# Normalize the rotor to ensure it represents a valid rotation.
func normalize() -> void:
	var norm = sqrt(s * s + xy * xy + xz * xz + xw * xw + yz * yz + yw * yw + zw * zw)
	if norm != 0:
		s /= norm
		xy /= norm
		xz /= norm
		xw /= norm
		yz /= norm
		yw /= norm
		zw /= norm


func round_to_decimals(value: float, decimals: int) -> float:
	var factor = pow(10, decimals)
	return round(value * factor) / factor

# Rotor multiplication (combining rotations).
func multiply(other: Rotor, debug:bool=false) -> Rotor:

	other.normalize()

	var s_new = s * other.s - xy * other.xy - xz * other.xz - xw * other.xw - yz * other.yz - yw * other.yw - zw * other.zw

	var xy_new = s * other.xy + xy * other.s + xz * other.yz - yz * other.xz + xw * other.yw - yw * other.xw
	var xz_new = s * other.xz - xy * other.yz + xz * other.s + yz * other.xy + xw * other.zw - zw * other.xw
	var xw_new = s * other.xw - xy * other.yw + xz * other.zw - zw * other.xz + xw * other.s + yw * other.xy
	var yz_new = s * other.yz + xy * other.xz - xz * other.xy + yz * other.s + yw * other.zw - zw * other.yw
	var yw_new = s * other.yw + xy * other.xw - xw * other.xy + xz * other.zw - zw * other.xz + yw * other.s
	var zw_new = s * other.zw + xy * other.xw - xw * other.xy + yz * other.yw - yw * other.yz + zw * other.s

	var result = Rotor.new(s_new, xy_new, xz_new, xw_new, yz_new, yw_new, zw_new)
	result.normalize()  # Final normalization step.

	return result

func getListOfAngles():
	var list = "radxy: " + str(xy)+" radxz: " + str(xz)+" radxw: " + str(xw)+" radyz: " + str(yz)+" radyw: " + str(yw)+" radzw: " + str(zw)
	return list
# Apply the rotor to a 4D vector.
func rotate(vector: Vector4) -> Vector4:
	
	var rotated = rotate_vector(vector)
	
	
	return Vector4(rotated.x, rotated.y, rotated.z, rotated.w)  # Now correct!


# Utility function to create a rotor from an angle (in degrees) and a plane.
static func from_angle_plane_degrees(angle_degrees: float, plane: String) -> Rotor:
	if angle_degrees == null:
		angle_degrees = 0.0

	var half_angle = deg_to_rad(float(angle_degrees)) / 2.0
	var cos_half = cos(half_angle)
	var sin_half = sin(half_angle)

	# Ensure all components are initialized to 0
	var rotor = Rotor.new(cos_half, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

	match plane:
		"xy": rotor.xy = sin_half
		"xz": rotor.xz = sin_half
		"xw": rotor.xw = sin_half
		"yz": rotor.yz = sin_half
		"yw": rotor.yw = sin_half
		"zw": rotor.zw = sin_half
		_:
			push_error("Invalid rotation plane: " + plane)

	return rotor

# Returns a duplicate of the normalized components without modifying the original.
func get_normalized_components() -> Array:
	var comps = components.duplicate(true)
	var norm = 0.0
	for value in comps:
		norm += value * value
	norm = sqrt(norm)
	if norm != 0:
		for i in range(comps.size()):
			comps[i] /= norm
	return comps

# Custom comparison function for Rotor.
func equals(other: Rotor, tol: float = 1e-6) -> bool:
	
	if self.angle_xy != other.angle_xy:
		return false
	if self.angle_xz != other.angle_xz:
		return false
	if self.angle_xw != other.angle_xw:
		return false
	if self.angle_yz != other.angle_yz:
		return false
	if self.angle_yw != other.angle_yw:
		return false
	if self.angle_zw != other.angle_zw:
		return false
	
		
	return true
func betterEquals(other: Rotor, tol: float = .00001) -> bool:
	
	if abs(self.xy - other.xy)>tol:
		return false
	if abs(self.xz - other.xz)>tol:
		return false
	if abs(self.xw - other.xw)>tol:
		return false
	if abs(self.yz - other.yz)>tol:
		return false
	if abs(self.yw - other.yw)>tol:
		return false
	if abs(self.zw - other.zw)>tol:
		return false
	return true
# Add this function to your Rotor class.
func rotation_matrix_xy(angle: float) -> Array:
	var cos_theta = cos(angle)
	var sin_theta = sin(angle)
	return [
		[ cos_theta, -sin_theta, 0, 0],
		[ sin_theta,  cos_theta, 0, 0],
		[        0,         0, 1, 0],
		[        0,         0, 0, 1]
	]

func rotation_matrix_xz(angle: float) -> Array:
	var cos_theta = cos(angle)
	var sin_theta = sin(angle)
	return [
		[ cos_theta, 0, -sin_theta, 0],
		[        0, 1,         0, 0],
		[ sin_theta, 0,  cos_theta, 0],
		[        0, 0,         0, 1]
	]
func rotation_matrix_xw(angle: float) -> Array:
	var cos_theta = cos(angle)
	var sin_theta = sin(angle)
	return [[ cos_theta,       0,       0, sin_theta],
			 [      0,       1,       0,       0],
			 [      0,       0,       1,       0],
			 [ sin_theta,       0,       0,  cos_theta]]

func rotation_matrix_yz(angle: float) -> Array:
	var cos_theta = cos(angle)
	var sin_theta = sin(angle)
	return [[      1,       0,       0,       0],
	 [      0,  cos_theta, -sin_theta,       0],
	 [      0,  sin_theta,  cos_theta,       0],
	 [      0,       0,       0,       1]]


func rotation_matrix_yw(angle: float) -> Array:
	var cos_theta = cos(angle)
	var sin_theta = sin(angle)
	return [
		[ 1,         0, 0,         0],
		[ 0,  cos_theta, 0, -sin_theta],
		[ 0,         0, 1,         0],
		[ 0,  sin_theta, 0,  cos_theta]
	]


func rotation_matrix_zw(angle: float) -> Array:
	var cos_theta = cos(angle)
	var sin_theta = sin(angle)
	return [
		[ 1, 0,         0,         0],
		[ 0, 1,         0,         0],
		[ 0, 0,  cos_theta, -sin_theta],
		[ 0, 0,  sin_theta,  cos_theta]
	]
	


# Returns a 4x4 rotation matrix as an Array of 4 Arrays (each a row of 4 floats).
func get_combined_rotation_matrix() -> Array:
	var R = [
		[1, 0, 0, 0],
		[0, 1, 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]
	]  # Identity matrix
	self.normalize()
	R = multiply_matrices(R, rotation_matrix_xy(xy))
	R = multiply_matrices(R, rotation_matrix_xz(xz))
	R = multiply_matrices(R, rotation_matrix_xw(xw))
	R = multiply_matrices(R, rotation_matrix_yz(yz))
	R = multiply_matrices(R, rotation_matrix_yw(yw))
	R = multiply_matrices(R, rotation_matrix_zw(zw))

	return R
func rotate_vector(v: Vector4) -> Vector4:
	var R = get_combined_rotation_matrix()
	var x = v.x * R[0][0] + v.y * R[0][1] + v.z * R[0][2] + v.w * R[0][3]
	var y = v.x * R[1][0] + v.y * R[1][1] + v.z * R[1][2] + v.w * R[1][3]
	var z = v.x * R[2][0] + v.y * R[2][1] + v.z * R[2][2] + v.w * R[2][3]
	var w = v.x * R[3][0] + v.y * R[3][1] + v.z * R[3][2] + v.w * R[3][3]
	return Vector4(x, y, z, w)
func inverse() -> Rotor:
	
	return Rotor.new(s, -xy, -xz, -xw, -yz, -yw, -zw)



func multiply_matrices(A: Array, B: Array) -> Array:
	var result = []
	for i in range(4):
		var row = []
		for j in range(4):
			var sum = 0.0
			for k in range(4):
				sum += A[i][k] * B[k][j]
			row.append(sum)
		result.append(row)
	return result
# Computes the logarithm of a simple rotor.
# For a rotor R = cos(theta) + sin(theta)*B_hat (with B_hat a unit bivector),
# the logarithm is log(R) = theta * B_hat.
# This implementation assumes the rotor is a simple rotor (rotation in one plane).
# Inside your Rotor class:
func logarithm() -> Dictionary:
	var eps = 1e-6
	var biv_mag_sq = self.xy * self.xy + self.xz * self.xz + self.xw * self.xw + self.yz * self.yz + self.yw * self.yw + self.zw * self.zw
	var biv_mag = sqrt(biv_mag_sq)
	
	var s_clamped = clamp(self.s, -1.0, 1.0)
	var theta = acos(s_clamped)
	# Adjust for the discontinuity if necessary.
	# For instance, if s_clamped is nearly -1, force theta to π instead of jumping.
	if abs(s_clamped + 1.0) < eps:
		theta = PI
	
	if biv_mag < eps:
		return {"xy": 0.0, "xz": 0.0, "xw": 0.0, "yz": 0.0, "yw": 0.0, "zw": 0.0}
	
	var unit_xy = self.xy / biv_mag
	var unit_xz = self.xz / biv_mag
	var unit_xw = self.xw / biv_mag
	var unit_yz = self.yz / biv_mag
	var unit_yw = self.yw / biv_mag
	var unit_zw = self.zw / biv_mag
	
	return {
		"xy": theta * unit_xy,
		"xz": theta * unit_xz,
		"xw": theta * unit_xw,
		"yz": theta * unit_yz,
		"yw": theta * unit_yw,
		"zw": theta * unit_zw
	}
