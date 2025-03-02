extends Object
class_name Bivector

var xy: float = 0.0
var xz: float = 0.0
var xw: float = 0.0
var yz: float = 0.0
var yw: float = 0.0
var zw: float = 0.0

func _init(_xy: float = 0.0, _xz: float = 0.0, _xw: float = 0.0, _yz: float = 0.0, _yw: float = 0.0, _zw: float = 0.0) -> void:
	xy = _xy
	xz = _xz
	xw = _xw
	yz = _yz
	yw = _yw
	zw = _zw


# Static method: converts a rotor to a bivector by computing its logarithm.
static func from_rotor(r: Rotor) -> Bivector:
	var eps = 1e-6
	# Compute the squared magnitude of the bivector part of the rotor.
	var biv_mag_sq = r.xy * r.xy + r.xz * r.xz + r.xw * r.xw + r.yz * r.yz + r.yw * r.yw + r.zw * r.zw
	var biv_mag = sqrt(biv_mag_sq)
	
	# Clamp the scalar part to ensure it is in [-1,1]
	var s_clamped = clamp(r.s, -1.0, 1.0)
	var theta = acos(s_clamped)
	
	# If the bivector part is nearly zero, return a zero bivector.
	if biv_mag < eps:
		return Bivector.new()
	
	# Normalize the bivector components to get the unit bivector.
	var unit_xy = r.xy / biv_mag
	var unit_xz = r.xz / biv_mag
	var unit_xw = r.xw / biv_mag
	var unit_yz = r.yz / biv_mag
	var unit_yw = r.yw / biv_mag
	var unit_zw = r.zw / biv_mag
	
	# Multiply the unit bivector by theta to get the logarithm of the rotor.
	return Bivector.new(
		theta * unit_xy,
		theta * unit_xz,
		theta * unit_xw,
		theta * unit_yz,
		theta * unit_yw,
		theta * unit_zw
	)
static func wedge(v: Vector4, w: Vector4) -> Bivector:
	var biv_xy = v.x * w.y - v.y * w.x
	var biv_xz = v.x * w.z - v.z * w.x
	var biv_xw = v.x * w.w - v.w * w.x
	var biv_yz = v.y * w.z - v.z * w.y
	var biv_yw = v.y * w.w - v.w * w.y
	var biv_zw = v.z * w.w - v.w * w.z
	return Bivector.new(biv_xy, biv_xz, biv_xw, biv_yz, biv_yw, biv_zw)
func add(other: Bivector) -> Bivector:
	return Bivector.new(xy + other.xy, xz + other.xz, xw + other.xw, yz + other.yz, yw + other.yw, zw + other.zw)

func subtract(other: Bivector) -> Bivector:
	return Bivector.new(xy - other.xy, xz - other.xz, xw - other.xw, yz - other.yz, yw - other.yw, zw - other.zw)
func getComponents():
	var tArray = [self.xy,self.xz,self.xw,self.yz,self.yw,self.zw]
	return tArray
func scale(scalar: float) -> Bivector:
	return Bivector.new(
		xy * scalar,
		xz * scalar,
		xw * scalar,
		yz * scalar,
		yw * scalar,
		zw * scalar
	)
func magnitude() -> float:
	return sqrt(xy * xy + xz * xz + xw * xw + yz * yz + yw * yw + zw * zw)
