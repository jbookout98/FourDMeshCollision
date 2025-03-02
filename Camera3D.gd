extends Camera3D

@export var move_speed: float = 64.0
@export var sprint_multiplier: float = 2.0
@export var look_sensitivity: float = 0.2
@export var acceleration: float = 10.0
@export var deceleration: float = 5.0

var velocity := Vector3.ZERO
var target_velocity := Vector3.ZERO
var mouse_captured := true  # Track mouse state

func _ready():
	# Capture mouse for free-look mode initially
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

func _input(event):
	# Handle mouse movement for looking around
	if event is InputEventMouseMotion and mouse_captured:
		rotate_y(-deg_to_rad(event.relative.x * look_sensitivity))
		var pitch_rotation = -deg_to_rad(event.relative.y * look_sensitivity)
		var new_rotation_x = rotation_degrees.x + rad_to_deg(pitch_rotation)
		
		# Clamping vertical rotation to avoid flipping
		if new_rotation_x > -90 and new_rotation_x < 90:
			rotate_x(pitch_rotation)

	# Toggle mouse capture with ESC
	if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
		mouse_captured = !mouse_captured  # Toggle state
		if mouse_captured:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)

func _process(delta):
	handle_movement(delta)

func handle_movement(delta):
	var direction := Vector3.ZERO
	var speed := move_speed
	
	# Movement input (only when mouse is captured)
	if mouse_captured:
		if Input.is_action_pressed("move_forward"):
			direction -= transform.basis.z
		if Input.is_action_pressed("move_backward"):
			direction += transform.basis.z
		if Input.is_action_pressed("move_left"):
			direction -= transform.basis.x
		if Input.is_action_pressed("move_right"):
			direction += transform.basis.x
		if Input.is_action_pressed("move_up"):
			direction += transform.basis.y
		if Input.is_action_pressed("move_down"):
			direction -= transform.basis.y
	
		# Sprinting
		if Input.is_action_pressed("sprint"):
			speed *= sprint_multiplier

		# Normalize direction to prevent diagonal boost
		if direction != Vector3.ZERO:
			direction = direction.normalized()

		# Smooth movement using acceleration/deceleration
		target_velocity = direction * speed
		velocity = velocity.lerp(target_velocity, acceleration * delta)  # Smooth acceleration
		velocity = velocity.lerp(Vector3.ZERO, deceleration * delta)  # Smooth deceleration
		
		# Apply movement
		global_translate(velocity * delta)
