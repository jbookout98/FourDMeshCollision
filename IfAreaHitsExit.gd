extends Node3D


func _on_area_3d_body_entered(body: Node3D) -> void:
	print(body.name)
	get_tree().quit() 
	pass # Replace with function body.


func _on_area_3d_area_entered(area: Area3D) -> void:
	print(area.name)
	get_tree().quit() 
	pass # Replace with function body.
