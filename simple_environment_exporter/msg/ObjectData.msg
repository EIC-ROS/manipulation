uint8 MESH=0
uint8 SHELF=1
uint8 BOX=3

string id
int32 type
geometry_msgs/Pose pose
geometry_msgs/Vector3 shape

# For object with mesh .stl
string mesh_collision_source # for making collision object as pkg_path/file
string mesh_visual_source    # for making marker as package://pkg/file

# For CUPBOARD
float32[] shelf_height