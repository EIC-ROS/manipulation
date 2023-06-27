from simple_environment_exporter.msg import ObjectData

o = ObjectData()
o.type = ObjectData.SHELF
o.mesh_source = "s"
o.shelf_height = [1,2]
print(bool(o.shelf_height))

print(o)



