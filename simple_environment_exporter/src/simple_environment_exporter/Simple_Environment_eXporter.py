from simple_environment_exporter._import import *
from visualization_msgs.msg import Marker,MarkerArray

class Simple_Environment_eXporter:

    OBJECT_TYPE = {0:"MESH", 1:"SHELF", 3:"BOX"}

    def __init__(self, pkg:str="simple_environment_exporter", file:str="/ed/test_ed.yaml") -> None:
        self.pkg = pkg
        self.pkg_path = RosPack().get_path(pkg)
        self.ED_DATA:dict = self.__read_yaml(self.pkg_path+file)
        assert self.__read_document(), "Something worng with Document"

        self.Visual_publisher = rospy.Publisher("/SEX/Object_visual",MarkerArray,queue_size=1)
    
    def __read_yaml(self, file:str) -> Union[dict,None]:
        with open(file,'r') as ed_file:
            try:
                ed_data = yaml.safe_load(ed_file)
                return ed_data
            except yaml.YAMLError as exc:
                rospy.logerr(exc)
                return None
    
    def __read_document(self):
        self.Name:str         = self.ED_DATA["Name"]
        self.Frame:str        = self.ED_DATA["Frame"]
        self.Mesh_Folder:str  = self.ED_DATA["Mesh_Folder"]
        self.Fur_list:list    = self.__get_fur_list()
        return True
        
    
    def __get_fur_list(self):
        self.Furniture = self.ED_DATA["Furniture"]
        return list(map(itemgetter('id'), self.Furniture))
    
    def load_object_data(self, id_obj:str) -> dict:
        assert (id_obj in self.__get_fur_list()),\
                "The object id isn't declare in yaml file"
        return self.Furniture[self.Fur_list.index(id_obj)]
    
    def read_object_data(self,data:dict) -> ObjectData :
        def __get_pose(obj_pose_dict:dict) -> Pose:
            position = obj_pose_dict['position']
            orientation = obj_pose_dict['orientation']

            obj_pose = Pose()
            obj_pose.position.x = position['x']
            obj_pose.position.y = position['y']
            obj_pose.position.z = position['z']

            obj_pose.orientation.x = orientation['x']
            obj_pose.orientation.y = orientation['y']
            obj_pose.orientation.z = orientation['z']
            obj_pose.orientation.w = orientation['w']

            return obj_pose
        
        def __get_shape(obj_shape_dict:dict) -> Vector3:
            obj_scale = Vector3()
            obj_scale.x = obj_shape_dict["x"]
            obj_scale.y = obj_shape_dict["y"]
            obj_scale.z = obj_shape_dict["z"]

            return obj_scale
        
        obj_data = ObjectData()
        obj_data.id = data["id"]
        obj_data.type = eval("ObjectData.%s" % (str(data["type"]).upper()))
        if "mesh_source" in data:
            obj_data.mesh_collision_source = self.pkg_path+self.Mesh_Folder+data["mesh_source"]
            obj_data.mesh_visual_source = "package://"+self.pkg+self.Mesh_Folder+data["mesh_source"]
        
        if "shelf_height" in data:
            obj_data.shelf_height = data["shelf_height"]
        

        obj_data.shape = __get_shape(data["shape"])
        obj_data.pose = __get_pose(data["pose"])

        return obj_data
    
    def add_object_visual(self,obj_data:ObjectData,id_count:int):
        def __make_Header(frame_id:str):
            header = Header()
            header.frame_id = frame_id
            header.stamp = rospy.Time.now()
            return header
        
        def mesh_marker(obj_data:ObjectData):
            marker               = Marker()
            marker.type          = Marker.MESH_RESOURCE

            marker.scale.x       = 1
            marker.scale.y       = 1
            marker.scale.z       = 1
         
            marker.color.r       = 0
            marker.color.g       = 255/255
            marker.color.b       = 255/255
            marker.color.a       = 0.8

            marker.mesh_resource = obj_data.mesh_visual_source
            marker.mesh_use_embedded_materials = False
            return marker
        
        def box_marker(obj_data:ObjectData):
            marker               = Marker()           
            marker.type          = Marker.CUBE

            marker.scale.x       = obj_data.shape.x
            marker.scale.y       = obj_data.shape.y
            marker.scale.z       = obj_data.shape.z

            marker.color.r       = 0
            marker.color.g       = 255/255
            marker.color.b       = 255/255
            marker.color.a       = 0.8
            return marker
        

        if obj_data.mesh_visual_source:
            marker = mesh_marker(obj_data)
        else:
            marker = box_marker(obj_data)
        
        marker.id            = id_count
        marker.header        = __make_Header(self.Frame)
        marker.pose          = obj_data.pose
        marker.action        = Marker.ADD
        return marker
        
    def create_visual_all(self):
        marker_list = []
        id_count = 0
        for obj in self.Fur_list:
            o = self.read_object_data(self.load_object_data(obj))
            marker_list.append(self.add_object_visual(o,id_count))
            id_count +=1

        print(marker_list)

        marker_array = MarkerArray()
        marker_array.markers = marker_list

        return marker_array
        
if __name__ == "__main__":
    rospy.init_node("sex_tester")
    rospy.logwarn(RosPack().get_path("simple_environment_exporter"))
    sex = Simple_Environment_eXporter(file="/save/ks.yaml")
    rospy.sleep(1)
    
    print(sex.Fur_list)

    data = sex.load_object_data("sds")
    print(sex.read_object_data(data))
    rospy.logwarn("Don't forget to add marker array")

    # print(sex.add_object_visual("table",1))
    vza = sex.create_visual_all()
    print(vza)
    while not rospy.is_shutdown():
        sex.Visual_publisher.publish(vza)
        rospy.sleep(1)

    rospy.spin()