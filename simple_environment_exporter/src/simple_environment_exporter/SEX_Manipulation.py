# import SEX
from simple_environment_exporter._import import *
from simple_environment_exporter.Simple_Environment_eXporter import Simple_Environment_eXporter as SEX

# moveit import
from moveit_commander import PlanningSceneInterface

# msg import
from moveit_msgs.msg import PlanningScene

class SEX_Manipulation:

    SCENE = PlanningSceneInterface(synchronous=True)

    def __init__(self,sex:SEX) -> None:
        self.sex = sex

        # scene publisher
        self.scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=1)
    
    def generate_collision_object(self, id:str):
        
        def __make_Header(frame_id:str):
            header = Header()
            header.frame_id = frame_id
            header.stamp = rospy.Time.now()
            return header
        
        def __generate_mesh(obj_mesh:str,obj_pose:Pose):
            obj_posestamp = PoseStamped
            obj_posestamp.header = __make_Header(sex.Frame)
            obj_posestamp.pose = obj_pose

            self.SCENE.add_mesh(id, obj_posestamp, obj_mesh)
            self.scene_pub.publish(PlanningScene())
            return True
        
        data =  sex.load_object_data(id)
        obj_data = sex.read_object_data(data)

        if obj_data.mesh_collision_source:
            return  __generate_mesh(obj_data.mesh_collision_source,obj_data.pose)

        else:
            rospy.logerr("Cannot generate Collision object of type: "+sex.OBJECT_TYPE[obj_data.type])
            return False

if __name__ == "__main__":
    rospy.init_node("sex_mani_test")
    sex = SEX()
    sex_man = SEX_Manipulation(sex)
    print(sex_man.generate_collision_object("table"))
    