import rospy
import socket
import struct
from std_msgs.msg import Float32
from lift.srv import Demand_position, Demand_positionRequest, Demand_positionResponse

class lift_socket:

    demand_position = 0
    max_position = 17.0
    min_position = 0.0

    def __init__(self, host:str, port:int) -> None:
        self.host = host
        self.port = port

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.host, self.port))
        print("Connected to ESP32")

        rospy.Service("/lift/set_position",Demand_position,self.request_position)

    def request_position(self,req:Demand_positionRequest):
        res = Demand_positionResponse()
        self.demand_position = req.demand_position

        if self.demand_position > self.max_position:
            res.confident_position = self.max_position
        elif self.demand_position < self.min_position:
            res.confident_position = self.min_position
        else:
            res.confident_position = self.demand_position
        
        res.send = True
        return res
            
    
    def run(self):

        while not rospy.is_shutdown():
            try:
                i = 1

                while not rospy.is_shutdown():
                    self.client_socket.send(struct.pack('!f', self.demand_position))
                    print("Sent:", self.demand_position, "Iter:",i) 
                    rospy.sleep(1)
                    
                    # use for reconnect i = 10000
                    # if i == 100 :
                    #     client_socket.close()

                    i += 1
                    
            except (socket.error, ConnectionResetError):
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.host, self.port))
                print("Reconnecting")
                continue

            except KeyboardInterrupt:
                self.client_socket.close()
                rospy.signal_shutdown("shutdown node")
                break

if __name__ == "__main__":
    rospy.init_node("test_lift")
    lift_server = lift_socket("192.168.1.101",9090)
    lift_server.run()