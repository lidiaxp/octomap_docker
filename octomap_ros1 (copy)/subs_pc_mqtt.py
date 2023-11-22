import rclpy
from rclpy.node import Node
# import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import paho.mqtt.client as mqtt_client

broker = 'b37.mqtt.one'
port = 1883
topic_1 = "138chw3762/vlp16"
topic_2 = "138chw3762/vlp161"
client_id = f'subscriber-pointcloud-unity-to-ros'
username = '138chw3762'
password = '350bghjqvx'


class PointCloudPublisher(Node):
# class PointCloudPublisher():
    def __init__(self):
        super().__init__('publisher_point_cloud_unity')
        # rospy.init_node('publisher_point_cloud_unity')
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/cloud_unity', 10)
        # self.point_cloud_publisher = rospy.Publisher('/cloud_unity', PointCloud2, queue_size=10)
        # self.client = self.connect_mqtt()
        # self.subscribe_mqtt()

        self.pc_data = []
        self.pc_flag = 0

        client = self.connect_mqtt()
        self.subscribe(client)
        client.loop_forever()


    def publish_point_cloud(self):
        # Crie uma mensagem PointCloud2
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'point_cloud_from_unity'

        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))

        msg.height = 16
        msg.width = 360

        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        pc_data = np.asarray(self.pc_data).reshape(-1, 3)
        for point in pc_data:
            msg.data.extend(bytearray(struct.pack('fff', *point)))
        
        # msg.data = self.pc_data

        self.point_cloud_publisher.publish(msg)

        self.pc_flag = 0


    def connect_mqtt(self) -> mqtt_client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(client_id)
        client.username_pw_set(username, password)
        client.on_connect = on_connect

        client.connect(broker, port)
        return client


    def subscribe(self, client: mqtt_client):
        def on_message(client, userdata, msg):
            if msg.topic == topic_1:
                if self.pc_flag == 0:
                    data = msg.payload
                    pc_data_1 = []
                    n = 4
                    for i in range(0, len(data), n):
                        byte_pair = data[i:i+n]
                        # number = int.from_bytes(byte_pair, byteorder='big', signed=False)
                        number = struct.unpack('f', byte_pair)[0]
                        pc_data_1.append(number)

                    self.pc_data = pc_data_1
                    self.pc_flag = 1
                    print('First')

            if msg.topic == topic_2:
                if self.pc_flag == 1:
                    data = msg.payload
                    pc_data_2 = []
                    n = 4
                    for i in range(0, len(data), n):
                        byte_pair = data[i:i+n]
                        # number = int.from_bytes(byte_pair, byteorder='big', signed=False)
                        number = struct.unpack('f', byte_pair)[0]
                        pc_data_2.append(number)

                    self.pc_data += pc_data_2
                    print(max(self.pc_data))
                    self.publish_point_cloud()
                    print('Second')

        client.subscribe(topic_1)
        client.subscribe(topic_2)
        client.on_message = on_message


def run(args=None):
    rclpy.init(args=args)
    point_cloud_publisher_node = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher_node)
    rclpy.shutdown()

    # try:
    #     PointCloudPublisher()
    # except rospy.ROSInterruptException:
    #     pass


if __name__ == '__main__':
    run()

