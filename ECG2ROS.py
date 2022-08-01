from threading import Thread
import BLE_GATT
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

POLAR_MAC_ADDRESS = 'F4:D7:C4:F3:57:9A' #change this variable to the MAC address of your Polar H10

"""
Change this bytearray to configure the data streamed from the Polar H10
In its default state, this is configured to transmit ECG data
Default Value: bytearray([0x02, 0x00, 0x00, 0x01, 0x82, 0x00, 0x01, 0x01, 0x0E, 0x00])
See here for more: https://github.com/polarofficial/polar-ble-sdk/blob/master/technical_documentation/Polar_Measurement_Data_Specification.pdf
"""
POLAR_STREAM_CONFIG = bytearray([0x02, 0x00, 0x00, 0x01, 0x82, 0x00, 0x01, 0x01, 0x0E, 0x00])

STREAM_START_UUID='FB005C81-02E7-F387-1CAD-8ACD2D8DF0C8' #UUID for stream configuration

STREAM_RECEIVE_UUID = 'FB005C82-02E7-F387-1CAD-8ACD2D8DF0C8' #UUID for receiving stream data

PUB_NODE = None

"""
This function establishes a connection to the Polar H10, configures the stream, and then begins monitoring the data
Note: the BLE_GATT library does not have a pairing function, so you must be sure that you have already paired the Polar H10
with the computer running this script
"""
def connect_to_stream():
    #initialize BLE_GATT and connect to sensor
    polar = BLE_GATT.Central(POLAR_MAC_ADDRESS)
    polar.connect()
    #set up notifications for new data
    polar.on_value_change(STREAM_RECEIVE_UUID, stream_notify)

    #write configuration to decive so correct data is streamed
    polar.char_read(STREAM_START_UUID)
    polar.char_write(STREAM_START_UUID, POLAR_STREAM_CONFIG)

    #spin forever
    polar.wait_for_notifications()

"""
Callback for when new data is detected from the stream. Passes the data to the conversion function
"""
def stream_notify(data):
    data_conv(data)

"""
Simple ROS publisher node
Is able to accept ECG data, and publish it to the ROS topic 'biosensors/polar_h10/ECG_DATA'
This node is spun on a separate thread from the data stream
"""
class StreamPublishNode(Node):
    def __init__(self):
        super().__init__('stream_publish_node')
        self.publisher = self.create_publisher(Int32, 'biosensors/polar_h10/ECG_DATA', 100)

    def publish_data(self, data):
        self.publisher.publish(Int32(data=data))

"""
This function takes the raw ECG data from the Polar H10 (which is delivered as a bytearray) and
converts it into signed integers, which it then passes to the ROS node to be published
Taken from here: https://github.com/markspan/PolarBand2lsl/blob/main/Polar2LSL.py
"""
def data_conv(data: bytearray):
    if data[0] == 0x00:
        step = 3
        samples = data[10:]
        offset = 0
        while offset < len(samples):
            ecg = convert_array_to_signed_int(samples, offset, step)
            offset += step
            PUB_NODE.publish_data(ecg)

"""
Helper function for the data conversion process
Taken from here: https://github.com/markspan/PolarBand2lsl/blob/main/Polar2LSL.py
"""            
def convert_array_to_signed_int(data, offset, length):
    return int.from_bytes(
        bytearray(data[offset : offset + length]), byteorder="little", signed=True,
    )

if __name__ == "__main__":
    #initialize ROS and the publisher node
    rclpy.init(args=None)
    PUB_NODE = StreamPublishNode()

    #set up a thread for receiving the stream data, and a thread for spinning the ROS node
    stream_thread = Thread(target=connect_to_stream)
    spin_thread = Thread(target=rclpy.spin, args=(PUB_NODE,))

    #start both threads
    stream_thread.start()
    spin_thread.start()