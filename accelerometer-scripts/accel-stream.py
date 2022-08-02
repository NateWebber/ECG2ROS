import BLE_GATT

"""
This script just connects to the Polar H10, starts receiving accelerometer data, and then prints it to the console
It's useful for testing whether the basic connection and data reception is working
"""

polar_mac_address = 'F4:D7:C4:F3:57:9A' #MAC address of Polar sensor, change this to yours


"""
This particular config is a standard configuration for accelerometer data
See the "Starting Stream" section here for more: 
https://github.com/polarofficial/polar-ble-sdk/blob/master/technical_documentation/Polar_Measurement_Data_Specification.pdf
"""
polar_stream_config = bytearray([0x02, 0x02, 0x02, 0x01, 0x08, 0x00, 0x00, 0x01, 0xC8, 0x00, 0x01, 0x01, 0x10, 0x00])

stream_start_uuid='FB005C81-02E7-F387-1CAD-8ACD2D8DF0C8'

stream_receive_uuid = 'FB005C82-02E7-F387-1CAD-8ACD2D8DF0C8'

def connect_to_stream():
    polar = BLE_GATT.Central(polar_mac_address)
    polar.connect()
    print("connected to device!")
    polar.on_value_change(stream_receive_uuid, data_conv)

    polar.char_read(stream_start_uuid)

    polar.char_write(stream_start_uuid, polar_stream_config)

    print("waiting for data!")

    polar.wait_for_notifications()

def data_conv(data):
    if data[0] == 0x02: #0x02 as first octet -> ACC data type
        """
        first octet (data[0]) is data type,
        the next 8 (data[1:8]) are a timestamp,
        and the next (data[9]) is the frame type,
        therefore we should start at data[10]
        """
        samples = data[10:] 
        step = 6 #six bytes in a "data chunk", 2 for each axis
        offset = 0
        while offset < len(samples):
            acc_data = convert_array_to_acc_data(samples, offset)
            offset += step
            print(f"acc_data: {acc_data}")

"""
This function looks at the octets that it sees at a given offset in the data
Once the samples start, data is presented in sets of 6 bytes, where the first 2 are x, middle 2 are y, and last 2 are z
Like other data from the Polar H10 it is represented in "little" byte order (i.e. most significant byte comes last)
"""
def convert_array_to_acc_data(data, offset):
    x = int.from_bytes(
        bytearray(data[offset: offset + 2]), byteorder="little", signed=True
        )
    y = int.from_bytes(
        bytearray(data[offset + 2: offset + 4]), byteorder="little", signed=True
        )
    z = int.from_bytes(
        bytearray(data[offset + 4: offset + 6]), byteorder="little", signed=True
        )

    return [-x, -y, -z] #to be totally honest with you, I'm not sure why all the data seems to be inverted, but it's otherwise accurate so I just fix it here
    
if __name__ == "__main__":
    connect_to_stream()