from threading import Thread
import time
import BLE_GATT

import matplotlib.pyplot as plt

polar_mac_address = 'F4:D7:C4:F3:57:9A'

polar_stream_config = bytearray([0x02, 0x00, 0x00, 0x01, 0x82, 0x00, 0x01, 0x01, 0x0E, 0x00])

stream_start_uuid='FB005C81-02E7-F387-1CAD-8ACD2D8DF0C8'

stream_receive_uuid = 'FB005C82-02E7-F387-1CAD-8ACD2D8DF0C8'

received_file = None

RECORD_TIME = 15 #time to record data, in seconds

"""
This additional script records data to an external file for a user-determined time period (default 15 seconds)
It then uses matplotlib to make a simple graph of the data
Most of the code here is identical to that in ECG2ROS.py
"""

def connect_to_stream():
    polar = BLE_GATT.Central(polar_mac_address)
    polar.connect()
    polar.on_value_change(stream_receive_uuid, stream_notify)

    polar.char_read(stream_start_uuid)

    polar.char_write(stream_start_uuid, polar_stream_config)

    polar.wait_for_notifications()

def stream_notify(value):
    data_conv(value)

def data_conv(data: bytearray):
    if data[0] == 0x00:
        step = 3
        samples = data[10:]
        offset = 0
        while offset < len(samples):
            ecg = convert_array_to_signed_int(samples, offset, step)
            offset += step
            received_file.write(f"{str(ecg)}\n")
            
def convert_array_to_signed_int(data, offset, length):
    return int.from_bytes(
        bytearray(data[offset : offset + length]), byteorder="little", signed=True,
    )

if __name__ == "__main__":

    open('received.txt', 'w').close() #wipe existing received.txt

    received_file = open('received.txt', 'w')
    
    #setup and run thread for collecting data from stream
    stream_thread = Thread(target=connect_to_stream)
    stream_thread.start()

    #wait until chosen time has elapsed
    for i in range(RECORD_TIME):
        print(i + 1)
        time.sleep(1)

    #close the output file
    received_file.close()   

    #reopen output file for reading
    data_file = open('received.txt', 'r')

    y_axis = data_file.read() #get all data
    y_axis = y_axis.split('\n') #convert to list

    #remove any empty data points (there's usually just one at the end)
    for x in y_axis:
        if x == '':
            y_axis.remove(x)
    
    #convert all data points into ints (they are strings before this step)
    y_axis = [int(x) for x in y_axis]

    data_file.close()

    #plot the ECG data
    plt.plot(y_axis, color='red')
    plt.ylim([-1000, 1000])
    plt.show()
