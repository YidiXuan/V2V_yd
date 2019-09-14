
from buffer_data_packet import *

if __name__ == '__main__':
    file_hand = open('vtx_data.txt', mode='w')
    file_hand.write("V2V车辆数据包\n")
    file_hand.write("车辆id     数据包id      数据包到达时间       数据包离开时间        数据包长度\n")
    d2d_num = 20
    slot = 10
    slot_time = 1  # ms
    arrival_time = 0
    data_rate = 0.01  # packet/slot
    average_length = 6400  # bits/packet
    # arrival_interval = r_interval(data_rate)  # 平均到达间隔为 1/（0.01/1ms）=100 ms

    list_tx2dictp = []
    dict_tx2packet = {}  # 发射车辆的

    for i in range(1, d2d_num + 1):
        dict_tx2packet[0] = Data(average_length, i)
        temp = dict_tx2packet[0]
        file_hand.write(
            str(temp.get_vtx_id()) + "\t" + str(0) + "\t" + str(temp.get_arrival_time()) +
            "\t" + str(temp.get_leave_time()) + "\t" + str(temp.get_packet_length())+"\n")
        num_of_packet_in_buffer = 1
        pre_arrival_time = 0
        arrival_interval = r_interval(data_rate)  # 平均到达间隔为 1/（0.01/1ms）=100 ms
        while pre_arrival_time + arrival_interval < slot * slot_time:
            dict_tx2packet[num_of_packet_in_buffer] = Data(average_length, i)
            temp=dict_tx2packet[num_of_packet_in_buffer]
            dict_tx2packet[num_of_packet_in_buffer].set_packet_id(num_of_packet_in_buffer)
            dict_tx2packet[num_of_packet_in_buffer].initial_arrival_time(pre_arrival_time, arrival_interval)
            file_hand.write(str(temp.get_vtx_id())+"\t"+str(num_of_packet_in_buffer)+"\t"+str(temp.get_arrival_time())+
                            "\t"+str(temp.get_leave_time())+"\t"+str(temp.get_packet_length())+"\n")
            pre_arrival_time = pre_arrival_time + arrival_interval
            arrival_interval = r_interval(data_rate)  # 平均到达间隔为 1/（0.01/1ms）=100 ms
            num_of_packet_in_buffer = num_of_packet_in_buffer + 1
        list_tx2dictp.append(dict_tx2packet.copy())
    print("stop")

