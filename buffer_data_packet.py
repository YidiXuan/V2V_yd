#from scipy.stats import expon
import numpy as np
import time
import random


class Data(object):
    def __init__(self, average_length, vtx_id):
        self.__vtx_id = vtx_id
        self.__packet_id = 0
        self.__arrival_time = 0
        self.__leave_time = 0
        self.__length = r_length(average_length)

    def get_vtx_id(self):
        return self.__vtx_id

    def set_packet_id(self, packet_id):
        self.__packet_id = packet_id

    def update_packet_id(self):
        self.__packet_id = self.__packet_id - 1

    def initial_arrival_time(self, pre_packet_arrival_time, interval_time):  # according to average arrival time to
        # produce the arrival
        #  time of packet
        self.__arrival_time = pre_packet_arrival_time + interval_time  # 随机生成服从指数分布的到达间隔

    def get_arrival_time(self):
        return self.__arrival_time

    def get_leave_time(self):
        return self.__leave_time

    def get_packet_length(self):
        return self.__length

    def set_leave_time(self, pre_packet_leave_time, arrival_time, transmission_time):
        if pre_packet_leave_time < arrival_time:
            self.__leave_time = arrival_time + transmission_time
        else:
            self.__leave_time = pre_packet_leave_time + transmission_time

    def get_packet_delay(self):
        return self.__leave_time - self.__arrival_time


def r_length(average_length):
    return int(average_length * random.expovariate(1/average_length))


def r_interval(data_rate):
    return (1/data_rate) * random.expovariate(1/data_rate)



