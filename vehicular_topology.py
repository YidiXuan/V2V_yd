from vehicular_resourance_allocation import *
from coalitional_games import *
# import random
import math
# import numpy as np
import time
from dual_graph_colouring import *
from Resource_management_by_YeLi import *
from Resource_management_icc import *
from Fair_new import *


# 单小区拓扑类
class SingleCell(object):
    def __init__(self, cue_num, d2d_num, rb_num, up_or_down_link, d_tx2rx, highway):
        # self.__radius = radius  # 小区半径
        self.__cue_num = cue_num
        self.__d2d_num = d2d_num
        self.__rb_num = rb_num
        self.__up_or_down_link = up_or_down_link
        self.__d_tx2rx = d_tx2rx  # D2D发射机与接收机之间的最大距离
        self.__highway = highway

        self.__dict_id2device = {}  # id-设备对象登记表
        self.__dict_id2rx = {}  # id-接收机对象登记表
        self.__dict_id2tx = {}  # id-发射机对象登记表
        self.__dict_id2channel = {}  # 接收机id-信道对象登记表
        self.__dict_id2channel_mmwave = {}  # 接收机id-mmwave信道对象登记表
        # self.__observations = {}
        # self.__observations_ = {}
        self.__dict_tx_id2sinr = {}
        self.__list_rate = []
        self.__list_slot = []

        self.__list_cue_sinr_random = []
        self.__list_cue_sinr_fair = []
        # self.__list_cue_sinr_rl = []
        self.__list_d2d_sinr_random = []
        self.__list_d2d_sinr_fair = []
        self.__list_d2d_rate_random = []
        self.__list_d2d_rate_fair = []
        # self.__list_d2d_sinr_rl = []
        self.__list_cue_sinr_reinforcement = []
        self.__list_d2d_sinr_reinforcement = []
        self.__list_d2d_rate_reinforcement = []

        self.__list_total_reward = []
        self.__list_single_reward = []

        self.__success_transmission = 0
        self.__failed_transmission = 0

    def get_success(self):
        return self.__success_transmission

    def get_fail(self):
        return self.__failed_transmission

    def initial(self):
        # 生成蜂窝用户对象 V2I
        for i_id in range(1, 1+self.__cue_num):
            cue = CUE(i_id, 'CUE')
            # x, y = self.random_position()
            cue.initial_user_location(self.__highway)
            # cue.set_location(x, y)
            self.__dict_id2device[i_id] = cue
            if self.__up_or_down_link == 'down':
                self.__dict_id2rx[i_id] = cue
            else:
                self.__dict_id2tx[i_id] = cue

        # 生成基站对象
        bs = BS(0, 'BS')  # 一个基站 id = 0
        self.__dict_id2device[0] = bs
        if self.__up_or_down_link == 'down':
            self.__dict_id2tx[0] = bs
        else:
            for tx_id in self.__dict_id2tx:
                bs.set_tx(tx_id)
            self.__dict_id2rx[0] = bs

        # 生成D2D对象
        for i_id in range(1+self.__cue_num, 1+self.__cue_num+self.__d2d_num):
            # D2D发射机对象
            d2d_tx = D2DTx(i_id, 'D2DTx')
            # tx_x, tx_y = self.random_position()
            # d2d_tx.set_location(tx_x, tx_y)
            d2d_tx.initial_user_location(self.__highway)
            d2d_tx.make_pair(i_id+self.__d2d_num)

            # 第一个D2D发射机用于训练
            # if i_id == self.__cue_num+self.__d2d_num:
            # d2d_tx.train = True
            # D2D接收机对象
            d2d_rx = D2DRx(i_id+self.__d2d_num, 'D2DRx')
            # rx_x, rx_y = self.d2d_rx_position(self.__d_tx2rx, tx_x, tx_y)
            # d2d_rx.set_location(rx_x, rx_y)
            d2d_rx.initial_user_location(self.__highway, d2d_tx)
            d2d_rx.make_pair(i_id)

            self.__dict_id2device[i_id] = d2d_tx
            self.__dict_id2tx[i_id] = d2d_tx
            self.__dict_id2device[i_id+self.__d2d_num] = d2d_rx
            self.__dict_id2rx[i_id+self.__d2d_num] = d2d_rx

        # D2D用户之间的车辆
        '''for tx_id in range(1+self.__cue_num, 1+self.__cue_num+self.__d2d_num):
            n = 0
            tx_x_point = self.__dict_id2tx[tx_id].get_x_point()
            tx_y_point = self.__dict_id2tx[tx_id].get_y_point()
            rx_x_point = self.__dict_id2rx[self.__dict_id2tx[tx_id].get_rx_id()].get_x_point()
            rx_y_point = self.__dict_id2rx[self.__dict_id2tx[tx_id].get_rx_id()].get_y_point()
            for i_id in self.__dict_id2device:
                if i_id != tx_id:
                    x = self.__dict_id2device[i_id].get_x_point()
                    y = self.__dict_id2device[i_id].get_y_point()
                    if ((y - tx_y_point)/(x - tx_x_point) <= (rx_y_point - tx_y_point)/(rx_x_point - tx_x_point) *
                        (math.sin(math.asin(1 / 5)) + 1)) & \
                            ((y - tx_y_point)/(x - tx_x_point) >= (rx_y_point - tx_y_point)/(rx_x_point - tx_x_point) *
                             (-math.sin(math.asin(1 / 5)) + 1)):
                        n = n + 1
            self.__dict_id2tx[tx_id].set_blockers(n)
            print(self.__dict_id2tx[tx_id].get_blockers())
            print(tx_id)'''

        # 生成信道 一个接收机对应一个信道对象
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            temp_channel = Channel(rx_id)
            print(rx_id)
            for tx_id in self.__dict_id2tx:  # 遍历所有的发射机
                print(tx_id)

                temp_channel.update_link_loss_cell(self.__dict_id2tx[tx_id], self.__dict_id2rx[rx_id])
                if tx_id in range(1+self.__cue_num, 1+self.__cue_num+self.__d2d_num):
                    if rx_id != 0:

                        temp_channel.update_link_loss_mmwave(self.__dict_id2tx[tx_id], self.__dict_id2rx[rx_id], self)

                        self.__dict_id2channel_mmwave[temp_channel.get_rx_id()] = temp_channel
                self.__dict_id2channel[temp_channel.get_rx_id()] = temp_channel

        random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)

    def accumulate_blockers(self, tx_id, tx_x_point, tx_y_point, rx_x_point, rx_y_point):
        n = 0
        for i_id in self.__dict_id2device:
            if i_id != tx_id:
                x = self.__dict_id2device[i_id].get_x_point()
                y = self.__dict_id2device[i_id].get_y_point()
                # 计算tx_rx向量与tx_car的夹角
                vector1 = np.array([(rx_x_point - tx_x_point), (rx_y_point - tx_y_point)])
                vector2 = np.array([(x - tx_x_point), (y - tx_y_point)])
                cosvalue_tx = float(vector1.dot(vector2)) / (np.sqrt(vector2.dot(vector2)) * np.sqrt(vector1.dot(vector1)))
                cosvalue_tx = round(cosvalue_tx, 4)
                angel_tx = np.arccos(cosvalue_tx)
                # 计算rx_tx向量与rx_car的夹角
                vector3 = np.array([(tx_x_point - rx_x_point), (tx_y_point - rx_y_point)])
                vector4 = np.array([(x - rx_x_point), (y - rx_y_point)])
                cosvalue_rx = float(vector3.dot(vector4)) / (np.sqrt(vector3.dot(vector3)) * np.sqrt(vector4.dot(vector4)))
                cosvalue_rx = round(cosvalue_rx, 4)
                angel_rx = np.arccos(cosvalue_rx)
                if angel_rx <= math.pi / 12 and angel_tx <= math.pi / 12:
                    n = n + 1
                self.__dict_id2tx[tx_id].set_blockers(n)
        return n

    def random_spectrum_allocation_work(self):
        for rx_id in self.__dict_id2rx:
            if rx_id != 0:
                rx_channel_mmwave = self.__dict_id2channel_mmwave[rx_id]
                for tx_id in self.__dict_id2tx:
                    if tx_id not in range(1, 1 + self.__cue_num):
                        channel_link_loss_mm = rx_channel_mmwave.get_link_loss_mmwave(tx_id)
                        channel_link_loss_mm = pow(10, -channel_link_loss_mm / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                        channel_link_loss_mm = -10 * math.log10(channel_link_loss_mm)
                        rx_channel_mmwave.set_link_loss_mmwave(channel_link_loss_mm, tx_id)
        # time_start = time.time()
        random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # time_end = time.time()
        # time_use = (time_end - time_start) / 3600
        # for i_id in range(1, 1 + self.__cue_num + self.__d2d_num):
            # self.__dict_id2device[i_id].update_location_after_spectrum_allocation(time_use)

        #print('40车辆位置：' + str(self.__dict_id2device[40].get_x_point())+' '+str(self.__dict_id2device[40].get_y_point()))

        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num-1 and rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                inter_power = self.__dict_id2rx[rx_id].comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)
                if self.__dict_id2rx[rx_id].get_sinr() > 5:
                    rate = 2.16 * pow(10, 9) * math.log2(1 + self.__dict_id2rx[rx_id].get_sinr())
                else:
                    rate = 0
            else:   # 计算蜂窝用户
                inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                if rx_id != 0:
                    if self.__dict_id2rx[rx_id].get_sinr() > 5:
                        rate = 1.8 * pow(10, 5) * math.log2(1 + self.__dict_id2rx[rx_id].get_sinr())
                    else:
                        rate = 0
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                rb_id_use = self.__dict_id2rx[rx_id].get_allocated_rb()[0]
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                self.__dict_tx_id2sinr[tx_id] = sinr
                print('D2D接收机ID:' + str(rx_id) + ' '+str(rb_id_use) + ' SINR:' + str(sinr))
                self.__list_d2d_sinr_random.append(sinr)
                self.__list_d2d_rate_random.append(rate)
            else:  # CUE
                for tx_id in sinr:
                    rb_id_use = self.__dict_id2tx[tx_id].get_allocated_rb()[0]
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    print('基站对应的发射机ID:' + str(tx_id) + ' '+str(rb_id_use) +' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_random.append(sinr[tx_id])


    def graph_spectrum_allocation_work(self):

        # 快衰信道
        for rx_id in self.__dict_id2rx:
            rx = self.__dict_id2rx[rx_id]
            rx_channel = self.__dict_id2channel[rx_id]
            if rx_id != 0:
                rx_channel_mmwave = self.__dict_id2channel_mmwave[rx_id]
                for tx_id in self.__dict_id2tx:
                    if tx_id not in range(1, 1 + self.__cue_num):
                        channel_link_loss_mm = rx_channel_mmwave.get_link_loss_mmwave(tx_id)
                        channel_link_loss_mm = pow(10, -channel_link_loss_mm / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                        channel_link_loss_mm = -10 * math.log10(channel_link_loss_mm)
                        rx_channel_mmwave.set_link_loss_mmwave(channel_link_loss_mm, tx_id)
            for tx_id in self.__dict_id2tx:
                if tx_id not in range(1, 1 + self.__cue_num) and rx_id != 0:
                    channel_link_loss = rx_channel.get_link_loss(tx_id)
                    channel_link_loss = pow(10, -channel_link_loss / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                    channel_link_loss = -10 * math.log10(channel_link_loss)
                    rx_channel.set_link_loss_cell(channel_link_loss, tx_id)
        # time_start = time.time()
        # random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # graph_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # time_end = time.time()
        # time_use = (time_end - time_start) / 3600
        # for i_id in range(1, 1 + self.__cue_num + self.__d2d_num):
            # self.__dict_id2device[i_id].update_location_after_spectrum_allocation(time_use)
        min_sir_cue = pow(10, 5 / 10)
        min_sir_v2v = 0
        # print('40车辆位置：' + str(self.__dict_id2device[40].get_x_point())+' '+str(self.__dict_id2device[40].get_y_point()))

        graph_colouring(self.__dict_id2tx, self.__dict_id2rx, self.__cue_num, self.__d2d_num, self.__rb_num, self.__dict_id2channel,
                        self.__dict_id2channel_mmwave, min_sir_cue, min_sir_v2v)

        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num-1 and rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                inter_power = self.__dict_id2rx[rx_id].comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)
                if self.__dict_id2rx[rx_id].get_sinr() > 0:
                    rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                else:
                    rate = 0
                rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            else:   # 计算蜂窝用户
                inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                if rx_id != 0:
                    if self.__dict_id2rx[rx_id].get_sinr() > 0:
                        rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                    else:
                        rate = 0
                    rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                rb_id_use = self.__dict_id2rx[rx_id].get_allocated_rb()[0]
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                self.__dict_tx_id2sinr[tx_id] = sinr
                print('D2D接收机ID:' + str(rx_id) + ' '+str(rb_id_use) + ' SINR:' + str(sinr))
                self.__list_d2d_sinr_random.append(sinr)
                self.__list_d2d_rate_random.append(rate)
            else:  # CUE
                for tx_id in sinr:
                    rb_id_use = self.__dict_id2tx[tx_id].get_allocated_rb()[0]
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    print('基站对应的发射机ID:' + str(tx_id) + ' '+str(rb_id_use) +' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_random.append(sinr[tx_id])

    def graph_spectrum_allocation_icc_work(self):
        for rx_id in self.__dict_id2rx:
            rx = self.__dict_id2rx[rx_id]
            rx_channel = self.__dict_id2channel[rx_id]
            if rx_id != 0:
                rx_channel_mmwave = self.__dict_id2channel_mmwave[rx_id]
                for tx_id in self.__dict_id2tx:
                    if tx_id not in range(1, 1 + self.__cue_num):
                        channel_link_loss_mm = rx_channel_mmwave.get_link_loss_mmwave(tx_id)
                        channel_link_loss_mm = pow(10, -channel_link_loss_mm / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                        channel_link_loss_mm = -10 * math.log10(channel_link_loss_mm)
                        rx_channel_mmwave.set_link_loss_mmwave(channel_link_loss_mm, tx_id)
            for tx_id in self.__dict_id2tx:
                if tx_id not in range(1, 1 + self.__cue_num) and rx_id != 0:
                    channel_link_loss = rx_channel.get_link_loss(tx_id)
                    channel_link_loss = pow(10, -channel_link_loss / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                    channel_link_loss = -10 * math.log10(channel_link_loss)
                    rx_channel.set_link_loss_cell(channel_link_loss, tx_id)

        # time_start = time.time()
        # random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # graph_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # time_end = time.time()
        # time_use = (time_end - time_start) / 3600
        # for i_id in range(1, 1 + self.__cue_num + self.__d2d_num):
            # self.__dict_id2device[i_id].update_location_after_spectrum_allocation(time_use)
        min_sir_cue = pow(10, 5 / 10)
        min_sir_v2v = 0
        # print('40车辆位置：' + str(self.__dict_id2device[40].get_x_point())+' '+str(self.__dict_id2device[40].get_y_point()))

        graph_colouring_icc(self.__dict_id2tx, self.__dict_id2rx, self.__cue_num, self.__d2d_num, self.__rb_num, self.__dict_id2channel,
                        self.__dict_id2channel_mmwave, min_sir_cue, min_sir_v2v)

        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num-1 and rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                inter_power = self.__dict_id2rx[rx_id].comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)
                if self.__dict_id2rx[rx_id].get_sinr() > 0:
                    rate = 2.16 * pow(10, 9) * math.log2(1 + self.__dict_id2rx[rx_id].get_sinr())
                else:
                    rate = 0
            else:   # 计算蜂窝用户
                inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                if rx_id != 0:
                    if self.__dict_id2rx[rx_id].get_sinr() > 0:
                        rate = 1.8 * pow(10, 5) * math.log2(1 + self.__dict_id2rx[rx_id].get_sinr())
                    else:
                        rate = 0
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                rb_id_use = self.__dict_id2rx[rx_id].get_allocated_rb()[0]
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                self.__dict_tx_id2sinr[tx_id] = sinr
                print('D2D接收机ID:' + str(rx_id) + ' '+str(rb_id_use) + ' SINR:' + str(sinr))
                self.__list_d2d_sinr_random.append(sinr)
                self.__list_d2d_rate_random.append(rate)
            else:  # CUE
                for tx_id in sinr:
                    rb_id_use = self.__dict_id2tx[tx_id].get_allocated_rb()[0]
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    print('基站对应的发射机ID:' + str(tx_id) + ' '+str(rb_id_use) +' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_random.append(sinr[tx_id])

    def update_location_slot(self, slot):
        # 频谱分配结束后的车辆位置
        for i_id in range(1 + self.__cue_num, 1 + self.__cue_num + self.__d2d_num * 2):
            x_point = self.__dict_id2device[i_id].get_x_point() + self.__dict_id2device[i_id].get_direction() * \
                      self.__dict_id2device[i_id].get_v() / 3600
            y_point = self.__dict_id2device[i_id].get_y_point()
            self.__dict_id2device[i_id].set_location(x_point, y_point)
        for channel in self.__dict_id2channel:
            channel = self.__dict_id2channel[channel]
            rx = self.__dict_id2rx[channel.get_rx_id()]
            if rx.get_id() != 0:
                tx = self.__dict_id2tx[rx.get_tx_id()]
                channel.update_link_loss_cell(tx, rx)
                channel.update_link_loss_mmwave(tx, rx, self)
            else:
                for id in range(1, 1 + self.__cue_num):
                    tx = self.__dict_id2tx[id]
                    channel.update_link_loss_cell(tx, rx)
        if (slot + 1) % 1== 0:
            for tx in self.__dict_id2tx:
                tx = self.__dict_id2tx[tx]
                tx.initial_user_location(self.__highway)
            for rx in self.__dict_id2rx:
                rx = self.__dict_id2rx[rx]
                if rx.get_id() != 0:
                    rx.initial_user_location(self.__highway, self.__dict_id2tx[rx.get_tx_id()])
            for channel in self.__dict_id2channel:
                channel = self.__dict_id2channel[channel]
                rx = self.__dict_id2rx[channel.get_rx_id()]
                if rx.get_id() != 0:
                    tx = self.__dict_id2tx[rx.get_tx_id()]
                    channel.update_link_loss_cell(tx, rx)
                    channel.update_link_loss_mmwave(tx, rx, self)
                else:
                    for id in range(1, 1 + self.__cue_num):
                        tx = self.__dict_id2tx[id]
                        channel.update_link_loss_cell(tx, rx)



    def random_allocation_work(self, slot):
        print('--------------random allocation--------------')
        random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # 计算SINR
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            inter = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                self.__dict_tx_id2sinr[tx_id] = sinr
                # print('D2D接收机ID:' + str(rx_id) + ' SINR:' + str(sinr))
                self.__list_d2d_sinr_random.append(sinr)
            else:  # CUE
                for tx_id in sinr:
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    # print('基站对应的发射机ID:' + str(tx_id) + ' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_random.append(sinr[tx_id])

    def resource_coalitional_games_work(self):
        for rx_id in self.__dict_id2rx:
            rx = self.__dict_id2rx[rx_id]
            rx_channel = self.__dict_id2channel[rx_id]
            if rx_id != 0:
                rx_channel_mmwave = self.__dict_id2channel_mmwave[rx_id]
                for tx_id in self.__dict_id2tx:
                    if tx_id not in range(1, 1 + self.__cue_num):
                        channel_link_loss_mm = rx_channel_mmwave.get_link_loss_mmwave(tx_id)
                        channel_link_loss_mm = pow(10, -channel_link_loss_mm / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                        channel_link_loss_mm = -10 * math.log10(channel_link_loss_mm)
                        rx_channel_mmwave.set_link_loss_mmwave(channel_link_loss_mm, tx_id)
            for tx_id in self.__dict_id2tx:
                if tx_id not in range(1, 1 + self.__cue_num) and rx_id != 0:
                    channel_link_loss = rx_channel.get_link_loss(tx_id)
                    channel_link_loss = pow(10, -channel_link_loss / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                    channel_link_loss = -10 * math.log10(channel_link_loss)
                    rx_channel.set_link_loss_cell(channel_link_loss, tx_id)
        # time_start = time.time()
        # random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # graph_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        # time_end = time.time()
        # time_use = (time_end - time_start) / 3600
        # for i_id in range(1, 1 + self.__cue_num + self.__d2d_num):
            # self.__dict_id2device[i_id].update_location_after_spectrum_allocation(time_use)
        min_sir_cue = pow(10, 5 / 10)
        min_sir_v2v = 0
        # print('40车辆位置：' + str(self.__dict_id2device[40].get_x_point())+' '+str(self.__dict_id2device[40].get_y_point()))

        resource_coalitional_games(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num, self.__cue_num, self.__d2d_num, self.__dict_id2channel,
                                   self.__dict_id2channel_mmwave)

        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num-1 and rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                inter_power = self.__dict_id2rx[rx_id].comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)
                if self.__dict_id2rx[rx_id].get_sinr() > 5:
                    rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                else:
                    rate = 0
                # rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            else:   # 计算蜂窝用户
                inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                if rx_id != 0:
                    if self.__dict_id2rx[rx_id].get_sinr() > 5:
                        rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                    else:
                        rate = 0
                    # rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                rb_id_use = self.__dict_id2rx[rx_id].get_allocated_rb()[0]
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                self.__dict_tx_id2sinr[tx_id] = sinr
                print('D2D接收机ID:' + str(rx_id) + ' '+str(rb_id_use) + ' SINR:' + str(sinr))
                self.__list_d2d_sinr_random.append(sinr)
                self.__list_d2d_rate_random.append(rate)
            else:  # CUE
                for tx_id in sinr:
                    rb_id_use = self.__dict_id2tx[tx_id].get_allocated_rb()[0]
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    print('基站对应的发射机ID:' + str(tx_id) + ' '+str(rb_id_use) +' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_random.append(sinr[tx_id])

    def resource_YeLi_work(self):
        Resource_management_by_YeLi(self.__dict_id2tx, self.__dict_id2rx, self.__dict_id2channel, self.__rb_num,
                                    self.__cue_num, self.__d2d_num, min_sinr_cue=pow(10, 5/10), min_sinr_d2d=pow(10, 5/10))
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
            if rx_id != 0:
                if self.__dict_id2rx[rx_id].get_sinr() > 0:
                    rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                else:
                    rate = 0
                rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                rb_id_use = self.__dict_id2rx[rx_id].get_allocated_rb()[0]
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                self.__dict_tx_id2sinr[tx_id] = sinr
                print('D2D接收机ID:' + str(rx_id) + ' '+str(rb_id_use) + ' SINR:' + str(sinr))
                self.__list_d2d_sinr_random.append(sinr)
                self.__list_d2d_rate_random.append(rate)
            else:  # CUE
                for tx_id in sinr:
                    rb_id_use = self.__dict_id2tx[tx_id].get_allocated_rb()[0]
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    print('基站对应的发射机ID:' + str(tx_id) + ' '+str(rb_id_use) +' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_random.append(sinr[tx_id])

    def resource_fair_graph(self):
        for rx_id in self.__dict_id2rx:
            rx = self.__dict_id2rx[rx_id]
            rx_channel = self.__dict_id2channel[rx_id]
            if rx_id != 0:
                rx_channel_mmwave = self.__dict_id2channel_mmwave[rx_id]
                for tx_id in self.__dict_id2tx:
                    if tx_id not in range(1, 1 + self.__cue_num):
                        channel_link_loss_mm = rx_channel_mmwave.get_link_loss_mmwave(tx_id)
                        channel_link_loss_mm = pow(10, -channel_link_loss_mm / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                        channel_link_loss_mm = -10 * math.log10(channel_link_loss_mm)
                        rx_channel_mmwave.set_link_loss_mmwave(channel_link_loss_mm, tx_id)
            for tx_id in self.__dict_id2tx:
                if tx_id not in range(1, 1 + self.__cue_num) and rx_id != 0:
                    channel_link_loss = rx_channel.get_link_loss(tx_id)
                    channel_link_loss = pow(10, -channel_link_loss / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                    channel_link_loss = -10 * math.log10(channel_link_loss)
                    rx_channel.set_link_loss_cell(channel_link_loss, tx_id)
        graph_colouring_fair_graph(self.__dict_id2tx, self.__dict_id2rx, self.__cue_num, self.__d2d_num, self.__rb_num, self.__dict_id2channel,
                        self.__dict_id2channel_mmwave, min_sir_cue=pow(10, 5/10), min_sir_v2v=pow(10, 5/10))
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num - 1 and rx_id in range(
                    1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                inter_power = self.__dict_id2rx[rx_id].comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel,
                                                                        self.__dict_id2rx)
                if self.__dict_id2rx[rx_id].get_sinr() > 0:
                    rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                else:
                    rate = 0
                rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            else:  # 计算蜂窝用户
                inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                if rx_id != 0:
                    if self.__dict_id2rx[rx_id].get_sinr() > 0:
                        rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                    else:
                        rate = 0
                    rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                rb_id_use = self.__dict_id2rx[rx_id].get_allocated_rb()[0]
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                self.__dict_tx_id2sinr[tx_id] = sinr
                print('D2D接收机ID:' + str(rx_id) + ' ' + str(rb_id_use) + ' SINR:' + str(sinr))
                self.__list_d2d_sinr_fair.append(sinr)
                self.__list_d2d_rate_fair.append(rate)
            else:  # CUE
                for tx_id in sinr:
                    rb_id_use = self.__dict_id2tx[tx_id].get_allocated_rb()[0]
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    print('基站对应的发射机ID:' + str(tx_id) + ' ' + str(rb_id_use) + ' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_fair.append(sinr[tx_id])


    def dqn_test_work(self, slot, agent):

        print('--------------DQN-test------------')
        tx_id2sinr_sort_list = {}

        for rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__cue_num + self.__d2d_num * 2):  # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            # 获取接收机位置
            temp_rx_x = temp_rx.get_x_point()
            temp_rx_y = temp_rx.get_y_point()
            tx_id = temp_rx.get_tx_id()
            temp_tx = self.__dict_id2tx[tx_id]
            temp_tx.clear_previous_neibour_rb()
            for second_tx_id in self.__dict_id2tx:
                if second_tx_id != tx_id:  # and second_tx_id not in range(1, 1+self.__cue_num):  # 其他V2V
                    second_tx = self.__dict_id2tx[second_tx_id]
                    second_tx_x = second_tx.get_x_point()
                    second_tx_y = second_tx.get_y_point()
                    distance = get_distance(temp_rx_x, temp_rx_y, second_tx_x, second_tx_y)
                    if distance < 100:
                        temp_tx.add_neibour_V2V(second_tx_id)

        # 选择信道质量最不好的用户有限分配
        for rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__cue_num + self.__d2d_num * 2):  # 遍历所有的接收机
            rx = self.__dict_id2rx[rx_id]
            if rx.get_allocated_rb()[0] == self.__rb_num-1 and rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                inter_power = rx.comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)
            else:   # 计算蜂窝用户
                inter_power = rx.comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
            sinr = rx.get_sinr()
            tx_id2sinr_sort_list[rx.get_tx_id()] = sinr
        sort_list = sorted(tx_id2sinr_sort_list.items(), key=lambda asd: asd[1])
        for tuple in sort_list:
            tx_id = tuple[0]
            tx = self.__dict_id2tx[tx_id]
            # 观察环境
            self.obvious_previous_rb(tx)  # 观察previous_rb
            self.update_neibour_rb(tx)  # 观察邻居的 rb
            self.update_during_slot()  # 更新信道 3种实时csi
            self.interference_aware_tx(tx)  # 观察 干扰

            self.__dict_id2rx[0].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
            sinr_cue_dict = self.__dict_id2rx[0].get_sinr()
            for tx_id_2 in range(1, 1 + self.__cue_num):
                self.__dict_tx_id2sinr[tx_id_2] = sinr_cue_dict[tx_id_2]

            rb_id = tx.get_allocated_rb()[0]
            if rb_id != self.__rb_num - 1:  # V2V使用蜂窝频段
                cue_sinr = self.__dict_tx_id2sinr[rb_id + 1]  # 蜂窝用户的信噪比
                tx.set_cue_sinr(cue_sinr)
            else:
                tx.set_cue_sinr(-1)

            state = tx.get_state(self.__rb_num, self)

            action = agent.choose_action(state)

        '''for channel_id in self.__dict_id2channel:
            channel = self.__dict_id2channel[channel_id]
            for tx_id in range(1 + self.__cue_num, 1 + self.__cue_num + self.__d2d_num):
                if channel_id != 0:
                    value = channel.get_link_loss_mmwave(tx_id) + random.gauss(0, 1)
                    channel.set_link_loss_mmwave(value, tx_id)
                value = channel.get_link_loss(tx_id) + random.gauss(0, 1)
                channel.set_link_loss_cell(value, tx_id)'''

        # 计算SINR
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num-1 and rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                inter_power = self.__dict_id2rx[rx_id].comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)
                rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            else:   # 计算蜂窝用户
                inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                if rx_id != 0:
                    rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
            sinr = self.__dict_id2rx[rx_id].get_sinr()
            if type(sinr) == float:  # D2D
                rb_id_use = self.__dict_id2rx[rx_id].get_allocated_rb()[0]
                tx_id = self.__dict_id2rx[rx_id].get_tx_id()
                tx = self.__dict_id2tx[tx_id]

                self.__dict_tx_id2sinr[tx_id] = sinr
                if rate > tx.get_arrival_rate():
                    self.__success_transmission += 1
                else:
                    self.__failed_transmission += 1
                rate = round(rate / 1000,1)

                print('D2D接收机ID:' + str(rx_id) + ' '+str(rb_id_use) + ' SINR:' + str(sinr) + ' Rate:' + str(rate))
                self.__list_d2d_sinr_reinforcement.append(sinr)
                self.__list_d2d_rate_reinforcement.append(rate)
            else:  # CUE
                for tx_id in sinr:
                    rb_id_use = self.__dict_id2tx[tx_id].get_allocated_rb()[0]
                    self.__dict_tx_id2sinr[tx_id] = sinr[tx_id]
                    print('基站对应的发射机ID:' + str(tx_id) + ' '+str(rb_id_use) +' SINR:' + str(sinr[tx_id]))
                    self.__list_cue_sinr_reinforcement .append(sinr[tx_id])


    def vehicle_train_ly(self, slot, agent):
        states = []
        next_states = []
        actions = []
        rewards = []
        # maddpg.slot = slot

        print('--------------rb allocation train------------' + str(slot))
        if slot == 0:  # first slot, random allocation
            random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)


        # 得到每个V2V节点的邻居
        for rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__cue_num + self.__d2d_num * 2):  # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            # 获取接收机位置
            temp_rx_x = temp_rx.get_x_point()
            temp_rx_y = temp_rx.get_y_point()
            tx_id = temp_rx.get_tx_id()
            temp_tx = self.__dict_id2tx[tx_id]
            temp_tx.clear_previous_neibour_rb()
            for second_tx_id in self.__dict_id2tx:
                if second_tx_id != tx_id:  # and second_tx_id not in range(1, 1+self.__cue_num):  # 其他V2V
                    second_tx = self.__dict_id2tx[second_tx_id]
                    second_tx_x = second_tx.get_x_point()
                    second_tx_y = second_tx.get_y_point()
                    distance = get_distance(temp_rx_x, temp_rx_y, second_tx_x, second_tx_y)
                    if distance < 100:
                        temp_tx.add_neibour_V2V(second_tx_id)
        total_reward = 0
        for tx_id in self.__dict_id2tx:
            temp_tx = self.__dict_id2tx[tx_id]
            if temp_tx.get_type() == 'D2DTx':
                # 观察环境
                self.obvious_previous_rb(temp_tx)  # 观察previous_rb
                self.update_neibour_rb(temp_tx)  # 观察邻居的 rb
                self.update_during_slot()  # 更新信道 3种实时csi
                self.interference_aware_tx(temp_tx)  # 观察 干扰

                self.__dict_id2rx[0].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                sinr_cue_dict = self.__dict_id2rx[0].get_sinr()
                for tx_id_2 in range(1, 1 + self.__cue_num):
                    self.__dict_tx_id2sinr[tx_id_2] = sinr_cue_dict[tx_id_2]

                rb_id = temp_tx.get_allocated_rb()[0]
                if rb_id != self.__rb_num - 1:  # V2V使用蜂窝频段
                    cue_sinr = self.__dict_tx_id2sinr[rb_id + 1]  # 蜂窝用户的信噪比
                    temp_tx.set_cue_sinr(cue_sinr)
                else:
                    temp_tx.set_cue_sinr(-1)

                state = temp_tx.get_state(self.__rb_num, self)
                states.append(state)

                # 根据观测到的环境选择动作
                if slot < 2000:
                    action = agent.choose_random_action()
                else:
                    action = agent.choose_action(state)
                actions.append(action)
                temp_tx.do_action(action, self.__dict_id2rx, self.__rb_num)

                # 计算reward 计算cue
                self.__dict_id2rx[0].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                sinr_cue_dict = self.__dict_id2rx[0].get_sinr()
                for tx_id_2 in range(1, 1 + self.__cue_num):
                    self.__dict_tx_id2sinr[tx_id_2] = sinr_cue_dict[tx_id_2]

                rx_id = temp_tx.get_rx_id()
                rx = self.__dict_id2rx[rx_id]
                rb_id = rx.get_allocated_rb()[0]
                if rb_id != self.__rb_num - 1:
                    rx.comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                else:
                    rx.comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)

                reward_ec = self.compute_reward_tx(temp_tx)
                reward_ec = reward_ec / pow(10, 7)
                number_V2V = 0
                for neibours in temp_tx.get_neibor_V2V():
                    if neibours in range(1 + self.__cue_num, 1 + self.__cue_num + self.__d2d_num):
                        number_V2V += 1
                        neibours_tx = self.__dict_id2tx[neibours]
                        reward_ec += self.compute_reward_tx(neibours_tx) / pow(10, 7)
                reward_ec = round(reward_ec / (1 + number_V2V), 1)
                if reward_ec > 200:
                    reward_ec = 200
                rb_id = temp_tx.get_allocated_rb()[0]
                d2d_sinr = rx.get_sinr()
                if d2d_sinr < -10:
                    reward_ec = -20
                if rb_id != self.__rb_num - 1:  # V2V使用蜂窝频段
                    cue_sinr = self.__dict_tx_id2sinr[rb_id + 1]  # 蜂窝用户的信噪比
                    if cue_sinr < 5:
                        reward_ec = -20

                    temp_tx.set_cue_sinr(cue_sinr)
                else:
                    temp_tx.set_cue_sinr(-1)
                total_reward += reward_ec
                print('D2DTx ' + str(tx_id) + ' reward: ' + str(reward_ec))

                self.__list_single_reward.append(reward_ec)

                rewards.append(reward_ec)

                # 观测之后的状态
                self.obvious_previous_rb(temp_tx)  # 观察previous_rb
                self.update_neibour_rb(temp_tx)  # 观察邻居的 rb
                self.interference_aware_tx(temp_tx)  # 观察 干扰
                self.update_during_slot()  # 更新信道 3种实时csi

                state = temp_tx.get_state(self.__rb_num, self)
                next_states.append(state)

                pass
        print('total reward: ' + str(total_reward))
        self.__list_total_reward.append(total_reward)

        # 训练
        for i in range(len(states)):
            agent.store_transition(states[i], actions[i], rewards[i], next_states[i])

        if slot > 2000:
            agent.learn()

        # 更新
        # if slot % 1 == 0:
        # self.update(slot)  # update environment

    def vehicle_train(self, slot, agent):
        states = []
        next_states = []
        actions = []
        rewards = []
        # maddpg.slot = slot

        print('--------------rb allocation train------------' + str(slot))
        if slot == 0:  # first slot, random allocation
            random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)

        # state中 当前时隙观察到的上一时隙使用的rb
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            tx_id = temp_rx.get_tx_id()

            # 计算智能体的在每个频段的干扰功率
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num - 1 and rx_id in range(
                    1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                temp_tx = self.__dict_id2tx[tx_id]
                previous_rb = temp_tx.get_allocated_rb()[0]  # 前一时隙使用的rb
                temp_tx.set_previous_rb(previous_rb)
            else:
                if rx_id != 0:  # 蜂窝v2v接收机
                    temp_tx = self.__dict_id2tx[tx_id]
                    previous_rb = temp_tx.get_allocated_rb()[0]  # 前一时隙使用的rb
                    temp_tx.set_previous_rb(previous_rb)
                else:  # 蜂窝用户
                    self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                    sinr_cue_dict = self.__dict_id2rx[rx_id].get_sinr()
                    for tx_id in range(1, 1 + self.__cue_num):
                        self.__dict_tx_id2sinr[tx_id] = sinr_cue_dict[tx_id]

        # 得到每个V2V节点的邻居 state中 当前时隙观察到的上一时隙邻居用户使用的rb
        for rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__cue_num + self.__d2d_num * 2):  # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            # 获取接收机位置
            temp_rx_x = temp_rx.get_x_point()
            temp_rx_y = temp_rx.get_y_point()
            tx_id = temp_rx.get_tx_id()
            temp_tx = self.__dict_id2tx[tx_id]
            temp_tx.clear_previous_neibour_rb()
            for second_tx_id in self.__dict_id2tx:
                if second_tx_id != tx_id:  # and second_tx_id not in range(1, 1+self.__cue_num):  # 其他V2V
                    second_tx = self.__dict_id2tx[second_tx_id]
                    second_tx_x = second_tx.get_x_point()
                    second_tx_y = second_tx.get_y_point()
                    distance = get_distance(temp_rx_x, temp_rx_y, second_tx_x, second_tx_y)
                    if distance < 50:
                        rb_neibour = second_tx.get_allocated_rb()[0]
                        if rb_neibour not in temp_tx.get_neibor_rb():
                            temp_tx.add_neibour_rb(rb_neibour)
                        temp_tx.add_neibour_V2V(second_tx_id)

        # state中 当前时隙观察到的上衣时隙干扰感知
        self.interference_aware()

        # 更新了当前时隙的 3种实时csi
        self.update_during_slot()

        for tx_id in self.__dict_id2tx:
            temp_tx = self.__dict_id2tx[tx_id]
            if temp_tx.get_type() == 'D2DTx':
                state = temp_tx.get_state(self.__rb_num, self)
                states.append(state)
                if slot < 1000:
                    action = agent.choose_random_action()
                else:
                    action = agent.choose_action(state)
                actions.append(action)
                temp_tx.do_action(action, self.__dict_id2rx, self.__rb_num)
                pass

        # 计算动作执行后的reward
        total_reward = 0
        for tx_id in self.__dict_id2tx:
            temp_tx = self.__dict_id2tx[tx_id]
            if temp_tx.get_type() == 'D2DTx':  # 计算每一个V2V的reward
                reward_ec = self.compute_reward_tx(temp_tx)
                print('D2DTx ' + str(tx_id) + ' reward: ' + str(reward_ec))
                rb_id = temp_tx.get_allocated_rb()[0]
                if rb_id != self.__rb_num - 1:  # V2V使用蜂窝频段
                    cue_sinr = self.__dict_tx_id2sinr[rb_id + 1]  # 蜂窝用户的信噪比
                    print('D2DTx ' + str(tx_id) + ' reward: ' + ' ' + str(cue_sinr))
                    if cue_sinr < 5:
                        reward_ec = - 1 * pow(10, 9)
                        print('D2DTx ' + str(tx_id) + ' reward: ' + str(reward_ec))

                rewards.append(reward_ec)
                total_reward += reward_ec
        print('total reward: ' + str(total_reward))

        if slot > 1000:
            self.__list_total_reward.append(total_reward)

        # post-state中 上一时隙使用的rb
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            tx_id = temp_rx.get_tx_id()

            # 计算智能体的在每个频段的干扰功率
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num - 1 and rx_id in range(
                    1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                temp_tx = self.__dict_id2tx[tx_id]
                previous_rb = temp_tx.get_allocated_rb()[0]  # 前一时隙使用的rb
                temp_tx.set_previous_rb(previous_rb)
            else:
                if rx_id != 0:  # 蜂窝v2v接收机
                    temp_tx = self.__dict_id2tx[tx_id]
                    previous_rb = temp_tx.get_allocated_rb()[0]  # 前一时隙使用的rb
                    temp_tx.set_previous_rb(previous_rb)
                else:  # 蜂窝用户
                    self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                    sinr_cue_dict = self.__dict_id2rx[rx_id].get_sinr()
                    for tx_id in range(1, 1 + self.__cue_num):
                        self.__dict_tx_id2sinr[tx_id] = sinr_cue_dict[tx_id]

        # post-state后 上一时隙邻居用户使用的rb
        for rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__cue_num + self.__d2d_num * 2):
            # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            # 获取接收机位置
            temp_rx_x = temp_rx.get_x_point()
            temp_rx_y = temp_rx.get_y_point()
            tx_id = temp_rx.get_tx_id()
            temp_tx = self.__dict_id2tx[tx_id]
            temp_tx.clear_previous_neibour_rb()
            for second_tx_id in self.__dict_id2tx:
                if second_tx_id != tx_id:  # and second_tx_id not in range(1, 1+self.__cue_num):  # 其他V2V
                    second_tx = self.__dict_id2tx[second_tx_id]
                    second_tx_x = second_tx.get_x_point()
                    second_tx_y = second_tx.get_y_point()
                    distance = get_distance(temp_rx_x, temp_rx_y, second_tx_x, second_tx_y)
                    if distance < 100:
                        rb_neibour = second_tx.get_allocated_rb()[0]
                        if rb_neibour not in temp_tx.get_neibor_rb():
                            temp_tx.add_neibour_rb(rb_neibour)
                        temp_tx.add_neibour_V2V(second_tx_id)

        # 上一时隙 干扰感知
        self.interference_aware()

        # post-state 的 瞬时信道信息
        self.update_during_slot()

        # 记录之后的状态
        for tx_id in self.__dict_id2tx:
            temp_tx = self.__dict_id2tx[tx_id]
            if temp_tx.get_type() == 'D2DTx':
                state = temp_tx.get_state(self.__rb_num, self)
                next_states.append(state)

        # 训练
        for i in range(len(states)):
            agent.store_transition(states[i], actions[i], rewards[i], next_states[i])
        if slot > 1000:
            agent.learn()

        # 更新
        if slot % 2 == 0:
            self.update(slot)  # update environment


    def resource_dqn_train(self, slot, agent):
        states = []
        next_states = []
        actions = []
        rewards = []
        # maddpg.slot = slot

        print('--------------rb allocation train------------' + str(slot))
        if slot == 0:  # first slot, random allocation
            random_allocation(self.__dict_id2tx, self.__dict_id2rx, self.__rb_num)
        else:
            for tx_id in self.__dict_id2tx:
                temp_tx = self.__dict_id2tx[tx_id]
                if temp_tx.get_type() == 'D2DTx':
                    state = temp_tx.get_state(self.__rb_num, self)
                    states.append(state)
                    if slot < 1000:
                        action = agent.choose_random_action()
                    else:
                        action = agent.choose_action(state)
                    actions.append(action)
                    temp_tx.do_action(action, self.__dict_id2rx, self.__rb_num)
                    pass

            # for tx_id in self.__dict_id2tx:
                # temp_tx = self.__dict_id2tx[tx_id]
                # if temp_tx.get_type() == 'D2DTx':
                    # self.update_neighbor_rb(temp_tx)  # the channels selected by neighbors in the previous time

        # 计算蜂窝用户的瞬时信噪比
        for rx_id in self.__dict_id2rx:  # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            tx_id = temp_rx.get_tx_id()

            # 计算智能体的在每个频段的干扰功率
            if self.__dict_id2rx[rx_id].get_allocated_rb()[0] == self.__rb_num-1 and rx_id in range(1 + self.__cue_num + self.__d2d_num, 1 + self.__d2d_num * 2 + self.__cue_num):
                #  毫米波v2v接收机
                # 计算接收信噪比 并 记录干扰功率  需要修改为有效容量
                # inter_power = self.__dict_id2rx[rx_id].comp_mmwave_sinr(self.__dict_id2tx, self.__dict_id2channel, self.__dict_id2rx)
                # if self.__dict_id2rx[rx_id].get_sinr() > 0:
                     # rate_v2v += 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                # else:
                     # rate_v2v += 0
                # rate = 2.16 * pow(10, 9) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                temp_tx = self.__dict_id2tx[tx_id]
                temp_tx.previous_rb = temp_tx.get_allocated_rb()[0]  # 前一时隙使用的rb
                self.__dict_tx_id2sinr[tx_id] = self.__dict_id2rx[rx_id].get_sinr()  # 毫米波v2v的sinr
            else:
                # 计算V2V接收机干扰功率
                # inter_power = self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                if rx_id != 0:  # 蜂窝v2v接收机
                    #if self.__dict_id2rx[rx_id].get_sinr() > 0:
                        # rate_v2v += 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                    # else:
                        # rate_v2v += 0
                    # rate = 1.8 * pow(10, 5) * math.log2(1 + pow(10, self.__dict_id2rx[rx_id].get_sinr() / 10))
                    temp_tx = self.__dict_id2tx[tx_id]
                    temp_tx.previous_rb = temp_tx.get_allocated_rb()[0]  # 前一时隙使用的rb

                    # self.__dict_tx_id2sinr[tx_id] = self.__dict_id2rx[rx_id].get_sinr()  # 蜂窝v2v的sinr
                else:  # 蜂窝用户
                    self.__dict_id2rx[rx_id].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
                    sinr_cue_dict = self.__dict_id2rx[rx_id].get_sinr()
                    for tx_id in range(1, 1 + self.__cue_num):
                        self.__dict_tx_id2sinr[tx_id] = sinr_cue_dict[tx_id]
            # sinr = self.__dict_id2rx[rx_id].get_sinr()

        # 得到每个V2V节点的邻居
        for rx_id in range(1+self.__cue_num+self.__d2d_num, 1+self.__cue_num+self.__d2d_num*2):  # 遍历所有的接收机
            temp_rx = self.__dict_id2rx[rx_id]
            # 获取接收机位置
            temp_rx_x = temp_rx.get_x_point()
            temp_rx_y = temp_rx.get_y_point()
            tx_id = temp_rx.get_tx_id()
            temp_tx = self.__dict_id2tx[tx_id]
            temp_tx.clear_previous_neibour_rb()
            for second_tx_id in self.__dict_id2tx:
                if second_tx_id != tx_id:  # and second_tx_id not in range(1, 1+self.__cue_num):  # 其他V2V
                    second_tx = self.__dict_id2tx[second_tx_id]
                    second_tx_x = second_tx.get_x_point()
                    second_tx_y = second_tx.get_y_point()
                    distance = get_distance(temp_rx_x, temp_rx_y, second_tx_x, second_tx_y)
                    if distance < 50:
                        rb_neibour = second_tx.get_allocated_rb()[0]
                        if rb_neibour not in temp_tx.get_neibor_rb():
                            temp_tx.add_neibour_rb(rb_neibour)
                        temp_tx.add_neibour_V2V(second_tx_id)

        # 上一时隙 干扰感知
        self.interference_aware()


        if slot >0:
            # 计算 当前状态 reward
            total_reward = 0
            for tx_id in self.__dict_id2tx:
                temp_tx = self.__dict_id2tx[tx_id]
                if temp_tx.get_type() == 'D2DTx':  # 计算每一个V2V的reward
                    reward_ec = self.compute_reward_tx(temp_tx)
                    rb_id = temp_tx.get_allocated_rb()[0]
                    if rb_id != self.__rb_num-1:  # V2V使用蜂窝频段
                        cue_sinr = self.__dict_tx_id2sinr[rb_id+1]  # 蜂窝用户的信噪比
                        if cue_sinr < 0:
                            reward_ec = - 100
                    print('D2DTx ' + str(tx_id) + ' reward: ' + str(reward_ec))
                    rewards.append(reward_ec)
                    total_reward += reward_ec


            self.update(slot)  # update environment
            # 更新车辆位置和信道

            for tx_id in self.__dict_id2tx:
                temp_tx = self.__dict_id2tx[tx_id]
                if temp_tx.get_type() == 'D2DTx':
                    state = temp_tx.get_state(self.__rb_num, self)
                    next_states.append(state)

            for i in range(len(states)):
                agent.store_transition(states[i], actions[i], rewards[i], next_states[i])
            if slot > 1000:
                agent.learn()

            # print('total reward: ' + str(total_reward))
            # self.save_reward(slot, total_reward)

            # print('capacity: ' + str(capacity))
            # self.save_capacity(slot, capacity)

            # print('D2D capacity: ' + str(d2d_capacity))
            # self.save_d2d_capacity(slot, d2d_capacity)

        self.__list_slot.append(slot)

    def update_neibour_rb(self, tx):
        tx.clear_neibour_rb()
        for second_tx_id in tx.get_neibor_V2V():
            second_tx = self.__dict_id2tx[second_tx_id]
            rb_neibour = second_tx.get_allocated_rb()[0]
            if rb_neibour not in tx.get_neibor_rb():
                tx.add_neibour_rb(rb_neibour)


    # 邻居
    def find_neibour(self, tx):
        rx = self.__dict_id2rx[tx.get_rx_id()]
        # 获取接收机位置
        temp_rx_x = rx.get_x_point()
        temp_rx_y = rx.get_y_point()
        tx_id = rx.get_tx_id()
        tx.clear_previous_neibour_rb()
        for second_tx_id in self.__dict_id2tx:
            if second_tx_id != tx_id:  # and second_tx_id not in range(1, 1+self.__cue_num):  # 其他V2V
                second_tx = self.__dict_id2tx[second_tx_id]
                second_tx_x = second_tx.get_x_point()
                second_tx_y = second_tx.get_y_point()
                distance = get_distance(temp_rx_x, temp_rx_y, second_tx_x, second_tx_y)
                if distance < 50:
                    rb_neibour = second_tx.get_allocated_rb()[0]
                    if rb_neibour not in tx.get_neibor_rb():
                        tx.add_neibour_rb(rb_neibour)
                    tx.add_neibour_V2V(second_tx_id)



    # 干扰感知
    def interference_aware(self):
        # 每一个V2V
        for tx_id in range(1+self.__cue_num, 1 + self.__cue_num+self.__d2d_num):
            tx = self.__dict_id2tx[tx_id]  # V2V 发射机
            tx.clear_previous_inter()
            tx_neibour = tx.get_neibor_V2V()  # 智能体的邻居 发射机
            rb_neibour = tx.get_neibor_rb()  # 智能体邻居使用的rb
            for rb_id in range(self.__rb_num):  # 计算每一个rb上感知到的干扰
                if rb_id not in rb_neibour:  # 如果邻居未使用此rb 则功率为0
                    tx.add_prevousr_inter_neibour(0)
                else:
                    power_rb = 0  # 每一个rb的累计干扰
                    for second_tx_id in tx_neibour:
                        second_tx = self.__dict_id2tx[second_tx_id]  # 邻居的发射机
                        if rb_id == second_tx.get_allocated_rb()[0]:  # 使用当前rb的邻居发射机
                            if rb_id == self.__rb_num-1:  # 毫米波频段干扰计算
                                inter_power = second_tx.get_power()  # dBm
                                inter_power = pow(10, (inter_power - 30) / 10)  # W
                                inter_channel = self.__dict_id2channel_mmwave[tx.get_rx_id()]  # 当前接收机的信道矩阵
                                inter_link_loss = inter_channel.get_link_loss_mmwave(second_tx_id)  # dB
                                inter_gain = pow(10, -inter_link_loss / 10)

                                # 接收机为自己本身，发射机为干扰发射机
                                rx = self.__dict_id2rx[tx.get_rx_id()]
                                tx_a_gain = second_tx.tx_a_gain_mmwave(rx.get_x_point(), rx.get_y_point(), self.__dict_id2rx)
                                tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
                                rx_a_gain = rx.rx_a_gain_mmwave(second_tx.get_x_point(), second_tx.get_y_point(),
                                                                  self.__dict_id2tx)
                                rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
                                power_rb += inter_power * inter_gain * tx_a_gain * rx_a_gain
                            else:  # 蜂窝频段干扰计算
                                inter_power = second_tx.get_power()  # dBm
                                inter_power = pow(10, (inter_power - 30) / 10)  # W
                                inter_channel = self.__dict_id2channel[tx.get_rx_id()]
                                inter_link_loss = inter_channel.get_link_loss(second_tx_id)  # dB
                                inter_gain = pow(10, -inter_link_loss / 10)  # * math.sqrt(pow(random.gauss(0, 1), 2))
                                power_rb += inter_power * inter_gain
                    tx.add_prevousr_inter_neibour(power_rb)

    def interference_aware_tx(self, tx):
        tx.clear_previous_inter()
        tx_neibour = tx.get_neibor_V2V()  # 智能体的邻居 发射机
        rb_neibour = tx.get_neibor_rb()  # 智能体邻居使用的rb
        for rb_id in range(self.__rb_num):  # 计算每一个rb上感知到的干扰
            if rb_id not in rb_neibour:  # 如果邻居未使用此rb 则功率为0
                tx.add_prevousr_inter_neibour(0)
            else:
                power_rb = 0  # 每一个rb的累计干扰
                for second_tx_id in tx_neibour:
                    second_tx = self.__dict_id2tx[second_tx_id]  # 邻居的发射机
                    if rb_id == second_tx.get_allocated_rb()[0]:  # 使用当前rb的邻居发射机
                        if rb_id == self.__rb_num - 1:  # 毫米波频段干扰计算
                            inter_power = second_tx.get_power()  # dBm
                            inter_power = pow(10, (inter_power - 30) / 10)  # W
                            inter_channel = self.__dict_id2channel_mmwave[tx.get_rx_id()]  # 当前接收机的信道矩阵
                            inter_link_loss = inter_channel.get_link_loss_mmwave(second_tx_id)  # dB
                            inter_gain = pow(10, -inter_link_loss / 10)

                            # 接收机为自己本身，发射机为干扰发射机
                            rx = self.__dict_id2rx[tx.get_rx_id()]
                            tx_a_gain = second_tx.tx_a_gain_mmwave(rx.get_x_point(), rx.get_y_point(),
                                                                   self.__dict_id2rx)
                            tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
                            rx_a_gain = rx.rx_a_gain_mmwave(second_tx.get_x_point(), second_tx.get_y_point(),
                                                            self.__dict_id2tx)
                            rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
                            power_rb += inter_power * inter_gain * tx_a_gain * rx_a_gain
                        else:  # 蜂窝频段干扰计算
                            inter_power = second_tx.get_power()  # dBm
                            inter_power = pow(10, (inter_power - 30) / 10)  # W
                            inter_channel = self.__dict_id2channel[tx.get_rx_id()]
                            inter_link_loss = inter_channel.get_link_loss(second_tx_id)  # dB
                            inter_gain = pow(10, -inter_link_loss / 10)  # * math.sqrt(pow(random.gauss(0, 1), 2))
                            power_rb += inter_power * inter_gain
                tx.add_prevousr_inter_neibour(power_rb)

    # 计算reward
    def compute_reward_tx(self, tx):
        # 使用蒙特卡洛方法近似得到
        N = 100  # 蒙特卡洛取样次数
        # 当前智能体的邻居节点
        dict_nei = {}
        EC = 0
        band_width = 0
        ec_capacity = 0  # 遍历容量
        e_capacity = 0  # 指数遍历容量
        for index in range(N):
            if tx.get_allocated_rb()[0] != self.__rb_num-1:
                band_width = 1.8 * pow(10, 5)
                white_noise = -134  # -174dBm / Hz
                noise_fig = 5  # dB
                noise_fig = pow(10, noise_fig / 10)  # 线性值
                thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

                # 计算接收目标信号功率
                target_power = tx.get_power()  # dBm
                target_power = pow(10, (target_power - 30) / 10)  # W
                target_channel = self.__dict_id2channel[tx.get_rx_id()]
                target_link_loss = target_channel.get_link_loss(tx.get_id()) + random.gauss(0, 1)   # dB
                target_gain = pow(10, -target_link_loss / 10)
                receive_target_power = target_power * target_gain

                receive_inter_power = 0
                for tx_id in tx.get_neibor_V2V():
                    if tx_id != tx.get_id():
                        if tx.get_allocated_rb()[0] in self.__dict_id2tx[tx_id].get_allocated_rb():
                            inter_tx = self.__dict_id2tx[tx_id]  # 干扰发射机
                            inter_power = inter_tx.get_power()  # dBm
                            inter_power = pow(10, (inter_power - 30) / 10)  # W
                            inter_channel = self.__dict_id2channel[tx.get_rx_id()]
                            inter_link_loss = inter_channel.get_link_loss(tx_id) + random.gauss(0, 1)   # dB
                            inter_gain = pow(10, -inter_link_loss / 10)  # * math.sqrt(pow(random.gauss(0, 1), 2))
                            receive_inter_power += inter_power * inter_gain
                sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
                ec_capacity += math.log2(1 + receive_target_power / (receive_inter_power + thermal_noise_pow))
                e_capacity += pow(math.e, math.log2(1 + receive_target_power / (receive_inter_power + thermal_noise_pow)))
            else:
                band_width = 1.08 * pow(10, 9)
                noise_power = -174 + 10 * math.log10(band_width) + 6
                thermal_noise_pow = pow(10, (noise_power - 30) / 10)

                # 计算接收目标信号功率
                target_power = tx.get_power()  # dBm
                target_power = pow(10, (target_power - 30) / 10)  # W
                target_channel = self.__dict_id2channel_mmwave[tx.get_rx_id()]
                target_link_loss = target_channel.get_link_loss_mmwave(tx.get_id()) + random.gauss(0, 1)  # dB
                target_gain = pow(10, -target_link_loss / 10)

                # 发射机为配对的发射机，接收机为自己
                tx_a_gain = tx.tx_a_gain_mmwave(self.__dict_id2rx[tx.get_rx_id()].get_x_point(), self.__dict_id2rx[tx.get_rx_id()].get_y_point(), self.__dict_id2rx)
                tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
                rx = self.__dict_id2rx[tx.get_rx_id()]
                rx_a_gain = rx.rx_a_gain_mmwave(tx.get_x_point(), tx.get_y_point(), self.__dict_id2tx)
                rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
                receive_target_power = target_power * target_gain * tx_a_gain * rx_a_gain

                # 计算接收干扰信号总功率
                receive_inter_power = 0
                for tx_id in tx.get_neibor_V2V():
                    if tx_id != tx.get_id():
                        if tx.get_allocated_rb()[0] in self.__dict_id2tx[tx_id].get_allocated_rb():
                            inter_tx = self.__dict_id2tx[tx_id]  # 干扰发射机
                            inter_power = inter_tx.get_power()  # dBm
                            inter_power = pow(10, (inter_power - 30) / 10)  # W
                            inter_channel = self.__dict_id2channel_mmwave[tx.get_rx_id()]
                            inter_link_loss = inter_channel.get_link_loss_mmwave(tx_id) + random.gauss(0, 1)  # dB
                            inter_gain = pow(10, -inter_link_loss / 10)

                            # 接收机为自己本身，发射机为干扰发射机
                            tx_a_gain = inter_tx.tx_a_gain_mmwave(tx.get_x_point(), tx.get_y_point(), self.__dict_id2rx)
                            tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
                            rx_a_gain = rx.rx_a_gain_mmwave(inter_tx.get_x_point(), inter_tx.get_y_point(),
                                                              self.__dict_id2tx)
                            rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
                            receive_inter_power += inter_power * inter_gain * tx_a_gain * rx_a_gain

                sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
                ec_capacity += math.log2(1 + receive_target_power / (receive_inter_power + thermal_noise_pow))
                e_capacity += pow(math.e, math.log2(1 + receive_target_power / (receive_inter_power + thermal_noise_pow)))


        ec_capacity = ec_capacity/N
        e_capacity = e_capacity/N

        # 计算错误
        temp = tx.get_outage_probability() * tx.get_arrival_rate() * band_width * ec_capacity
        theta = -1/tx.get_arrival_rate()/tx.get_delay_requirement() * math.log(math.e, temp)

        EC = -1 / theta/pow(10, -6) * math.log(math.e, pow(math.e, -1 * theta * pow(10, -6) * band_width) * e_capacity)

        return EC

    def compute_reward(self, tx):
        reward_ec = self.compute_reward_tx(tx)
        rb_id = tx.get_allocated_rb()[0]
        if rb_id != self.__rb_num - 1:  # V2V使用蜂窝频段
            self.__dict_id2rx[0].comp_sinr(self.__dict_id2tx, self.__dict_id2channel)
            sinr_cue_dict = self.__dict_id2rx[0].get_sinr()
            for tx_id in range(1, 1 + self.__cue_num):
                self.__dict_tx_id2sinr[tx_id] = sinr_cue_dict[tx_id]
            cue_sinr = self.__dict_tx_id2sinr[rb_id + 1]  # 蜂窝用户的信噪比
            if cue_sinr < 0:
                reward_ec = - 100
        print('D2DTx ' + str(tx.get_id()) + ' reward: ' + str(reward_ec))
        return reward_ec

    def obvious_previous_rb(self, tx):
        previous_rb = tx.get_allocated_rb()[0]  # 前一时隙使用的rb
        tx.set_previous_rb(previous_rb)

    def update(self, slot):
        # 更新用户位置 更新信道
        # 更新车辆位置
        for tx in self.__dict_id2tx:
            # if tx not in range(1, 1 + self.__cue_num):
            tx = self.__dict_id2tx[tx]
            tx.initial_user_location(self.__highway)
        for rx in self.__dict_id2rx:
            rx = self.__dict_id2rx[rx]
            if rx.get_id() != 0:
                rx.initial_user_location(self.__highway, self.__dict_id2tx[rx.get_tx_id()])
        bs_channel = self.__dict_id2channel[0]
        for channel in self.__dict_id2channel:
            channel = self.__dict_id2channel[channel]
            rx = self.__dict_id2rx[channel.get_rx_id()]
            if rx.get_id() != 0:
                tx = self.__dict_id2tx[rx.get_tx_id()]
                channel.update_link_loss_cell(tx, rx)
                channel.update_link_loss_mmwave(tx, rx, self)
                tx_id = tx.get_id()
                value = channel.get_link_loss_mmwave(tx_id)
                tx.set_v2v_csi_mmwave(value)
                value = channel.get_link_loss(tx_id)
                tx.set_v2v_csi_cell(value)
                value = bs_channel.get_link_loss(tx_id)
                tx.set_tx2bs_csi(value)
            else:
                for id in range(1, 1 + self.__cue_num):
                    tx = self.__dict_id2tx[id]
                    channel.update_link_loss_cell(tx, rx)
        # 更新车辆进行的服务更新时延约束门限和节点到达率
        # for tx_id in self.__dict_id2tx:
            # if tx_id in range(1 + self.__cue_num, 1 + self.__cue_num + self.__d2d_num):
                # tx = self.__dict_id2tx[tx_id]
                # tx.update_arrival_rate_and_delay(slot)

    def test_update_channel(self):
        for channel in self.__dict_id2channel:
            rx_channel = self.__dict_id2channel[channel]
            if channel != 0:
                for tx_id in self.__dict_id2tx:
                    link_loss_cell = rx_channel.get_link_loss(tx_id) + random.gauss(0, 1)
                    link_loss_mmwave = rx_channel.get_link_loss_mmwave(tx_id) + random.gauss(0, 1)
                    rx_channel.set_link_loss_cell(link_loss_cell, tx_id)
                    rx_channel.set_link_loss_mmwave(link_loss_mmwave, tx_id)


    def update_during_slot(self):
        # 更新用户位置 更新信道
        # 更新车辆位置
        for i_id in range(1 + self.__cue_num, 1 + self.__cue_num + self.__d2d_num * 2):
            x_point = self.__dict_id2device[i_id].get_x_point() + round(self.__dict_id2device[i_id].get_direction() * \
                      self.__dict_id2device[i_id].get_v() / 360, 2)
            y_point = self.__dict_id2device[i_id].get_y_point()
            self.__dict_id2device[i_id].set_location(x_point, y_point)

        # 车辆不重新撒点
        bs_channel = self.__dict_id2channel[0]
        for channel in self.__dict_id2channel:
            channel = self.__dict_id2channel[channel]
            rx = self.__dict_id2rx[channel.get_rx_id()]
            if rx.get_id() != 0:
                tx = self.__dict_id2tx[rx.get_tx_id()]
                channel.update_link_loss_cell(tx, rx)
                channel.update_link_loss_mmwave(tx, rx, self)
                tx_id = tx.get_id()
                value = channel.get_link_loss_mmwave(tx_id)
                tx.set_v2v_csi_mmwave(value)
                value = channel.get_link_loss(tx_id)
                tx.set_v2v_csi_cell(value)
                value = bs_channel.get_link_loss(tx_id)
                tx.set_tx2bs_csi(value)
            else:
                for id in range(1, 1 + self.__cue_num):
                    tx = self.__dict_id2tx[id]
                    channel.update_link_loss_cell(tx, rx)

    '''
    def update(self, slot):
        # 更新用户位置 更新信道
        # 更新车辆位置
        for i_id in range(1 + self.__cue_num, 1 + self.__cue_num + self.__d2d_num * 2):
            x_point = self.__dict_id2device[i_id].get_x_point() + self.__dict_id2device[i_id].get_direction() * \
                      self.__dict_id2device[i_id].get_v() / 3600
            y_point = self.__dict_id2device[i_id].get_y_point()
            self.__dict_id2device[i_id].set_location(x_point, y_point)

        # 车辆不重新撒点
        for channel in self.__dict_id2channel:
            channel = self.__dict_id2channel[channel]
            rx = self.__dict_id2rx[channel.get_rx_id()]
            if rx.get_id() != 0:
                tx = self.__dict_id2tx[rx.get_tx_id()]
                channel.update_link_loss_cell(tx, rx)
                channel.update_link_loss_mmwave(tx, rx, self)
            else:
                for id in range(1, 1 + self.__cue_num):
                    tx = self.__dict_id2tx[id]
                    channel.update_link_loss_cell(tx, rx)
        # 到达撒点周期， 重新撒点
        if (slot + 1) % 1== 0:
            for tx in self.__dict_id2tx:
                tx = self.__dict_id2tx[tx]
                tx.initial_user_location(self.__highway)
            for rx in self.__dict_id2rx:
                rx = self.__dict_id2rx[rx]
                if rx.get_id() != 0:
                    rx.initial_user_location(self.__highway, self.__dict_id2tx[rx.get_tx_id()])
            for channel in self.__dict_id2channel:
                channel = self.__dict_id2channel[channel]
                rx = self.__dict_id2rx[channel.get_rx_id()]
                if rx.get_id() != 0:
                    tx = self.__dict_id2tx[rx.get_tx_id()]
                    channel.update_link_loss_cell(tx, rx)
                    channel.update_link_loss_mmwave(tx, rx, self)
                else:
                    for id in range(1, 1 + self.__cue_num):
                        tx = self.__dict_id2tx[id]
                        channel.update_link_loss_cell(tx, rx)
    '''

    def agent_obverse(self, tx):  # agent 观测环境
        return self.__dict_id2channel_mmwave[tx.get_rx_id()].get_link_loss_mmwave(tx.get_id()), self.__dict_id2channel[tx.get_rx_id()].get_link_loss(tx.get_id())


    def get_neighbors(self, rx, num):
        neighbors = []
        tx_id2distance = {}
        for tx_id in self.__dict_id2tx:
            if rx.get_tx_id() != tx_id:
                temp_tx = self.__dict_id2tx[tx_id]
                distance = get_distance(temp_tx.get_x_point(), temp_tx.get_y_point(),
                                        rx.get_x_point(), rx.get_y_point())
                tx_id2distance[tx_id] = distance
        list_tx_id2distance = sorted(tx_id2distance.items(), key=lambda item: item[1])
        for i in range(num):
            neighbors.append(list_tx_id2distance[i][0])
        return neighbors

    def update_neighbor_rb(self, temp_tx):
        rx_id = temp_tx.get_rx_id()
        temp_rx = self.__dict_id2rx[rx_id]
        neighbors = self.get_neighbors(temp_rx, 3)
        temp_tx.previous_neighbor_1_rb = self.__dict_id2tx[neighbors[0]].get_allocated_rb()[0]
        temp_tx.previous_neighbor_2_rb = self.__dict_id2tx[neighbors[1]].get_allocated_rb()[0]
        temp_tx.previous_neighbor_3_rb = self.__dict_id2tx[neighbors[2]].get_allocated_rb()[0]

    def save_data_reward(self, name):
        save_name = './result/' + name + 'total' + '.txt'
        with open(save_name, 'w') as f:
            for total_reward in self.__list_total_reward:
                f.write(str(total_reward))
                f.write('\n')
        save_name_2 = './result/' + name + 'single' + '.txt'
        with open(save_name_2, 'w') as f:
            for single_reward in self.__list_single_reward:
                f.write(str(single_reward))
                f.write('\n')

    def save_data_reinforcement(self, name):
        save_name_1 = './result/' + name + '_d2d_rate' + '.txt'
        with open(save_name_1, 'w') as f:
            for sinr in self.__list_d2d_rate_reinforcement:
                f.write(str(sinr))
                f.write('\n')
        save_name_2 = './result/' + name + '_d2d_sinr' + '.txt'
        with open(save_name_2, 'w') as f:
            for sinr in self.__list_d2d_sinr_reinforcement:
                f.write(str(sinr))
                f.write('\n')
        save_name_3 = './result/' + name + '_cue_sinr' + '.txt'
        with open(save_name_3, 'w') as f:
            for sinr in self.__list_cue_sinr_reinforcement:
                f.write(str(sinr))
                f.write('\n')

    def save_data(self):
        with open('./result/cue_sinr_graph.txt', 'w') as f:
            for sinr in self.__list_cue_sinr_random:
                f.write(str(sinr))
                f.write('\n')

        '''with open('./result/cue_sinr_rl.txt', 'w') as f:
            for sinr in self.__list_cue_sinr_rl:
                f.write(str(sinr))
                f.write('\n')'''

        with open('./result/d2d_sinr_graph.txt', 'w') as f:
            for sinr in self.__list_d2d_sinr_random:
                f.write(str(sinr))
                f.write('\n')

        with open('./result/d2d_rate_graph.txt', 'w') as f:
            for sinr in self.__list_d2d_rate_random:
                f.write(str(sinr))
                f.write('\n')

        '''with open('./result/d2d_sinr_rl.txt', 'w') as f:
            for sinr in self.__list_d2d_sinr_rl:
                f.write(str(sinr))
                f.write('\n')'''

    def save_data_full_YL(self):
        with open('./result/cue_sinr_YL.txt', 'w') as f:
            for sinr in self.__list_cue_sinr_random:
                f.write(str(sinr))
                f.write('\n')


        with open('./result/d2d_sinr_YL.txt', 'w') as f:
            for sinr in self.__list_d2d_sinr_random:
                f.write(str(sinr))
                f.write('\n')

        with open('./result/d2d_rate_YL.txt', 'w') as f:
            for sinr in self.__list_d2d_rate_random:
                f.write(str(sinr))
                f.write('\n')

    def save_data_full_mmwave(self):
        with open('./result/cue_sinr_mmwave.txt', 'w') as f:
            for sinr in self.__list_cue_sinr_random:
                f.write(str(sinr))
                f.write('\n')


        with open('./result/d2d_sinr_mmwave.txt', 'w') as f:
            for sinr in self.__list_d2d_sinr_random:
                f.write(str(sinr))
                f.write('\n')

        with open('./result/d2d_rate_mmwave.txt', 'w') as f:
            for sinr in self.__list_d2d_rate_random:
                f.write(str(sinr))
                f.write('\n')

    def save_data_game(self):
        with open('./result/cue_sinr_game.txt', 'w') as f:
            for sinr in self.__list_cue_sinr_random:
                f.write(str(sinr))
                f.write('\n')


        with open('./result/d2d_sinr_game.txt', 'w') as f:
            for sinr in self.__list_d2d_sinr_random:
                f.write(str(sinr))
                f.write('\n')

        with open('./result/d2d_rate_game.txt', 'w') as f:
            for sinr in self.__list_d2d_rate_random:
                f.write(str(sinr))
                f.write('\n')

    def save_data_fair(self):
        with open('./result/cue_sinr_fair.txt', 'w') as f:
            for sinr in self.__list_cue_sinr_fair:
                f.write(str(sinr))
                f.write('\n')

        with open('./result/d2d_sinr_fair.txt', 'w') as f:
            for sinr in self.__list_d2d_sinr_fair:
                f.write(str(sinr))
                f.write('\n')

        with open('./result/d2d_rate_fair.txt', 'w') as f:
            for sinr in self.__list_d2d_rate_fair:
                f.write(str(sinr))
                f.write('\n')

    def clear_allocated_rb(self):
        for tx_id in self.__dict_id2tx:
            temp_tx = self.__dict_id2tx[tx_id]
            if temp_tx.get_type() == 'D2DTx':
                rx_id = temp_tx.get_rx_id()
                temp_rx = self.__dict_id2rx[rx_id]
                temp_tx.clear_allocated_rb()
                temp_rx.clear_allocated_rb()

