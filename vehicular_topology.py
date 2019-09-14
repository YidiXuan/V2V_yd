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


    def update(self):
        # 更新用户位置 更新信道
        for rx_id in self.__dict_id2channel:
            for tx_id in self.__dict_id2tx:  # 遍历所有的发射机
                tx = self.__dict_id2tx[tx_id]
                rx = self.__dict_id2rx[rx_id]
                # tx.update_location()
                # rx.update_location()
                self.__dict_id2channel[rx_id].update_link_loss_cell(tx, rx)

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

