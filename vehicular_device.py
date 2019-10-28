from vehicular_channel import *
from buffer_data_packet import *
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import time

# from tensorboardX import SummaryWriter

time = time.time()
# writer = SummaryWriter('log/Ye_Li/log_' + str(time).split('.')[0])
# writer = SummaryWriter('log')

# 超参数
BATCH_SIZE = 1000
LR = 0.01  # learning rate
EPSILON = 0.9  # 最优选择动作百分比
GAMMA = 0.9  # 奖励递减参数
TARGET_REPLACE_ITER = 100  # Q 现实网络的更新频率
MEMORY_CAPACITY = 20000  # 记忆库大小
random.seed()  # 随机数种子


# 所有设备的公共接口类
class Interface(object):
    def __init__(self, i_id, i_type):
        self.__id = i_id  # 设备序号
        self.__type = i_type  # 设备类型

    def get_id(self):  # 获得设备号
        return self.__id

    def get_type(self):  # 获得用户类型
        return self.__type


# 基站类
class BS(Interface):
    def __init__(self, i_id, i_type):
        # 调用父类的构造函数
        Interface.__init__(self, i_id, i_type)
        self.__power = 20  # 基站发射功率 dBm
        self.__x_point = 250  # 基站横坐标
        self.__y_point = -5  # 基站纵坐标
        self.__allocated_rb = []  # 基站所使用的RB登记表
        self.__txs_id = []  # 上行链路  蜂窝链路发射机登记表
        self.__rxs_id = []  # 下行链路  蜂窝链路接收机登记表
        self.__tx_id2sinr = {}  # 基站接受的发射机的SINR登记表 key：tx_id  value：sinr
        self.__direction = 0  # 基站的行驶方向为0

    def get_direction(self):
        return self.__direction

    def update_location(self):
        pass

    def set_location(self, x_point, y_point):   # 设置基站位置
        self.__x_point = x_point
        self.__y_point = y_point

    # 获得基站位置
    def get_x_point(self):
        return self.__x_point

    def get_y_point(self):
        return self.__y_point

    # 获得基站发射功率
    def get_power(self):
        return self.__power

    # ？？？
    def set_allocated_rb(self, rb_id):
        # self.__allocated_rb = []
        self.__allocated_rb.append(rb_id)

    def get_allocated_rb(self):
        return self.__allocated_rb

    def set_rx(self, rx_id):
        self.__rxs_id.append(rx_id)

    def set_tx(self, tx_id):
        self.__txs_id.append(tx_id)

    def get_tx_id(self):
        return self.__txs_id

    def comp_sinr(self, dict_id2tx, dict_id2channel):  # 计算接收 SINR
        if len(self.get_allocated_rb()):
            # 计算噪声功率  1个RB, 12个连续的载波, 12 * 15000 = 180000Hz
            white_noise = -134  # -174dBm / Hz
            noise_fig = 5  # dB
            noise_fig = pow(10, noise_fig / 10)  # 线性值
            thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

            a_gain = 0  # dBi
            a_gain = pow(10, a_gain / 10)

            for ID in self.__txs_id:
                # 计算接收目标信号功率
                target_tx = dict_id2tx[ID]  # 目标发射机
                target_power = target_tx.get_power()  # dBm
                target_power = pow(10, (target_power - 30) / 10)  # W
                target_channel = dict_id2channel[self.get_id()]
                target_link_loss = target_channel.get_link_loss(ID)  # dB
                target_gain = pow(10, -target_link_loss / 10) # * math.sqrt(pow(random.gauss(0, 1), 2))
                receive_target_power = target_power * target_gain * a_gain

                # 计算接收干扰信号总功率
                receive_inter_power = 0
                for tx_id in dict_id2tx:
                    if tx_id != ID:
                        if self.get_allocated_rb()[ID - 1] in dict_id2tx[tx_id].get_allocated_rb():
                            inter_tx = dict_id2tx[tx_id]  # 干扰发射机
                            inter_power = inter_tx.get_power()  # dBm
                            inter_power = pow(10, (inter_power - 30) / 10)  # W
                            inter_channel = dict_id2channel[self.get_id()]
                            inter_link_loss = inter_channel.get_link_loss(tx_id)  # dB
                            inter_gain = pow(10, -inter_link_loss / 10) # * math.sqrt(pow(random.gauss(0, 1), 2))
                            receive_inter_power += inter_power * inter_gain * a_gain

                sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
                self.__tx_id2sinr[ID] = sinr

            return None

    def get_sinr(self):
        return self.__tx_id2sinr


# 用户类 CUE and V2V user common interface ; create user's RB ,location, drive direction
class User(Interface):
    def __init__(self, i_id, i_type, v=30):
        # 调用父类的构造函数
        Interface.__init__(self, i_id, i_type)

        self.__x_point = -1
        self.__y_point = -1
        self.__allocated_rb = []
        self.__v = v + (random.random() - 0.5) * 20

        # 随机生成车辆行进方向
        rand_num = random.random()
        if rand_num > 0.5:
            self.__direction = 1  # 正方向
        else:
            self.__direction = -1  # 负方向

    # 用户位置
    def set_location(self, x_point, y_point):
        self.__x_point = x_point
        self.__y_point = y_point

    def get_direction(self):
        return self.__direction

    def get_x_point(self):
        return self.__x_point

    def get_y_point(self):
        return self.__y_point

    def get_v(self):
        return self.__v

    def set_v(self, v):
        self.__v = v

    # user's RB
    def set_allocated_rb(self, rb_id):
        self.__allocated_rb = []
        self.__allocated_rb.append(rb_id)

    def get_allocated_rb(self):
        return self.__allocated_rb

    def clear_allocated_rb(self):
        self.__allocated_rb = []

    '''def update_location(self):
        self.__x_point += random.normalvariate(0, 1)
        self.__y_point += random.normalvariate(0, 1)'''

    def initial_user_location(self, highway):
        self.__x_point = highway.get_start_x_point() + round(random.random() * highway.get_length(), 2)
        chedao_index = random.randint(0, 1)
        if self.__direction == 1:
            self.__y_point = highway.get_start_y_point() + highway.get_width() / 8 + chedao_index * highway.get_width() / 4
        else:
            self.__y_point = highway.get_start_y_point() + highway.get_width() / 8 + chedao_index * highway.get_width() / 4 + highway.get_width() / 2


# V2I用户类 使用User类创建V2I用户
class CUE(User):
    def __init__(self, i_id, i_type, power=23):
        User.__init__(self, i_id, i_type)
        self.__power = power
        self.__tx_id = 0
        self.set_v(0)

    def initial_cue_v(self):
        self.set_v(0)

    def initial_user_location(self, highway):
        value = random.gauss(0, 1)
        if value > 0:
            x_point = random.randint(round(highway.get_length()/8), round(highway.get_length() * 3 / 8))
        else:
            x_point = random.randint(round(highway.get_length() * 5 / 8), round(highway.get_length() * 7 / 8))
        y_point = -4
        self.set_location(x_point, y_point)

    def update_location_after_spectrum_allocation(self, time):
        # 频谱分配结束后的车辆位置
        x_point = self.get_x_point() + self.get_direction() * self.get_v() * time
        self.set_location(x_point, self.get_y_point())

    def get_power(self):
        return self.__power

    def set_power(self, power):
        self.__power = power


# D2D发射机类,use class User
class D2DTx(User):  # 智能体
    def __init__(self, i_id, i_type, power=17):
        User.__init__(self, i_id, i_type, 30)
        # user User's initial function create D2D transmitter's location ,RB and interface‘s id and type
        self.__rx_id = -1
        self.__power = power
        self.__blockers = 0
        self.__previous_rb = -1  # 智能体上一个时隙使用的信道  大小为 1
        self.__neighbor_V2V = []

        # 观测到的环境
        self.__v2v_csi_mmwave = -1  # 毫米波瞬时CSI 1
        self.__v2v_csi_cell = -1  # 蜂窝V2V瞬时CSI 1
        # self.bs2rx_csi = -1  # 蜂窝用户的CSI 1
        self.__tx2bs_csi = -1  # V2V 发射机对于bs的干扰 1
        self.__p0 = 0.01

        self.__previous_inter = []  # 智能体感知到的每一个信道上的干扰  大小为 1*(m+1)
        self.previous_neibor_rb = []  # 只能提邻居用户使用的信道  1*(m+1)

        self.__delay_requirement = 0.001  # V2V链路的延迟约束参数 1 服务1：10ms 服务2：100ms
        self.__arrival_rate = 64000  # V2V链路的到达参数 1 服务1：0.01*6400*1000 bits 服务2：0.01*6400*1000

        self.__cue_sinr = -1

        self.__observation = []  # 智能体观测的环境 5+2*(m+1) = 8 + 2m
        # self.__bs2v2v_csi_cell = -1  # 蜂窝到V2V的干扰CSI

    def set_Qos(self, cue_num, d2d_num):
        if self.get_id() in range(1 + cue_num, 1 + cue_num + d2d_num / 2):
            self.__delay_requirement = 0.01
            # self.__arrival_rate = 0.01 * 6400 * 1000
        else:
            # self.__arrival_rate = 0.1 * 6400 * 1000
            self.__delay_requirement = 0.1

    def set_cue_sinr(self, cue_sinr):
        self.__cue_sinr = cue_sinr

    def set_previous_rb(self, rb_id):
        self.__previous_rb = rb_id

    def set_v2v_csi_mmwave(self, value):
        self.__v2v_csi_mmwave = value

    def set_v2v_csi_cell(self, value):
        self.__v2v_csi_cell = value

    def set_tx2bs_csi(self, value):
        self.__tx2bs_csi = value

    def get_delay_requirement(self):
        return self.__delay_requirement

    def get_arrival_rate(self):
        return self.__arrival_rate

    def get_outage_probability(self):
        return self.__p0

    def add_prevousr_inter_neibour(self, power):
        self.__previous_inter.append(power)

    def get_previous_inter(self):
        return self.__previous_inter

    def clear_neibour_rb(self):
        self.previous_neibor_rb = []

    def add_neibour_rb(self, rb_id):
        self.previous_neibor_rb.append(rb_id)

    def get_neibor_rb(self):
        return self.previous_neibor_rb

    def add_neibour_V2V(self, neibour_tx_id):
        self.__neighbor_V2V.append(neibour_tx_id)

    def get_neibor_V2V(self):
        return self.__neighbor_V2V

    def clear_previous_inter(self):
        self.__previous_inter = []

    def clear_previous_neibour_rb(self):
        self.__neighbor_V2V = []

    def set_power(self, power):
        self.__power = power

    def update_location_after_spectrum_allocation(self, time):
        # 频谱分配结束后的车辆位置
        x_point = self.get_x_point() + self.get_direction() * self.get_v() * time
        self.set_location(x_point, self.get_y_point())

    # 配对
    def make_pair(self, rx_id):
        self.__rx_id = rx_id

    def get_power(self):
        return self.__power

    def get_rx_id(self):
        return self.__rx_id

    def get_blockers(self):
        return self.__blockers

    def set_blockers(self, blockers):
        self.__blockers = blockers

    def tx_a_gain_mmwave(self, x_point, y_point, dict_id2rx):  # x_point 为当前接受机位置
        rx_x_point = dict_id2rx[self.__rx_id].get_x_point()
        rx_y_point = dict_id2rx[self.__rx_id].get_y_point()
        tx_x_point = self.get_x_point()
        tx_y_point = self.get_y_point()
        gain_max = 10 * math.log10(pow(1.6162/(math.sin(math.pi / 12)), 2))
        gain_sl = -0.4111 * math.log(math.pi/6, math.e) - 10.579
        angle_ml = 2.6 * math.pi / 6
        # print("l1" + str((rx_y_point - tx_y_point) / (rx_x_point - tx_x_point)))
        # l1=math.asin((rx_y_point - tx_y_point)/(rx_x_point - tx_x_point))
        # l2=math.asin((y_point - tx_y_point)/(x_point - tx_x_point))
        # print("l1"+str((rx_y_point - tx_y_point)/(rx_x_point - tx_x_point)))
        # print(l2)

        # angle = math.atan((rx_y_point - tx_y_point)/(rx_x_point - tx_x_point)) - math.atan((y_point - tx_y_point) /
                                                                                          # (x_point - tx_x_point))
        x = np.array([int(rx_x_point - tx_x_point), int(rx_y_point - tx_y_point)])
        y = np.array([int(x_point - tx_x_point), int(y_point - tx_y_point)])
        cos_value = (float(x.dot(y)) / (np.sqrt(x.dot(x)) * np.sqrt(y.dot(y))))
        cos_value = round(cos_value, 8)
        angle = np.arccos(cos_value)
        # print("tx")
        # print(angle)
        angle = abs(angle)
        if angle <= angle_ml / 2:
            tx_a_gain = gain_max - 3.01 * pow(2 * angle / (math.pi / 6), 2)
            # print("tx  gain_max\t" + str(gain_max) + "\t" + str(3.01 * pow(2 * angle / (math.pi / 6), 2)))
        else:
            tx_a_gain = gain_sl
        return tx_a_gain

    def update_arrival_rate_and_delay(self, slot):  # 更新车辆进行的服务
        probability = random.random()
        if probability > 0:
            self.__delay_requirement = 0.01
            self.__arrival_rate = 0.01*6400*1000
        else:
            self.__arrival_rate = 0.1 * 6400 * 1000
            self.__delay_requirement = 0.1


    def obverse_environment(self, rb_num, single_cell):
        # 维度 2 * rb_num + 8
        self.__observation = []

        # 状态空间观测到的上一时隙使用rb
        self.__observation.append(self.__previous_rb)
        # for i in range(rb_num):
            # if i == self.__previous_rb:
                # self.__observation.append(1)
            # else:
                # self.__observation.append(0)

        # 状态空间种观测到的上一时隙的干扰
        for inter in self.__previous_inter:
            if inter == 0:
                self.__observation.append(-1)
            else:
                self.__observation.append(round(-math.log10(inter) / 10, 1))

        # 状态空间上一时隙邻居用户使用的rb
        for nei_rb in range(rb_num):
            if nei_rb in self.previous_neibor_rb:
                self.__observation.append(1)
            else:
                self.__observation.append(0)

        # 观测到的毫米波和蜂窝频段csi
        self.__observation.append(round(self.__v2v_csi_mmwave/10, 1))  # CSI of D2D link
        self.__observation.append(round(self.__v2v_csi_cell/10, 1))  # CSI of mmWave

        # 观测到的V2V对bs的csi
        # self.__observation.append(self.__tx2bs_csi/100)  # CSI of cellular link

        # 观测到的节点QoS
        self.__observation.append(self.__delay_requirement)
        self.__observation.append(self.__arrival_rate)

        # 观测到对cue信道情况
        self.__observation.append(self.__cue_sinr)


    def get_state(self, rb_num, single_cell):
            self.obverse_environment(rb_num, single_cell)
            state = self.__observation
            state = np.array(state)  # numpy array
            return state

    def do_action(self, action, dict_id2rx, rb_num):
            # rb_id = int(action / 3)
            rb_id = int(action)

            self.set_allocated_rb(rb_id)
            rx = dict_id2rx[self.__rx_id]
            rx.set_allocated_rb(rb_id)

            # power = 3 + (action % 3) * 10  # 3,13,23 dBm
            # self.__power = power
            # self.__power = 13

            print('D2DTx ' + str(self.get_id())
                  + ' choose RB: ' + str(rb_id)
                  + ' power: ' + str(self.__power))


# D2D接收机类,use class User
class D2DRx(User):
    def __init__(self, i_id, i_type):
        User.__init__(self, i_id, i_type, 30)
        self.__tx_id = -1
        self.__sinr = 0

    def rx_a_gain_mmwave(self, x_point, y_point, dict_id2tx):
        tx_x_point = dict_id2tx[self.__tx_id].get_x_point()
        tx_y_point = dict_id2tx[self.__tx_id].get_y_point()
        rx_x_point = self.get_x_point()
        rx_y_point = self.get_y_point()
        gain_max = 10 * math.log10(pow(1.6162/(math.sin(math.pi / 12)), 2))
        gain_sl = -0.4111 * math.log(math.pi/6, math.e) - 10.579
        angle_ml = 2.6 * math.pi / 6
        # print(str(angle_ml) + "angle_ml")
        # angle = math.tan((tx_y_point - rx_y_point)/(tx_x_point - rx_x_point)) - math.atan((y_point - rx_y_point) / (
                # x_point - rx_x_point))
        x = np.array([int(tx_x_point - rx_x_point), int(tx_y_point - rx_y_point)])
        y = np.array([int(x_point - rx_x_point), int(y_point - rx_y_point)])
        cos_value = (float(x.dot(y)) / (np.sqrt(x.dot(x))*np.sqrt(y.dot(y))))
        cos_value = round(cos_value, 8)
        angle = np.arccos(cos_value)
        # print("rx")
        # print(angle)
        angle = abs(angle)
        if angle <= angle_ml / 2:
            rx_a_gain = gain_max - 3.01 * pow(2 * angle / (math.pi / 6), 2)
            # print("rx  gain_max\t"+ str(gain_max)+ "\t"+ str(3.01 * pow(2 * angle / (math.pi / 6), 2)))
        else:
            rx_a_gain = gain_sl
        return rx_a_gain

    # 初始化v2v接受车辆的位置，与发射车辆举例小于10m
    def initial_user_location(self, highway, v2v_tx_vehicle):
        temp_x = random.randint(15, 30)
        pro = random.gauss(0, 1)
        if pro > 0.5:
            x_point = v2v_tx_vehicle.get_x_point() + temp_x
        else:
            x_point = v2v_tx_vehicle.get_x_point() - temp_x

        chedao_index = random.randint(0, 1)
        if self.get_direction() == 1:
            y_point = highway.get_start_y_point() + highway.get_width() / 8 + chedao_index * highway.get_width() / 4
        else:
            y_point = highway.get_start_y_point() + highway.get_width() / 8 + chedao_index * highway.get_width() / 4 + highway.get_width() / 2
        self.set_location(x_point, y_point)

    # 更新车辆位置，车辆仅直线行驶
    def update_location_after_spectrum_allocation(self, time):
        # 频谱分配结束后的车辆位置
        x_point = self.get_x_point() + self.get_direction() * self.get_v() * time
        self.set_location(x_point, self.get_y_point())

    def update_location(self, highway, v2v_tx_vehicle):
        temp_x = (random.random() - 0.5) * 10  # 设 V2V 之间最大间距500m
        temp_y = (random.random() - 0.5) * 10
        if highway.get_end_x_point() - highway.get_start_x_point():
            x_point = v2v_tx_vehicle.get_x_point() + temp_x
            y_point = v2v_tx_vehicle.get_y_point()
        else:
            x_point = v2v_tx_vehicle.get_x_point()
            y_point = v2v_tx_vehicle.get_y_point() + temp_y
        User.set_location(self, x_point, y_point)

        self.__tx_id = v2v_tx_vehicle.get_id()
        # v2v_tx_vehicle.set_rx_id(self.get_id())

    # 配对
    def make_pair(self, tx_id):
        self.__tx_id = tx_id

    def get_tx_id(self):
        return self.__tx_id

    def comp_sinr(self, dict_id2tx, dict_id2channel):  # 计算接收 SINR,return the sum power of interference signal
        if len(self.get_allocated_rb()):
            # 计算噪声功率  1个RB, 12个连续的载波, 12 * 15000 = 180000Hz
            white_noise = -134  # -174dBm / Hz
            noise_fig = 5  # dB
            noise_fig = pow(10, noise_fig / 10)  # 线性值
            thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

            # 计算接收目标信号功率
            target_tx = dict_id2tx[self.__tx_id]  # 目标发射机
            target_power = target_tx.get_power()  # dBm
            target_power = pow(10, (target_power - 30) / 10)  # W
            target_channel = dict_id2channel[self.get_id()]
            target_link_loss = target_channel.get_link_loss(self.__tx_id)  # dB
            target_gain = pow(10, -target_link_loss / 10) # * math.sqrt(pow(random.gauss(0, 1), 2))

            receive_target_power = target_power * target_gain

            # 计算接收干扰信号总功率
            receive_inter_power = 0
            for tx_id in dict_id2tx:
                if tx_id != self.__tx_id:
                    if self.get_allocated_rb()[0] in dict_id2tx[tx_id].get_allocated_rb():
                        inter_tx = dict_id2tx[tx_id]  # 干扰发射机
                        inter_power = inter_tx.get_power()  # dBm
                        inter_power = pow(10, (inter_power - 30) / 10)  # W
                        inter_channel = dict_id2channel[self.get_id()]
                        inter_link_loss = inter_channel.get_link_loss(tx_id)  # dB
                        inter_gain = pow(10, -inter_link_loss / 10) # * math.sqrt(pow(random.gauss(0, 1), 2))
                        receive_inter_power += inter_power * inter_gain

            self.__sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
            return receive_inter_power

    def comp_mmwave_sinr(self, dict_id2tx, dict_id2channel_mmwave, dict_id2rx):
        # 毫米波参数 频率 60GHz EIRP = 20 dBm  带宽：2.16GHz  Noise Figure：6dB  Noise Power : -174 + 10logB + NF dBm
        # 天线模型 sectored model  anternna sidelobe: - 15 dB
        band_width = 2.16 * pow(10, 9)

        if len(self.get_allocated_rb()):
            # 计算噪声功率  1个RB, 12个连续的载波, 12 * 15000 = 180000Hz
            # white_noise = -174  # -174dBm / Hz
            # noise_fig = 6  # dB
            # noise_fig = pow(10, noise_fig / 10)  # 线性值
            # thermal_noise_pow = pow(10, (white_noise - 30) / 10) * band_width   # 线性值  0.5w

            noise_power = -174 + 10 * math.log10(band_width) + 6
            thermal_noise_pow = pow(10, (noise_power - 30) / 10)

            # 计算接收目标信号功率
            target_tx = dict_id2tx[self.__tx_id]  # 目标发射机
            target_power = target_tx.get_power()  # dBm
            target_power = pow(10, (target_power - 30) / 10)  # W
            target_channel = dict_id2channel_mmwave[self.get_id()]
            target_link_loss = target_channel.get_link_loss_mmwave(self.__tx_id)  # dB
            target_gain = pow(10, -target_link_loss / 10)

            # 发射机为配对的发射机，接收机为自己
            tx_a_gain = target_tx.tx_a_gain_mmwave(dict_id2rx[target_tx.get_rx_id()].get_x_point(), dict_id2rx[target_tx
                                                   .get_rx_id()].get_y_point(), dict_id2rx)
            tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
            rx_a_gain = self.rx_a_gain_mmwave(target_tx.get_x_point(), target_tx.get_y_point(), dict_id2tx)
            rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
            receive_target_power = target_power * target_gain * tx_a_gain * rx_a_gain

            # 计算接收干扰信号总功率
            receive_inter_power = 0
            for tx_id in dict_id2tx:
                if tx_id != self.__tx_id:
                    if self.get_allocated_rb()[0] in dict_id2tx[tx_id].get_allocated_rb():
                        inter_tx = dict_id2tx[tx_id]  # 干扰发射机
                        inter_power = inter_tx.get_power()  # dBm
                        inter_power = pow(10, (inter_power - 30) / 10)  # W
                        inter_channel = dict_id2channel_mmwave[self.get_id()]
                        inter_link_loss = inter_channel.get_link_loss_mmwave(tx_id)  # dB
                        inter_gain = pow(10, -inter_link_loss / 10)

                        # 接收机为自己本身，发射机为干扰发射机
                        tx_a_gain = inter_tx.tx_a_gain_mmwave(self.get_x_point(), self.get_y_point(), dict_id2rx)
                        tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
                        rx_a_gain = self.rx_a_gain_mmwave(inter_tx.get_x_point(), inter_tx.get_y_point(), dict_id2tx)
                        rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
                        receive_inter_power += inter_power * inter_gain * tx_a_gain * rx_a_gain

            self.__sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
            return receive_inter_power


    def get_sinr(self):
        return self.__sinr


# highway class
class Highway(object):
    def __init__(self, start_x_point, start_y_point, length, width):
        self.__start_x_point = start_x_point
        self.__start_y_point = start_y_point
        self.__end_x_point = start_x_point+length
        self.__end_y_point = start_y_point+width
        self.__length = length
        self.__width = width

    def get_start_x_point(self):
        return self.__start_x_point

    def get_start_y_point(self):
        return self.__start_y_point

    def get_end_x_point(self):
        return self.__end_x_point

    def get_end_y_point(self):
        return self.__end_y_point

    def get_length(self):
        return self.__length

    def get_width(self):
        return self.__width


class Net(nn.Module):
    def __init__(self, n_states, n_actions):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(n_states, 128)
        self.fc1.weight.data.normal_(0, 0.1)  # initialization
        self.fc2 = nn.Linear(128, 64)
        self.fc2.weight.data.normal_(0, 0.1)  # initialization
        self.fc3 = nn.Linear(64, 32)
        self.fc3.weight.data.normal_(0, 0.1)  # initialization
        self.out = nn.Linear(32, n_actions)
        self.out.weight.data.normal_(0, 0.1)  # initialization

        # self.fc1 = nn.Linear(n_states, 512)
        # self.fc1.weight.data.normal_(0, 0.1)   # initialization
        # self.fc2 = nn.Linear(512, 256)
        # self.fc2.weight.data.normal_(0, 0.1)  # initialization
        # self.fc3 = nn.Linear(256, 128)
        # self.fc3.weight.data.normal_(0, 0.1)  # initialization
        # self.out = nn.Linear(128, n_actions)
        # self.out.weight.data.normal_(0, 0.1)   # initialization

    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        x = F.relu(x)
        x = self.fc3(x)
        x = F.relu(x)
        actions_value = self.out(x)
        return actions_value


class Ye_Li_DQN(object):
    def __init__(self, n_states, n_actions):
        self.n_states = n_states
        self.n_actions = n_actions

        # 建立 target net 和 eval net 还有 memory
        self.eval_net, self.target_net = Net(n_states, n_actions), Net(n_states, n_actions)

        self.learn_step_counter = 0  # for target updating
        self.memory_counter = 0  # for storing memory
        self.memory = np.zeros((MEMORY_CAPACITY, n_states * 2 + 2))  # initialize memory
        # n_states * 2 + 2 元组中存储动作选择前后state， 以及动作， reward
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=LR)
        # torch 的优化器
        self.loss_func = nn.MSELoss()
        # 误差公式

    def choose_action(self, x):
        # 根据环境观测值x选择动作的机制
        x = torch.unsqueeze(torch.FloatTensor(x), 0)
        # input only one sample
        if np.random.uniform() < EPSILON:  # greedy
            # 选最优动作
            actions_value = self.eval_net.forward(x)
            action = torch.max(actions_value, 1)[1].data.numpy()
            action = action[0]
        else:  # random
            # 选随机动作
            action = np.random.randint(0, self.n_actions)
        return action

    def choose_random_action(self):
        action = np.random.randint(0, self.n_actions)
        return action

    def choose_action_test(self, x):
        # 根据环境观测值x选择动作的机制
        x = torch.unsqueeze(torch.FloatTensor(x), 0)
        # input only one sample
        actions_value = self.target_net.forward(x)
        action = torch.max(actions_value, 1)[1].data.numpy()
        action = action[0]
        return action

    # 存储记忆
    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, [a, r], s_))
        # replace the old memory with new memory
        index = self.memory_counter % MEMORY_CAPACITY
        self.memory[index, :] = transition
        self.memory_counter += 1

    def learn(self):
        # target parameter update
        # 更新现实网络
        #
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        self.learn_step_counter += 1

        # sample batch transitions
        sample_index = np.random.choice(MEMORY_CAPACITY, BATCH_SIZE)  # 从MEMORY_CAPACITY中随机猜batch-size数量
        b_memory = self.memory[sample_index, :]
        b_s = torch.FloatTensor(b_memory[:, :self.n_states])  # 之前得状态
        b_a = torch.LongTensor(b_memory[:, self.n_states:self.n_states + 1].astype(int))  # 动作
        b_r = torch.FloatTensor(b_memory[:, self.n_states + 1:self.n_states + 2])  # reward
        b_s_ = torch.FloatTensor(b_memory[:, -self.n_states:])  # 转移后得状态

        # q_eval w.r.t the action in experience
        q_eval = self.eval_net(b_s).gather(1, b_a)  # shape (batch, 1)
        q_next = self.target_net(b_s_).detach()  # detach from graph, don't backpropagate
        q_target = b_r + GAMMA * q_next.max(1)[0].view(BATCH_SIZE, 1)  # shape (batch, 1)
        loss = self.loss_func(q_eval, q_target)

        # writer.add_scalar('Train/Loss', loss.data[0], self.learn_step_counter)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def load(self, name):
        self.target_net.load_state_dict(torch.load(name))

    def save(self, name):
        torch.save(self.target_net.state_dict(), name + '.pkl')