from vehicular_channel import *
from buffer_data_packet import *
import numpy as np
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

    '''def update_location(self):
        self.__x_point += random.normalvariate(0, 1)
        self.__y_point += random.normalvariate(0, 1)'''

    def initial_user_location(self, highway):
        self.__x_point = highway.get_start_x_point() + random.random() * highway.get_length()
        if self.__direction == 1:
            self.__y_point = highway.get_start_y_point() + random.randint(1, highway.get_width() / 2)
        else:
            self.__y_point = highway.get_start_y_point() + random.randint(1 + highway.get_width() / 2, highway.get_width())


# V2I用户类 使用User类创建V2I用户
class CUE(User):
    def __init__(self, i_id, i_type, power=17):
        User.__init__(self, i_id, i_type)
        self.__power = power
        self.__tx_id = 0
        self.set_v(0)

    def initial_cue_v(self):
        self.set_v(0)

    def initial_user_location(self, highway):
        x_point = random.uniform(0, highway.get_length())
        y_point = -2
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
class D2DTx(User):
    def __init__(self, i_id, i_type, power=17):
        User.__init__(self, i_id, i_type, 30)
        # user User's initial function create D2D transmitter's location ,RB and interface‘s id and type
        self.__rx_id = -1
        self.__power = power
        self.__blockers = 0

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
        temp_x = random.uniform(0, 20)
        x_point = v2v_tx_vehicle.get_x_point() + temp_x
        if self.get_direction() == 1:
            y_point = random.randint(1, highway.get_width() / 2)
        else:
            y_point = random.randint(1 + highway.get_width() / 2, highway.get_width())
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
