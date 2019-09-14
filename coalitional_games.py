from vehicular_topology import *
from vehicular_device import *

def resource_coalitional_games(dict_id2tx, dict_id2rx, rb_num, cue_num, d2d_num, dict_id2channel, dict_id2channel_mmwave):
    dict_c_id2coalition = {}

    for c_id in range(rb_num):
        dict_c_id2coalition[c_id] = Coalition(c_id)
    # 为用户分配联盟
    for tx_id in dict_id2tx:
        tx = dict_id2tx[tx_id]
        dict_c_id2coalition[tx.get_allocated_rb()[0]].add_tx(tx)
    print("分配完成")

    # 首先计算每个联盟的和速率
    system_sum_rate = 0  # 系统和速率
    for c_id in dict_c_id2coalition:
        if c_id == rb_num - 1:
            coalition_sum_rate = dict_c_id2coalition[c_id].comp_coalition_sum_rate_mmwave(dict_id2tx, dict_id2rx,
                                                                                          dict_id2channel_mmwave,
                                                                                          cue_num)
            system_sum_rate += coalition_sum_rate
            dict_c_id2coalition[c_id].set_coalition_sum_rate(coalition_sum_rate)
        if c_id != rb_num - 1:
            coalition_sum_rate = dict_c_id2coalition[c_id].comp_coalition_sum_rate(dict_id2tx, dict_id2rx,
                                                                                   dict_id2channel, cue_num)
            system_sum_rate += coalition_sum_rate
            dict_c_id2coalition[c_id].set_coalition_sum_rate(coalition_sum_rate)

    num = 0

    while num < 10 * d2d_num:
        # 开始博弈
        for tx_id in range(1 + cue_num, 1 + cue_num + d2d_num):  # 随机选择一个D2D
            d2d_tx = dict_id2tx[tx_id]
            # 随机选择一个当前d2d不在的联盟
            for c_id in dict_c_id2coalition:
                coalition = dict_c_id2coalition[c_id]
                if tx_id in coalition.get_tx_in_coalition():
                    d2d_c_id = c_id
            c_id = random.randint(0, rb_num - 1)
            while c_id == d2d_c_id:
                c_id = random.randint(0, rb_num - 1)
            coalition_now = dict_c_id2coalition[d2d_c_id]
            coalition_other = dict_c_id2coalition[c_id]

            coalition_now_id = coalition_now.get_c_id()
            coalition_other_id = coalition_other.get_c_id()

            # 获得switch操作前的和速率
            # 获得当前d2d所在联盟的 系统吞吐量
            coalition_now_sum_rate = 0
            coalition_other_sum_rate = 0
            if coalition_now_id != rb_num - 1:
                coalition_now_sum_rate = coalition_now.get_coalition_sum_rate()
            if coalition_now_id == rb_num - 1:  # 当前d2d联盟为毫米波联盟
                coalition_now_sum_rate = coalition_now.get_coalition_sum_rate()
            # 获得另一个随机联盟的系统吞吐量
            if coalition_other_id != rb_num - 1:
                # 另一个随机联盟为蜂窝频段联盟
                coalition_other_sum_rate = coalition_other.get_coalition_sum_rate()
            if coalition_other_id == rb_num - 1:  # 另一个随机联盟为毫米波联盟
                coalition_other_sum_rate = coalition_other.get_coalition_sum_rate()

            coalition_sum_rate_before_switch = coalition_now_sum_rate + coalition_other_sum_rate
            # 计算switch 操作前和吞吐量

            # 计算switch操作后的总吞吐量
            coalition_now.remove_tx(d2d_tx)
            coalition_other.add_tx(d2d_tx)

            # 计算switch操作后，d2d离开当前联盟的系统吞吐量
            coalition_now_sum_rate = 0
            coalition_other_sum_rate = 0
            if coalition_now_id != rb_num - 1:
                # 蜂窝频段联盟
                coalition_now_sum_rate = coalition_now.comp_coalition_sum_rate(dict_id2tx, dict_id2rx, dict_id2channel,
                                                                               cue_num)
            if coalition_now_id == rb_num - 1:  # 毫米波联盟
                coalition_now_sum_rate = coalition_now.comp_coalition_sum_rate(dict_id2tx, dict_id2rx,
                                                                               dict_id2channel_mmwave, cue_num)
            # 计算switch操作后，d2d加入另一个联盟的系统吞吐量
            if coalition_other_id != rb_num - 1:
                # 另一个联盟为蜂窝频段联盟
                coalition_other_sum_rate = coalition_other.comp_coalition_sum_rate(dict_id2tx, dict_id2rx,
                                                                                   dict_id2channel, cue_num)
            if coalition_other_id == rb_num - 1:  # 另一个联盟为毫米波联盟
                coalition_other_sum_rate = coalition_other.comp_coalition_sum_rate_mmwave(dict_id2tx, dict_id2rx,
                                                                                          dict_id2channel_mmwave,
                                                                                          cue_num)

            coalition_sum_rate_after_switch = coalition_now_sum_rate + coalition_other_sum_rate
            # switch 操作前和吞吐量

            if coalition_sum_rate_after_switch <= coalition_sum_rate_before_switch:  # 若switch操作无益，恢复原有联盟
                num = num + 1
                coalition_now.add_tx(d2d_tx)
                coalition_other.remove_tx(d2d_tx)
            else:  # 若操作有益，改变两个联盟的和吞吐量
                num = 0
                coalition_now.set_coalition_sum_rate(coalition_now_sum_rate)
                coalition_other.set_coalition_sum_rate(coalition_other_sum_rate)

        system_sum_rate_now = 0
        for c_id in dict_c_id2coalition:
            system_sum_rate_now += dict_c_id2coalition[c_id].get_coalition_sum_rate()
        print(num)
        system_sum_rate = system_sum_rate_now


    print(system_sum_rate_now)
    print("第一次博弈完成")







class Coalition(object):
    def __init__(self, id):
        self.__coalition_id = id
        self.__dict_id2tx_coalition = {}  # 存储联盟内的设备
        self.__num = -1  # 联盟内设备的
        self.__coalition_sum_rate = 0

    def set_coalition_sum_rate(self, coalition_sum_rate):
        self.__coalition_sum_rate = coalition_sum_rate

    def get_coalition_sum_rate(self):
        return self.__coalition_sum_rate

    def comp_coalition_sum_rate(self, dict_id2tx, dict_id2rx,dict_id2channel, cue_num):
        coalition_sum_rate = 0
        for tx_c_id in self.get_tx_in_coalition():
            tx_c = dict_id2tx[tx_c_id]
            if tx_c_id in range(1, 1 + cue_num):
                rx_c_id = 0
                rx_c = dict_id2rx[0]

                # 计算蜂窝用户噪声功率
                white_noise = -134  # -174dBm / Hz
                noise_fig = 5  # dB
                noise_fig = pow(10, noise_fig / 10)  # 线性值
                thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

            else:
                # 计算D2D用户干扰功率
                rx_c_id = dict_id2tx[tx_c_id].get_rx_id()
                rx_c = dict_id2rx[rx_c_id]

                # 计算d2d用户的噪声功率
                white_noise = -134  # -174dBm / Hz
                noise_fig = 5  # dB
                noise_fig = pow(10, noise_fig / 10)  # 线性值
                thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

            target_tx = tx_c
            target_power = target_tx.get_power()  # dBm
            target_power = pow(10, (target_power - 30) / 10)  # W
            target_channel = dict_id2channel[rx_c_id]
            target_link_loss = target_channel.get_link_loss(tx_c_id)  # dB
            target_gain = pow(10, -target_link_loss / 10)
            receive_target_power = target_power * target_gain

            receive_inter_power = 0
            for inter_tx_id in self.get_tx_in_coalition():
                if inter_tx_id != tx_c_id:
                    inter_tx = dict_id2tx[inter_tx_id]  # 干扰发射机
                    inter_power = inter_tx.get_power()  # dBm
                    inter_power = pow(10, (inter_power - 30) / 10)  # W
                    inter_channel = dict_id2channel[rx_c_id]
                    inter_link_loss = inter_channel.get_link_loss(inter_tx_id)  # dB
                    inter_gain = pow(10, -inter_link_loss / 10)
                    receive_inter_power += inter_power * inter_gain
            sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
            if sinr < 5:
                coalition_sum_rate += 0
            else:
                coalition_sum_rate += 180 * math.log10(1 + pow(10, sinr/ 10))
        return coalition_sum_rate

    def comp_coalition_sum_rate_mmwave(self, dict_id2tx, dict_id2rx, dict_id2channel_mmwave, cue_num):
        coalition_sum_rate = 0
        for tx_c_id in self.get_tx_in_coalition():
            tx_c = dict_id2tx[tx_c_id]
            # 计算D2D用户干扰功率
            rx_c_id = dict_id2tx[tx_c_id].get_rx_id()
            rx_c = dict_id2rx[rx_c_id]

            band_width = 2.16 * pow(10, 9)
            noise_power = -174 + 10 * math.log10(band_width) + 6
            thermal_noise_pow = pow(10, (noise_power - 30) / 10)

            target_tx = tx_c
            target_power = target_tx.get_power()  # dBm
            target_power = pow(10, (target_power - 30) / 10)  # W
            target_channel = dict_id2channel_mmwave[rx_c_id]
            target_link_loss = target_channel.get_link_loss_mmwave(tx_c_id)  # dB
            target_gain = pow(10, -target_link_loss / 10)

            tx_a_gain = target_tx.tx_a_gain_mmwave(dict_id2rx[target_tx.get_rx_id()].get_x_point(),
                                                   dict_id2rx[target_tx
                                                   .get_rx_id()].get_y_point(), dict_id2rx)
            tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
            rx_a_gain = rx_c.rx_a_gain_mmwave(target_tx.get_x_point(), target_tx.get_y_point(), dict_id2tx)
            rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
            receive_target_power = target_power * target_gain * tx_a_gain * rx_a_gain

            receive_inter_power = 0
            for inter_tx_id in self.get_tx_in_coalition():
                if inter_tx_id != tx_c_id:
                    inter_tx = dict_id2tx[inter_tx_id]  # 干扰发射机
                    inter_power = inter_tx.get_power()  # dBm
                    inter_power = pow(10, (inter_power - 30) / 10)  # W
                    inter_channel = dict_id2channel_mmwave[rx_c_id]
                    inter_link_loss = inter_channel.get_link_loss_mmwave(inter_tx_id)  # dB
                    inter_gain = pow(10, -inter_link_loss / 10)
                    tx_a_gain = inter_tx.tx_a_gain_mmwave(rx_c.get_x_point(), rx_c.get_y_point(), dict_id2rx)
                    tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
                    rx_a_gain = rx_c.rx_a_gain_mmwave(inter_tx.get_x_point(), inter_tx.get_y_point(), dict_id2tx)
                    rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
                    receive_inter_power += inter_power * inter_gain * tx_a_gain * rx_a_gain
            sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))

            if sinr < 5:
                coalition_sum_rate += 0
            else:
                coalition_sum_rate += 2.16 * pow(10, 6) * math.log10(1 + pow(10, sinr / 10))
        return coalition_sum_rate


    def get_tx_in_coalition(self):
        return self.__dict_id2tx_coalition

    def get_c_id(self):  # 获得联盟的id
        return self.__coalition_id

    def add_tx(self, tx):  # 向联盟中添加元素
        self.__dict_id2tx_coalition[tx.get_id()] = tx
        self.__num += 1

    def remove_tx(self, tx):  # 删除联盟中的元素
        self.__dict_id2tx_coalition.pop(tx.get_id())
        self.__num -= 1

