from vehicular_topology import *
from vehicular_device import *
# import numpy as np

def graph_colouring_icc(dict_id2tx, dict_id2rx, cue_num, d2d_num, rb_num, dict_id2channel, dict_id2channel_mmwave,
                    min_sir_cue, min_sir_v2v):
    dict_rx_id2node = {}
    list_nodes = []
    # 蜂窝用户分配正交频段
    tx_id = dict_id2rx[0].get_tx_id()
    for tx_i_id in tx_id:
        temp_node = Node(dict_id2tx[tx_i_id], dict_id2rx[0])
        temp_node.set_color(tx_i_id - 1)  # 蜂窝着色的范围为0-19
        temp_node.candidate_color.append(tx_i_id - 1)
        dict_rx_id2node[tx_i_id] = temp_node  # 按发射机记
        list_nodes.append(temp_node)
        dict_id2tx[tx_i_id].set_allocated_rb(tx_i_id - 1)
        dict_id2rx[0].set_allocated_rb(tx_i_id - 1)
    # print(str(temp_node.tx_id)+"\t"+ str(temp_node.rx_id)+"\t" + str(temp_node.get_node_id()) + str(temp_node.color))

    # 构建节点
    produce_node(dict_id2tx, dict_id2rx, rb_num, dict_rx_id2node, list_nodes)  # 构建 V2V节点

    # 构建干扰图
    produce_cell_interference(list_nodes, dict_id2channel, dict_id2tx, min_sir_cue, pow(10, 5/10))  # 构建蜂窝干扰图

    print("干扰图构建完成")
    dict_colored_node = {}  # 存储已着色的节点

    # 初始化V2V用户的可用颜色集
    for node in list_nodes:  # 删除节点不能使用的颜色
        if node.rx_id != 0:  # 仅处理V2V用户
            node.initial_candidate_color(rb_num)

    for node in list_nodes:
        if node.rx_id != 0:
            for color in range(0, rb_num):
                if node.get_node_id() not in dict_colored_node:
                    node.relation_color2node[color] = 0
            if node.get_node_id() not in dict_colored_node:
                node.update_relation_color2node(dict_rx_id2node, node.relation_color2node)
            # print(node.get_node_id())
            # print(node.relation_color2node)
    # 计算节点的 quality
    for color in range(0, rb_num):
            # if color == rb_num - 1:
            # print("end")
        dict_color_id2node = {}
        for node in list_nodes:
            if (color in node.candidate_color) or (color == node.color):
                dict_color_id2node[node.get_node_id()] = node  # 将候选颜色集中包括color的点总结在一个dict_color_id_node
        # if color == rb_num - 1:
            # print("mmwave start")
        for node_id in dict_color_id2node:
            if dict_color_id2node[node_id].rx_id != 0:
                if node_id not in dict_colored_node:
                    dict_color_id2node[node_id].update_color2quality(dict_color_id2node, color, rb_num, dict_id2channel,
                                                                 dict_id2channel_mmwave, dict_id2tx, dict_id2rx)
        dict_node_id2quality = {}
        for node_id in dict_color_id2node:
            if dict_rx_id2node[node_id].rx_id != 0:
                dict_node_id2quality[node_id] = dict_color_id2node[node_id].quality_color2node[color]


    print("end")

def produce_node(dict_id2tx, dict_id2rx, rb_num, dict_rx_id2node, list_nodes):  # 构建V2V节点
    for rx_id in dict_id2rx:
        if rx_id != 0:  # 构建 V2V节点
            tx_id = dict_id2rx[rx_id].get_tx_id()  # 获得V2V发射车辆id
            temp_node = Node(dict_id2tx[tx_id], dict_id2rx[rx_id])
            temp_node.set_candidate_color(rb_num)
            # 设置节点的候选颜色集，为两部分 蜂窝频段（0~rb_num-1）和毫米波频段 rb_num
            dict_rx_id2node[rx_id] = temp_node
            list_nodes.append(temp_node)


def produce_cell_interference(list_nodes, dict_id2channel, dict_id2tx, min_sir_cue, min_sir_v2v):  # 构建蜂窝频段干扰图
    for node1 in list_nodes:
        for node2 in list_nodes:
            if node1 != node2:
                if node1.rx_id != 0 | node2.rx_id != 0: # 蜂窝用户之间已正交分配频谱，因此不考虑他们之间的干扰
                    sir = compute_sir(node1, node2, dict_id2channel, dict_id2tx)
                    sir = int(sir)
                    if node1.rx_id == 0 and sir < min_sir_cue: # 蜂窝用户的中断率判断
                        if node2.rx_id not in node1.cannot_bear_node:
                            node1.cannot_bear_node.append(node2.rx_id)
                        if node1.tx_id not in node2.cannot_bear_node:
                            node2.cannot_bear_node.append(node1.tx_id)
                    if node1.rx_id != 0:  # V2V用户的中断率或最小传输速率判断
                        if sir < min_sir_v2v:
                            if node2.rx_id == 0:  # 若干扰节点为蜂窝用户，记录其发射机
                                if node2.tx_id not in node1.cannot_bear_node:
                                    node1.cannot_bear_node.append(node2.tx_id)
                                if node1.rx_id not in node2.cannot_bear_node:
                                    node2.cannot_bear_node.append(node1.rx_id)
                            else:  # 若干扰节点为V2V用户，记录其接收机
                                if node2.rx_id not in node1.cannot_bear_node:
                                    node1.cannot_bear_node.append(node2.rx_id)
                                if node1.rx_id not in node2.cannot_bear_node:
                                    node2.cannot_bear_node.append(node1.rx_id)
    # print("end")


def compute_sir(node1, node2, dict_id2channel, dict_id2tx):  # 蜂窝频段sir计算
    white_noise = -134  # -174dBm / Hz
    if node1.rx_id == 0:
        white_noise = -134
    noise_fig = 5  # dB
    noise_fig = pow(10, noise_fig / 10)  # 线性值
    thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

    target_tx = dict_id2tx[node1.tx_id]  # node1 目标发射机
    target_power = target_tx.get_power()  # dBm
    target_power = pow(10, (target_power - 30) / 10)  # W
    target_channel = dict_id2channel[node1.rx_id]
    target_link_loss = target_channel.get_link_loss(node1.tx_id)  # dB
    target_gain = pow(10, -target_link_loss / 10)
    receive_target_power = target_power * target_gain

    # 计算接收干扰信号总功率
    receive_inter_power = 0
    inter_tx = dict_id2tx[node2.tx_id]  # 干扰发射机
    inter_power = inter_tx.get_power()  # dBm
    inter_power = pow(10, (inter_power - 30) / 10)  # W
    inter_channel = target_channel
    inter_link_loss = inter_channel.get_link_loss(node2.tx_id)  # dB
    inter_gain = pow(10, -inter_link_loss / 10)
    receive_inter_power += inter_power * inter_gain

    sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
    return sinr



class Node(object):
    def __init__(self, tx, rx):
        self.tx_id = tx.get_id()
        self.rx_id = rx.get_id()
        self.tx_x_point = tx.get_x_point()
        self.tx_y_point = tx.get_y_point()
        self.rx_x_point = rx.get_x_point()
        self.rx_y_point = rx.get_y_point()

        self.cannot_bear_node = []  # 存储对自身产生难以容忍干扰的节点的发射机id
        self.cannot_bear_node_mmWave = []  # 存储在毫米波频段对自身产生难以容忍干肉的节点

        self.relation_color2node = {}  # 存储节点使用某一rb对其他节点的影响 key: color value:degree of influence

        self.quality_color2node = {}  # 节点使用某rb对系统性能的贡献，即（SIRNR_d2d)/节点影响

        self.color = -1  # 节点的颜色

        self.candidate_color = []  # 候选颜色集
        self.interested_color = -1  # 当前候选颜色

    def set_color(self, cue_id):  # 选择D2D复用哪个cue的频段
        self.color = cue_id

    def get_node_id(self):
        if self.rx_id == 0:
            return self.tx_id
        else:
            return self.rx_id

    def set_candidate_color(self, rb_num):  # 蜂窝频段的rb范围为 0~rb_num-1 毫米波频段为 rb_num
        for cue_id in range(0, rb_num):
            self.candidate_color.append(cue_id)
            self.interested_color = self.candidate_color[0]

    def initial_candidate_color(self, rb_num):  # 初始化节点候选集中的颜色，删除对cue存在干扰的d2d节点中cue所对应的color
        for cue_id in range(0, rb_num):
            cue_id_0 = cue_id + 1
            if cue_id_0 in self.cannot_bear_node:
                self.candidate_color.remove(cue_id_0 - 1)
        if len(self.candidate_color) > 0:
            self.interested_color = self.candidate_color[0]

    def update_candidate_color(self, color):
        if color in self.candidate_color:
            self.candidate_color.remove(color)
        if len(self.candidate_color) > 0:
            self.interested_color = self.candidate_color[0]

    def update_relation_color2node(self, dict_rx_id2node, relation_color2node):  # 计算节点使用某一rb，对其他节点的影响  未完成
        if len(self.cannot_bear_node) > 0:
            for node_id in self.cannot_bear_node:
                if dict_rx_id2node[node_id].rx_id != 0:
                    for color in self.candidate_color:
                        if color in dict_rx_id2node[node_id].candidate_color:
                            relation_color2node[color] += 1


    def update_color2quality(self, dict_color_id2node, color, rb_num, dict_id2channel, dict_id2channel_mmwave,
                             dict_id2tx, dict_id2rx):  # V2V用户
        white_noise = -134  # -174dBm / Hz
        noise_fig = 5  # dB
        noise_fig = pow(10, noise_fig / 10)  # 线性值
        thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

        # 计算接收目标信号功率
        target_tx = dict_id2tx[self.tx_id]  # 目标发射机
        target_power = target_tx.get_power()  # dBm
        target_power = pow(10, (target_power - 30) / 10)  # W
        target_channel = dict_id2channel[self.rx_id]
        target_link_loss = target_channel.get_link_loss(self.tx_id)  # dB
        target_gain = pow(10, -target_link_loss / 10)
        receive_target_power = target_power * target_gain

        # 计算接收干扰信号总功率
        receive_inter_power = 0
        # interfer_power = {}
        for node_id in dict_color_id2node:
            if node_id != self.get_node_id():
                inter_node_tx = dict_color_id2node[node_id].tx_id  # 当前干扰节点的发射机id
                inter_tx = dict_id2tx[inter_node_tx]  # 干扰发射机
                inter_power = inter_tx.get_power()  # dBm
                inter_power = pow(10, (inter_power - 30) / 10)  # W
                inter_channel = dict_id2channel[self.rx_id]
                inter_link_loss = inter_channel.get_link_loss(inter_node_tx)  # dB
                inter_gain = pow(10, -inter_link_loss / 10)
                # interfer_power[node_id] = inter_power * inter_gain
                receive_inter_power += inter_power * inter_gain

        sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))

        self.quality_color2node[color] = sinr / (self.relation_color2node[color] + 1)  # kHz



    def delete_candidate_color(self):
        self.candidate_color = []
        self.interested_color = -1