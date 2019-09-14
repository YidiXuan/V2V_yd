from vehicular_topology import *
from vehicular_device import *
# import numpy as np
import copy


def graph_colouring(dict_id2tx, dict_id2rx, cue_num, d2d_num, rb_num, dict_id2channel, dict_id2channel_mmwave,
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
    produce_cell_interference(list_nodes, dict_id2channel, dict_id2tx, 5, 25)  # 构建蜂窝干扰图  单位 db
    produce_mmwave_interference_graph(list_nodes, dict_id2channel_mmwave, dict_id2tx, dict_id2rx, 5)
    # 构建毫米波干扰图

    print("干扰图构建完成")
    dict_colored_node = {}  # 存储已着色的节点

    # 初始化V2V用户的可用颜色集
    for node in list_nodes:  # 删除节点不能使用的颜色
        if node.rx_id != 0:  # 仅处理V2V用户
            node.initial_candidate_color(rb_num)
            node.init_candidate_color = copy.deepcopy(node.candidate_color)

    while(len(dict_colored_node)!= d2d_num):
        for node in list_nodes:
            if node.rx_id != 0:
                for color in range(0, rb_num):
                    if node.get_node_id() not in dict_colored_node:
                        node.relation_color2node[color] = 0
                if node.get_node_id() not in dict_colored_node:
                    node.update_relation_color2node(dict_rx_id2node, node.relation_color2node, rb_num)
                # print(node.get_node_id())
                # print(node.relation_color2node)
        if len(dict_colored_node) == 1:
            print(len(dict_colored_node))
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
                 # 计算颜色集中每个节点的degree of contributor
        if len(dict_colored_node) == d2d_num - 1:
            print(len(dict_colored_node))
        print("end")

        # 双图着色算法
        node_maxcolor_list = {}
        node_list = {}
        # 选取每个节点quality最大的蜂窝频段颜色构成字典
        for node in list_nodes:
            if node.rx_id != 0 and node.get_node_id() not in dict_colored_node:
                node_quality = copy.deepcopy(node.quality_color2node)
                if (rb_num - 1) in node_quality:  # 防止选取毫米波节点
                    node_quality.pop(rb_num - 1)
                if len(node_quality) != 0:
                    temp_list = sorted(node_quality.values(), reverse=True)  # 对每个节点quality进行排序
                    node_max_color = temp_list[0]  # 选取最大的quality值
                    color_max = list(node.quality_color2node.keys())[list(node.quality_color2node.values()).index(node_max_color)]  # 搜索得到quality最大值 所对应的节点号
                    node_maxcolor_list[node.rx_id] = node_max_color  # 将所有节点的节点quality存入字典 key ：节点id value：节点quality最大值
                    node_list[node.rx_id] = color_max  # 将所有节点的节点颜色存入字典 可以：节点id value：节点quality最大值所对应的颜色
        if len(node_maxcolor_list) != 0:
            temp_list = sorted(node_maxcolor_list.values(), reverse=True)
            node_max_color = temp_list[0]  # 所有节点的最大quality
            node_max = list(node_maxcolor_list.keys())[list(node_maxcolor_list.values()).index(node_max_color)]
            # quality最大值所对应的节点
            node_color_max = node_list[node_max]  # quality最大值所对应的节点颜色
        else:
            node_max = -1
            node_color_max = -1
        #i = 1
        #while node_color_max == rb_num - 1 and i != len(temp_list):  # 不能选取毫米波颜色
            #node_max_color = temp_list[i]  # 若所选取的颜色为毫米波颜色 则选取第二大的quality值
            #node_max = list(node_maxcolor_list.keys())[list(node_maxcolor_list.values()).index(node_max_color)]
            #node_color_max = node_list[node_max]
            #i = i + 1

        # 选取毫米波中节点度最高的节点
        dict_node_quality_mmwave = {}
        for node_m in list_nodes:
            if node_m.rx_id != 0 and node_m.get_node_id() not in dict_colored_node and (rb_num - 1) in node_m.quality_color2node:
                dict_node_quality_mmwave[node_m.rx_id] = node_m.quality_color2node[rb_num - 1]
                # key：节点id value：节点毫米波quality
        temp_list = sorted(dict_node_quality_mmwave.values(), reverse=True)  # 对节点按quality排序
        if len(temp_list) == 0:
            node_color_max_mmwave = -1
            node_max_mmwave = -1
        else:
            node_max_color_mmwave = temp_list[0]  # 选取最大的quality
            node_color_max_mmwave = rb_num - 1  # 节点颜色
            node_max_mmwave = list(dict_node_quality_mmwave.keys())[list(dict_node_quality_mmwave.values()).index(
                node_max_color_mmwave)]  # 选取最大quality所对应的节点

        compare_cell_mmwave_node(node_color_max, node_color_max_mmwave, node_max, node_max_mmwave, dict_colored_node,
                             dict_rx_id2node, dict_id2tx, dict_id2rx, dict_id2channel, dict_id2channel_mmwave,
                             list_nodes, rb_num)  # 比较节点的吞吐量，并着色，进行相关处理
    print("end")


def compare_cell_mmwave_node(node_max_color, node_max_color_mmwave, node_max, node_max_mmwave, dict_colored_node,
                             dict_rx_id2node, dict_id2tx, dict_id2rx, dict_id2channel, dict_id2channel_mmwave,
                             list_nodes, rb_num):  # 最大的quality，所对应的颜色，所对应的节点
    if node_max != -1 and node_max_color != -1 and node_max_mmwave != -1 and node_max_mmwave != -1:  # 可以选出毫米波节点和蜂窝节点
        node_cell = dict_rx_id2node[node_max]
        node_mmwave = dict_rx_id2node[node_max_mmwave]

        white_noise = -134  # -174dBm / Hz
        noise_fig = 5  # dB
        noise_fig = pow(10, noise_fig / 10)  # 线性值
        thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

        #  计算蜂窝用户的sinr
        tx_cell = dict_id2tx[node_cell.tx_id]  # node1 目标发射机
        tx_cell_power = tx_cell.get_power()  # dBm
        tx_cell_power = pow(10, (tx_cell_power - 30) / 10)  # W
        cell_channel = dict_id2channel[node_cell.rx_id]
        cell_link_loss = cell_channel.get_link_loss(node_cell.tx_id)  # dB
        cell_gain = pow(10, -cell_link_loss / 10)
        receive_cell_power = tx_cell_power * cell_gain

        dict_max_color_node = {}
        for node in dict_colored_node:
            node = dict_colored_node[node]
            if node.color == node_max_color:
                dict_max_color_node[node.get_node_id()] = node
        receive_inter_power = 0
        for node_id in dict_max_color_node:
            if node_id != node_cell.get_node_id():
                inter_node_tx = dict_max_color_node[node_id].tx_id  # 当前干扰节点的发射机id
                inter_tx = dict_id2tx[inter_node_tx]  # 干扰发射机
                inter_power = inter_tx.get_power()  # dBm
                inter_power = pow(10, (inter_power - 30) / 10)  # W
                inter_channel = dict_id2channel[node_cell.rx_id]
                inter_link_loss = inter_channel.get_link_loss(inter_node_tx)  # dB
                inter_gain = pow(10, -inter_link_loss / 10)
                receive_inter_power += inter_power * inter_gain
        node_cell_sinr = 10 * math.log10(receive_cell_power / (receive_inter_power + thermal_noise_pow))

        if node_cell_sinr < 5:
            node_cell_rate = 0
        else:
            node_cell_rate = 180 * math.log10(1 + pow(10, node_cell_sinr / 10))

        # 计算毫米波用户的rate
        band_width = 2.16 * pow(10, 9)

        # 计算噪声功率  1个RB, 12个连续的载波, 12 * 15000 = 180000Hz
        # white_noise = -174  # -174dBm / Hz
        # noise_fig = 6  # dB
        # noise_fig = pow(10, noise_fig / 10)  # 线性值
        # thermal_noise_pow = pow(10, (white_noise - 30) / 10) * band_width  # 线性值  0.5w

        noise_power = -174 + 10 * math.log10(band_width) + 6
        thermal_noise_pow = pow(10, (noise_power - 30) / 10)  # * 2.16 * pow(10, 9)

        # 计算接收目标信号功率
        tx_mmwave = dict_id2tx[node_mmwave.tx_id]  # 目标发射机
        target_power = tx_mmwave.get_power()  # dBm
        target_power = pow(10, (target_power - 30) / 10)  # W
        target_channel = dict_id2channel_mmwave[tx_mmwave.get_rx_id()]
        target_link_loss = target_channel.get_link_loss_mmwave(node_mmwave.tx_id)  # dB
        target_gain = pow(10, -target_link_loss / 10)

        # 发射机为配对的发射机，接收机为自己
        target_rx = tx_mmwave.get_rx_id()
        target_rx = dict_id2rx[target_rx]
        tx_a_gain = tx_mmwave.tx_a_gain_mmwave(target_rx.get_x_point(), target_rx.get_y_point(), dict_id2rx)
        tx_a_gain = pow(10, (tx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 10)
        rx_a_gain = dict_id2rx[tx_mmwave.get_rx_id()].rx_a_gain_mmwave(tx_mmwave.get_x_point(), tx_mmwave.get_y_point(),
                                                                 dict_id2tx)
        rx_a_gain = pow(10, (rx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 10)
        receive_target_power = target_power * target_gain * tx_a_gain * rx_a_gain

        dict_max_color_node = {}
        for node in dict_colored_node:
            node = dict_colored_node[node]
            if node.color == node_max_color_mmwave:
                dict_max_color_node[node.get_node_id()] = node
        receive_inter_power = 0
        for node_id in dict_max_color_node:
            if node_id != node_mmwave.get_node_id():
                inter_node_tx = dict_max_color_node[node_id].tx_id  # 当前干扰节点的发射机id
                inter_tx = dict_id2tx[inter_node_tx]  # 干扰发射机
                inter_power = inter_tx.get_power()  # dBm
                inter_power = pow(10, (inter_power - 30) / 10)  # W
                inter_channel = dict_id2channel_mmwave[node_mmwave.rx_id]
                inter_link_loss = inter_channel.get_link_loss_mmwave(inter_node_tx)  # dB
                inter_gain = pow(10, -inter_link_loss / 10)
                # interfer_power[node_id] = inter_power * inter_gain

                tx_a_gain = inter_tx.tx_a_gain_mmwave(target_rx.get_x_point(), target_rx.get_y_point(), dict_id2rx)
                tx_a_gain = pow(10, (tx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 10)
                rx_a_gain = target_rx.rx_a_gain_mmwave(inter_tx.get_x_point(), inter_tx.get_y_point(), dict_id2tx)
                rx_a_gain = pow(10, (rx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 30)
                receive_inter_power += inter_power * inter_gain * tx_a_gain * rx_a_gain

        node_mmwave_sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
        # 计算毫米波节点的吞吐量
        if node_mmwave_sinr < 5:
            node_mmwave_rate = 0
        else:
            node_mmwave_rate = 2.16 * pow(10, 6) * math.log10(1 + pow(10, node_mmwave_sinr / 10))
        # 对节点进行着色
        if node_mmwave_rate > node_cell_rate:  # 节点着毫米波颜色
            node_mmwave.set_color(node_max_color_mmwave)
            dict_colored_node[node_mmwave.get_node_id()] = node_mmwave  # 在已着色字典中加入着色节点
            dict_id2tx[node_mmwave.tx_id].set_allocated_rb(node_max_color_mmwave)  # 分配发射机rb
            dict_id2rx[node_mmwave.rx_id].set_allocated_rb(node_max_color_mmwave)  # 分配接收机rb
            node_mmwave.delete_candidate_color()  # 删除毫米波节点的候选颜色集，相当于在干扰图中删除该节点
            node_mmwave.relation_color2node = {}
            node_mmwave.quality_color2node = {}
            for node in list_nodes:
                if node_mmwave != node and node.rx_id != 0:
                    if node.rx_id in node_mmwave.cannot_bear_node_mmWave:
                        if node_mmwave.color in node.candidate_color:
                            node.candidate_color.remove(node_mmwave.color)
                    if node_mmwave.get_node_id() in node.cannot_bear_node:
                        node.cannot_bear_node.remove(node_mmwave.get_node_id())  # 删除蜂窝干扰图中，node_mmwave和其邻居节点之间的干扰边
                    node.quality_color2node = {}
                    node.relation_color2node = {}
        if node_mmwave_rate < node_cell_rate:
            node_cell.set_color(node_max_color)  # 节点着蜂窝颜色
            dict_colored_node[node_cell.get_node_id()] = node_cell  # 在已着色字典中加入着色节点
            dict_id2rx[node_cell.rx_id].set_allocated_rb(node_max_color)  # 为接收机分配资源块
            dict_id2tx[node_cell.tx_id].set_allocated_rb(node_max_color)  # 为发射机分配资源块
            node_cell.delete_candidate_color()  # 删除节点的候选颜色集
            node_cell.quality_color2node = {}
            node_cell.relation_color2node = {}
            for node in list_nodes:
                if node_cell != node and node.rx_id != 0:  # V2V 节点
                    if node.rx_id in node_cell.cannot_bear_node:  # node_cell 的邻居节点
                        if node_cell.color in node.candidate_color:  # 删除node_cell 邻居节点候选颜色集中，node_cell已着颜色
                            node.candidate_color.remove(node_cell.color)
                    if node.rx_id in node_cell.cannot_bear_node_mmWave:
                        node.cannot_bear_node_mmWave.remove(node_cell.get_node_id())  # 在毫米波干扰图中， 删除 node_cell与其邻居节点的干扰边
                    node.quality_color2node = {}
                    node.relation_color2node = {}
        if node_mmwave_rate == 0 and node_cell_rate == 0:
            # if 180 * math.log2(1 + pow(10, node_cell_sinr/10)) > 2.16 * pow(10, 6) * math.log2(1 + pow(10, node_mmwave_sinr/10)):
            if node_mmwave_sinr < 0.01 and node_cell_sinr > node_mmwave_sinr:
                node_cell.set_color(node_max_color)  # 节点着蜂窝颜色
                dict_colored_node[node_cell.get_node_id()] = node_cell  # 在已着色字典中加入着色节点
                dict_id2rx[node_cell.rx_id].set_allocated_rb(node_max_color)  # 为接收机分配资源块
                dict_id2tx[node_cell.tx_id].set_allocated_rb(node_max_color)  # 为发射机分配资源块
                node_cell.delete_candidate_color()  # 删除节点的候选颜色集
                node_cell.quality_color2node = {}
                node_cell.relation_color2node = {}
                for node in list_nodes:
                    if node_cell != node and node.rx_id != 0:  # V2V 节点
                        if node.rx_id in node_cell.cannot_bear_node:  # node_cell 的邻居节点
                            if node_cell.color in node.candidate_color:  # 删除node_cell 邻居节点候选颜色集中，node_cell已着颜色
                                node.candidate_color.remove(node_cell.color)
                        if node.rx_id in node_cell.cannot_bear_node_mmWave:
                            node.cannot_bear_node_mmWave.remove(node.get_node_id())  # 在毫米波干扰图中， 删除 node_cell与其邻居节点的干扰边
                        node.quality_color2node = {}
                        node.relation_color2node = {}
            else:
                node_mmwave.set_color(node_max_color_mmwave)
                dict_colored_node[node_mmwave.get_node_id()] = node_mmwave  # 在已着色字典中加入着色节点
                dict_id2tx[node_mmwave.tx_id].set_allocated_rb(node_max_color_mmwave)  # 分配发射机rb
                dict_id2rx[node_mmwave.rx_id].set_allocated_rb(node_max_color_mmwave)  # 分配接收机rb
                node_mmwave.delete_candidate_color()  # 删除毫米波节点的候选颜色集，相当于在干扰图中删除该节点
                node_mmwave.relation_color2node = {}
                node_mmwave.quality_color2node = {}
                for node in list_nodes:
                    if node_mmwave != node and node.rx_id != 0:
                        if node.rx_id in node_mmwave.cannot_bear_node_mmWave:
                            if node_mmwave.color in node.candidate_color:
                                node.candidate_color.remove(node_mmwave.color)
                        if node_mmwave.get_node_id() in node.cannot_bear_node:
                            node.cannot_bear_node.remove(node_mmwave.get_node_id())  # 删除蜂窝干扰图中，node_mmwave和其邻居节点之间的干扰边
                        node.quality_color2node = {}
                        node.relation_color2node = {}
    elif node_max_mmwave != -1 and node_max_mmwave != -1:  # 能选出毫米波节点
        node_mmwave = dict_rx_id2node[node_max_mmwave]
        node_mmwave.set_color(node_max_color_mmwave)
        dict_colored_node[node_mmwave.get_node_id()] = node_mmwave  # 在已着色字典中加入着色节点
        dict_id2tx[node_mmwave.tx_id].set_allocated_rb(node_max_color_mmwave)  # 分配发射机rb
        dict_id2rx[node_mmwave.rx_id].set_allocated_rb(node_max_color_mmwave)  # 分配接收机rb
        node_mmwave.delete_candidate_color()  # 删除毫米波节点的候选颜色集，相当于在干扰图中删除该节点
        node_mmwave.relation_color2node = {}
        node_mmwave.quality_color2node = {}
        for node in list_nodes:
            if node_mmwave != node and node.rx_id != 0:
                if node.rx_id in node_mmwave.cannot_bear_node_mmWave:
                    if node_mmwave.color in node.candidate_color:
                        node.candidate_color.remove(node_mmwave.color)
                if node_mmwave.get_node_id() in node.cannot_bear_node:
                    node.cannot_bear_node.remove(node_mmwave.get_node_id())  # 删除蜂窝干扰图中，node_mmwave和其邻居节点之间的干扰边
                node.quality_color2node = {}
                node.relation_color2node = {}
    elif node_max != -1 and node_max_color != -1: # 能选出蜂窝节点
        node_cell = dict_rx_id2node[node_max]
        node_cell.set_color(node_max_color)  # 节点着蜂窝颜色
        dict_colored_node[node_cell.get_node_id()] = node_cell  # 在已着色字典中加入着色节点
        dict_id2rx[node_cell.rx_id].set_allocated_rb(node_max_color)  # 为接收机分配资源块
        dict_id2tx[node_cell.tx_id].set_allocated_rb(node_max_color)  # 为发射机分配资源块
        node_cell.delete_candidate_color()  # 删除节点的候选颜色集
        node_cell.quality_color2node = {}
        node_cell.relation_color2node = {}
        for node in list_nodes:
            if node_cell != node and node.rx_id != 0:  # V2V 节点
                if node.rx_id in node_cell.cannot_bear_node:  # node_cell 的邻居节点
                    if node_cell.color in node.candidate_color:  # 删除node_cell 邻居节点候选颜色集中，node_cell已着颜色
                        node.candidate_color.remove(node_cell.color)
                if node.rx_id in node_cell.cannot_bear_node_mmWave:
                    node.cannot_bear_node_mmWave.remove(node_cell.get_node_id())  # 在毫米波干扰图中， 删除 node_cell与其邻居节点的干扰边
                node.quality_color2node = {}
                node.relation_color2node = {}
    else:  # 两种节点都不能选出
        print("error")
        for node_cannot_color in list_nodes:
            if node_cannot_color.rx_id != 0:
                if node_cannot_color.rx_id not in dict_colored_node:
                    for color in range(rb_num):
                        node_cannot_color.candidate_color.append(color)

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
                    sir_cue_v2v = compute_sir(node2, node1, dict_id2channel, dict_id2tx)
                    sir = int(sir)
                    if node1.rx_id == 0 and (sir < min_sir_cue or sir_cue_v2v < min_sir_v2v): # 蜂窝用户的中断率判断
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

    sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))  # 单位dB
    return sinr


def produce_mmwave_interference_graph(list_nodes, dict_id2channel_mmwave, dict_id2tx, dict_id2rx, min_sir):
    # 构建毫米波频段干扰图
    for node1 in list_nodes:
        for node2 in list_nodes:
            if node1 != node2 and node1.rx_id != 0 and node2.rx_id != 0: # 毫米波频段只有V2V用户使用
                sir = compute_sir_mmwave(node1, node2, dict_id2channel_mmwave, dict_id2tx, dict_id2rx)
                if sir < min_sir:  # 毫米波频段通过中断率限制或最小传输速率限制构建边
                    if node2.rx_id not in node1.cannot_bear_node_mmWave:
                        node1.cannot_bear_node_mmWave.append(node2.rx_id)
                    if node1.rx_id not in node2.cannot_bear_node_mmWave:
                        node2.cannot_bear_node_mmWave.append(node1.rx_id)


def compute_sir_mmwave(node1, node2, dict_id2channel_mmwave, dict_id2tx, dict_id2rx):
    # 毫米波参数 频率 60GHz EIRP = 20 dBm  带宽：2.16GHz  Noise Figure：6dB  Noise Power : -174 + 10logB + NF dBm
    # 天线模型 sectored model  anternna sidelobe: - 15 dB
    band_width = 2.16 * pow(10, 9)

    # 计算噪声功率  1个RB, 12个连续的载波, 12 * 15000 = 180000Hz
    # white_noise = -174  # -174dBm / Hz
    # noise_fig = 6  # dB
    # noise_fig = pow(10, noise_fig / 10)  # 线性值
    # thermal_noise_pow = pow(10, (white_noise - 30) / 10) * band_width  # 线性值  0.5w

    noise_power = -174 + 10 * math.log10(band_width) + 6
    thermal_noise_pow = pow(10, (noise_power - 30) / 10)  # * 2.16 * pow(10, 9)

    # 计算接收目标信号功率
    target_tx = dict_id2tx[node1.tx_id]  # 目标发射机
    target_power = target_tx.get_power()  # dBm
    target_power = pow(10, (target_power - 30) / 10)  # W
    target_channel = dict_id2channel_mmwave[target_tx.get_rx_id()]
    target_link_loss = target_channel.get_link_loss_mmwave(node1.tx_id)  # dB
    target_gain = pow(10, -target_link_loss / 10)

    # 发射机为配对的发射机，接收机为自己
    target_rx = target_tx.get_rx_id()
    target_rx = dict_id2rx[target_rx]
    tx_a_gain = target_tx.tx_a_gain_mmwave(target_rx.get_x_point(), target_rx.get_y_point(), dict_id2rx)
    tx_a_gain = pow(10, (tx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 10)
    rx_a_gain = dict_id2rx[node1.rx_id].rx_a_gain_mmwave(target_tx.get_x_point(), target_tx.get_y_point(), dict_id2tx)
    rx_a_gain = pow(10, (rx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 10)
    receive_target_power = target_power * target_gain * tx_a_gain * rx_a_gain
    # print(str(target_rx)+"\t"+ str(target_power)+"\t"+ str(tx_a_gain)+"\t"+str(target_link_loss)+"\t"+str(rx_a_gain))
    # print("接受功率：\t"+str(receive_target_power))

    # 计算接收干扰信号总功率
    receive_inter_power = 0
    inter_tx = dict_id2tx[node2.tx_id]  # 干扰发射机
    inter_power = inter_tx.get_power()  # dBm
    inter_power = pow(10, (inter_power - 30) / 10)  # W
    inter_channel = dict_id2channel_mmwave[node1.rx_id]
    inter_link_loss = inter_channel.get_link_loss_mmwave(node2.tx_id)  # dB
    inter_gain = pow(10, -inter_link_loss / 10)

    # 接收机为自己本身，发射机为干扰发射机
    tx_a_gain = inter_tx.tx_a_gain_mmwave(target_rx.get_x_point(), target_rx.get_y_point(), dict_id2rx)
    tx_a_gain = pow(10, (tx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 10)
    rx_a_gain = dict_id2rx[node1.rx_id].rx_a_gain_mmwave(inter_tx.get_x_point(), inter_tx.get_y_point(), dict_id2tx)
    rx_a_gain = pow(10, (rx_a_gain - 3) / 10)  # / pow(10, (0.5 - 30) / 30)
    receive_inter_power = inter_power * inter_gain * tx_a_gain * rx_a_gain
    # print(str(inter_tx) + "\t" + str(inter_power) + "\t" + str(tx_a_gain) + "\t" + str(inter_link_loss) + "\t" + str(
    # rx_a_gain))
    # print("干扰功率：\t"+ str(receive_inter_power))
    # print("噪声：\t"+str(receive_inter_power + thermal_noise_pow))
    sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
    return sinr


def remove_edge(fromnode, tonode):
    tonode.cannot_bear_node.remove(fromnode.get_node_id())
    fromnode.cannot_bear_node.remove(tonode.get_node_id())


class Graph(object):
    def __init__(self):
        self.nodelist = {}

    def addnode(self,node):
        if node in self.nodelist:
            return
        if node.rx_id == 0:
            self.nodelist[node.tx_id] = node
        else:
            self.nodelist[node.rx_id] = node

    def addedge(self, fromnode, tonode):
        if fromnode == tonode:
            return
        if fromnode not in self.nodelist:
            return
        if tonode not in self.nodelist:
            return
        if tonode not in self.nodelist[fromnode.get_node_id()]:
            self.nodelist[fromnode.get_node_id()].cannot_bear_node.append(tonode)
        if fromnode not in self.nodelist[tonode.get_node_id()]:
            self.nodelist[tonode.get_node_id()].cannot_bear_node.append(fromnode)

    def removenode(self,node):
        if node in self.nodelist:
            removed = self.nodelist.pop(node)
            removed = removed.get_node_id()
            for resave_node in self.nodelist:
                if removed in resave_node.cannot_bear_node:
                    resave_node.cannot_bear_node.remove(removed)

    def removeedge(self,fromnode , tonode):
        if fromnode not in self.nodelist:
            if fromnode not in self.nodelist:
                return
            if tonode not in self.nodelist:
                return
        if fromnode in self.nodelist[tonode.get_node_id()].cannot_bear_node:
            self.nodelist[fromnode.get_node_id()].cannot_bear_node(tonode.get_node_id())
            self.nodelist[tonode.get_node_id()].cannot_bear_node(fromnode.get_node_id())


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
        self.init_candidate_color = []

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

    def update_relation_color2node(self, dict_rx_id2node, relation_color2node, rb_num):  # 计算节点使用某一rb，对其他节点的影响  未完成
        if len(self.cannot_bear_node) > 0:
            for node_id in self.cannot_bear_node:
                if dict_rx_id2node[node_id].rx_id != 0:
                    for color in self.candidate_color:
                        if color in dict_rx_id2node[node_id].candidate_color:
                            relation_color2node[color] += 1
        if len(self.cannot_bear_node_mmWave) > 0:
            for node_id in self.cannot_bear_node_mmWave:
                if dict_rx_id2node[node_id].rx_id != 0:
                    color = rb_num-1
                    if color in dict_rx_id2node[node_id].candidate_color:
                        relation_color2node[color] += 1


    def update_color2quality(self, dict_color_id2node, color, rb_num, dict_id2channel, dict_id2channel_mmwave,
                             dict_id2tx, dict_id2rx):  # V2V用户
        if color != rb_num - 1:
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

        if color == rb_num - 1:  # 毫米波信道
            # if self not in dict_color_id2node:
                # white_noise = -174  # -174dBm / Hz
                # noise_fig = 6  # dB
                # noise_fig = pow(10, noise_fig / 10)  # 线性值
                # thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 2.16 * math.pow(10, 9)  # 线性值  0.5w

            noise_power = -174 + 10 * math.log10(2.16 * pow(10, 6)) + 6
            thermal_noise_pow = pow(10, (noise_power - 30) / 10)

            # 计算接收目标信号功率
            target_tx = dict_id2tx[self.tx_id]  # 目标发射机
            target_power = target_tx.get_power()  # dBm
            target_power = pow(10, (target_power - 30) / 10)  # W
            target_channel = dict_id2channel_mmwave[self.rx_id]
            target_link_loss = target_channel.get_link_loss_mmwave(self.tx_id)  # dB
            target_gain = pow(10, -target_link_loss / 10)
            # 发射机为配对的发射机，接收机为自己
            tx_a_gain = target_tx.tx_a_gain_mmwave(dict_id2rx[target_tx.get_rx_id()].get_x_point(),
                                                       dict_id2rx[target_tx.get_rx_id()].get_y_point(), dict_id2rx)
            tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
            rx_a_gain = dict_id2rx[self.rx_id].rx_a_gain_mmwave(target_tx.get_x_point(), target_tx.get_y_point(),
                                                                dict_id2tx)
            rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
            receive_target_power = target_power * target_gain * tx_a_gain * rx_a_gain

            # 计算接收干扰信号总功率
            receive_inter_power = 0
            for node_id in dict_color_id2node:
                if node_id != self.get_node_id():
                    inter_node_tx = dict_color_id2node[node_id].tx_id
                    inter_tx = dict_id2tx[inter_node_tx]  # 干扰发射机
                    inter_power = inter_tx.get_power()  # dBm
                    inter_power = pow(10, (inter_power - 30) / 10)  # W
                    inter_channel = dict_id2channel_mmwave[self.rx_id]
                    inter_link_loss = inter_channel.get_link_loss_mmwave(inter_node_tx)  # dB
                    inter_gain = pow(10, -inter_link_loss / 10)
                    # 接收机为自己本身，发射机为干扰发射机
                    tx_a_gain = inter_tx.tx_a_gain_mmwave(dict_id2rx[self.rx_id].get_x_point(), dict_id2rx[self.
                                                          rx_id].get_y_point(), dict_id2rx)
                    tx_a_gain = pow(10, (tx_a_gain - 3) / 10)
                    rx_a_gain = dict_id2rx[self.rx_id].rx_a_gain_mmwave(inter_tx.get_x_point(),
                                                                        inter_tx.get_y_point(), dict_id2tx)
                    rx_a_gain = pow(10, (rx_a_gain - 3) / 10)
                    receive_inter_power += inter_power * inter_gain * tx_a_gain * rx_a_gain

            sinr = 10 * math.log10(receive_target_power / (receive_inter_power + thermal_noise_pow))
            self.quality_color2node[color] = sinr / (self.relation_color2node[color] + 1)  # kHz

    def delete_candidate_color(self):
        self.candidate_color = []
        self.interested_color = -1








