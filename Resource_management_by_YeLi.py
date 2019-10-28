from vehicular_topology import *
from vehicular_device import *
import copy
import itertools
import numpy as np
from scipy.optimize import linear_sum_assignment
# import matlab.engine
# import numpy as np


def Resource_management_by_YeLi(dict_id2tx, dict_id2rx, dict_id2channel, rb_num, cue_num, d2d_num, min_sinr_cue, min_sinr_d2d):
    dict_rb_id2cwn = {}
    dict_cue_id2d2d_power = {}  # 存储复用蜂窝用户的rb时，D2D传输功率
    n_cut(dict_id2tx, dict_id2rx, dict_id2channel, rb_num, cue_num, d2d_num, dict_rb_id2cwn)  # 分簇算法
    for cue_id in range(1, 1 + cue_num):  # 功率控制
        dict_d2d_power = {}
        for c_id in dict_rb_id2cwn:
            # power_control_1v1(dict_id2tx, dict_id2rx, dict_id2channel, dict_id2tx[cue_id], dict_rb_id2cwn[c_id], c_id, pow(10, 3/10), dict_d2d_power)
            power_control(dict_id2tx, dict_id2rx, dict_id2channel, cue_id, dict_rb_id2cwn[c_id], c_id, d2d_num, cue_num,
                        min_sinr_d2d, dict_d2d_power)
            # dict_d2d_power[str(c_id)] = 17
            # for d2d in range(1 + cue_num, 1 + cue_num + d2d_num):
                # dict_d2d_power[d2d] = 17
        dict_cue_id2d2d_power[cue_id] = copy.deepcopy(dict_d2d_power)
        #cfor c_id in dict_rb_id2cwn:
            # coalition = dict_rb_id2cwn[c_id]
            # if cue_id not in coalition.get_cue_capacity():
                # com_coalition_capacity(dict_id2tx, dict_id2rx, dict_id2channel, cue_num, d2d_num, coalition, cue_id, c_id, dict_cue_id2d2d_power)
        # print("end")

    for rx_id in dict_id2rx:
        rx = dict_id2rx[rx_id]
        rx_channel = dict_id2channel[rx_id]
        for tx_id in dict_id2tx:
            if tx_id not in range(1, 1 + cue_num) and rx_id != 0:
                channel_link_loss = rx_channel.get_link_loss(tx_id)
                channel_link_loss = pow(10, -channel_link_loss / 10) * math.sqrt(pow(random.gauss(0, 1), 2))
                channel_link_loss = -10 * math.log10(channel_link_loss)
                rx_channel.set_link_loss_cell(channel_link_loss, tx_id)

    for cue_id in range(1, 1 + cue_num):  # 计算吞吐量
        for c_id in dict_rb_id2cwn:
            coalition = dict_rb_id2cwn[c_id]
            if cue_id not in coalition.get_cue_capacity():
                com_coalition_capacity(dict_id2tx, dict_id2rx, dict_id2channel, cue_num, d2d_num, coalition, cue_id, c_id, dict_cue_id2d2d_power)


    #  二维匹配，匈牙利算法
    # 构建 task_matric
    task_matric = np.ones([cue_num, cue_num])
    i = -1
    for cue_id in range(1, 1 + cue_num):
        i = i + 1
        j = -1
        for c_id in range(1, 1 + len(dict_rb_id2cwn)):
            j = j + 1
            task_matric[i][j] = - round(dict_rb_id2cwn[c_id].get_cue_capacity()[cue_id], 4)
            if task_matric[i][j] == 0:
                task_matric[i][j] = 100
    # task_matric = np.matrix(task_matric)
    # print(task_matric)

    # ass_by_Hun = TaskAssignment(task_matric, 'Hungary')
    # ass_by_all = TaskAssignment(task_matric, 'all_permutation')
    # cue_l = ass_by_Hun.best_solution_row
    # coalition_l = ass_by_Hun.best_solution_h
    row_ind, col_ind = linear_sum_assignment(task_matric)

    coalition_l = col_ind
    cue_l = row_ind
    # for i in cue_num:
        # cue_l = cue_l.append(i)
    # print(task_matric)
    # print(ass_by_Hun.best_solution)
    # print("end")
    i = 0
    for coalition in coalition_l:
        dict_rb_id2cwn[coalition + 1].set_cue_use(cue_l[i])
        i += 1

    for tx_id in range(1, 1 + cue_num):
        power = dict_cue_id2d2d_power[tx_id][str(coalition_l[tx_id - 1] + 1)]
        dict_id2tx[tx_id].set_power(power)
    for c_id in range(1, 1 + cue_num):
        coalition = dict_rb_id2cwn[c_id]
        c_member = coalition.get_members()
        rb_use = coalition.get_cue_use()
        for tx_id in c_member:
            power = dict_cue_id2d2d_power[rb_use + 1][tx_id]
            dict_id2tx[tx_id].set_power(power)
            dict_id2tx[tx_id].set_allocated_rb(rb_use)
            dict_id2rx[dict_id2tx[tx_id].get_rx_id()].set_allocated_rb(rb_use)




def n_cut(dict_id2tx, dict_id2rx, dict_id2channel, rb_num, cue_num, d2d_num, dict_rb_id2cwn):
    rb_num = rb_num
    dict_id2tx_after_cut = {}
    # dict_rb_id2cwn = {}
    for n in range(1, rb_num + 1):  # 初始化无干扰簇
        dict_rb_id2cwn[n] = CWN()

    tx_id = random.randint(1 + cue_num,  cue_num + d2d_num)  # 随机选择d2d
    tx_have_chose = []  # 保证不会重复选择车辆
    while tx_id not in tx_have_chose:
        tx_have_chose.append(tx_id)
        tx = dict_id2tx[tx_id]
        rx_id = tx.get_rx_id()
        channel = dict_id2channel[rx_id]  # 当前被分配D2D的接受车辆
        dict_rb_id2cwn_noise = {}  # 存储当前被分配D2D和所有cwn的干扰
        for n in range(1, rb_num + 1):
            sum_link_loss = 0
            for tx_id_2 in dict_rb_id2cwn[n].get_members():
                channel2 = dict_id2channel[dict_id2tx[tx_id_2].get_rx_id()]
                sum_link_loss += pow(10, -channel.get_link_loss(tx_id_2) / 10) + pow(10, -channel2.get_link_loss(tx_id) / 10)
            dict_rb_id2cwn_noise[n] = sum_link_loss
        noise_list = sorted(dict_rb_id2cwn_noise.values(), reverse=False)
        noise_min = noise_list[0]
        c_id_min_noise = list(dict_rb_id2cwn_noise.keys())[list(dict_rb_id2cwn_noise.values()).index(noise_min)]
        dict_rb_id2cwn[c_id_min_noise].set_member(tx)
        while tx_id in tx_have_chose and len(tx_have_chose) != d2d_num:
            tx_id = random.randint(1 + cue_num,  cue_num + d2d_num)
    # print("end")

def power_control_1v1(dict_id2tx, dict_id2rx, dict_id2channel, cue, coalition, c_id, min_sinr_d2d, dict_d2d_power):
    for tx_id in coalition.get_members():
        due = dict_id2tx[tx_id]
    cue_id = cue.get_id()
    white_noise = -134  # -174dBm / Hz
    noise_fig = 5  # dB
    noise_fig = pow(10, noise_fig / 10)  # 线性值
    thermal_noise_pow = pow(10, (white_noise - 30) / 10) # * pow(10, 5) * 1.8    # 线性值

    due_tx = due  # d2d接受车辆
    due_tx_id = due_tx.get_id()
    due_rx_id = due_tx.get_rx_id()
    channel = dict_id2channel[due_rx_id]  # D2D对的信道
    d2d_link_loss = channel.get_link_loss(due_tx_id)
    d2d_link_loss = pow(10, - d2d_link_loss / 10)

    cue_d2d_link_loss = channel.get_link_loss(cue_id)
    cue_d2d_link_loss = pow(10, - cue_d2d_link_loss / 10)

    d2d_min_power = - min_sinr_d2d * thermal_noise_pow / (d2d_link_loss * math.log(1 - 0.05))

    if d2d_min_power > pow(10, (17 - 30) / 10): # d2d 的最大发射功率为10
        dict_d2d_power[str(c_id)] = 0
        dict_d2d_power[due_tx_id] = 0
        coalition.set_cue_capacity(cue_id, 0)
        coalition.set_capacity(cue_id, 0)
    else:
        x = d2d_link_loss * pow(10, (17 - 30) / 10) / (min_sinr_d2d * cue_d2d_link_loss)
        y = (math.exp(- (min_sinr_d2d * thermal_noise_pow) / (pow(10, (17 - 30) / 10) * d2d_link_loss)) / (1 - 0.05)) - 1
        cue_power_d2d_powe_max = x * y
        if cue_power_d2d_powe_max > pow(10, (17 - 30) / 10):
            cue_power = pow(10, (17 - 30) / 10)
            d2d_low = d2d_min_power
            d2d_max = pow(10, (17 - 30) / 10)
            d2d_mid = (d2d_low + d2d_max) / 2
            x2 = d2d_link_loss * d2d_mid / (min_sinr_d2d * cue_d2d_link_loss)
            y2 = (math.exp(- (min_sinr_d2d * thermal_noise_pow) / (d2d_mid * d2d_link_loss)) / (1 - 0.05)) - 1
            # d2d_mid2 = 0.009
            # y3 = (math.exp(- (min_sinr_d2d * thermal_noise_pow) / (d2d_mid2 * d2d_link_loss)) / (1 - 0.1)) - 1
            cue_power_mid = x2 * y2
            while abs(cue_power_mid - cue_power) > pow(10, -5):
                if cue_power_mid < cue_power:
                    d2d_low = d2d_mid
                    d2d_mid = (d2d_low + d2d_max) / 2
                    x2 = d2d_link_loss * d2d_mid / (min_sinr_d2d * cue_d2d_link_loss)
                    y2 = (math.exp(- (min_sinr_d2d * thermal_noise_pow) / (d2d_mid * d2d_link_loss)) / (1 - 0.05)) - 1
                    cue_power_mid = x2 * y2
                else:
                    d2d_max = d2d_mid
                    d2d_mid = (d2d_low + d2d_max) / 2
                    x2 = d2d_link_loss * d2d_mid / (min_sinr_d2d * cue_d2d_link_loss)
                    y2 = (math.exp(-(min_sinr_d2d * thermal_noise_pow) / (d2d_mid * d2d_link_loss)) / (1 - 0.05)) - 1
                    cue_power_mid = x2 * y2
            d2d_power = d2d_mid
        else:
            cue_power = cue_power_d2d_powe_max
            d2d_power = pow(10, (17 - 30) / 10)
        dict_d2d_power[str(c_id)] = 10 * math.log10(cue_power) + 30
        dict_d2d_power[due_tx_id] = 10 * math.log10(d2d_power) + 30
        # d2d发射功率最大时 cue的发射功率


    # print("end")


def power_control(dict_id2tx, dict_id2rx, dict_id2channel, cue_id, coalition, c_id, d2d_num, cue_num, min_sinr_d2d, dict_d2d_power):
    r_average = -pow(10, 5 / 10) / math.log(1 - 0.05)
    white_noise = -134  # -174dBm / Hz
    noise_fig = 5  # dB
    noise_fig = pow(10, noise_fig / 10)  # 线性值
    thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig # 线性值


    cue = dict_id2tx[cue_id]
    d2d_number = coalition.get_number()
    a = []  # a=[a[m,1], a[m,2]....,a[m,cn]] 蜂窝用户到D2D用户的链路状态信息
    for d2d_tx_id in range(1 + cue_num, 1 + cue_num + d2d_num):
        if d2d_tx_id in coalition.get_members():
            d2d_rx_id = dict_id2tx[d2d_tx_id].get_rx_id()
            channel = dict_id2channel[d2d_rx_id]
            link_loss = channel.get_link_loss(cue_id)
            link_loss = pow(10, -link_loss / 10)
            a.append(link_loss)
            dict_d2d_power[d2d_tx_id] = 0
    a = np.asarray(a)
    a = np.matrix(a)
    a_t = np.transpose(a)
    # print(a)
    # print(a_t)

    fa = np.zeros([d2d_number, d2d_number])
    i = -1
    j = -1
    for d2d_tx_id in range(1 + cue_num, 1 + cue_num + d2d_num):
        if d2d_tx_id in coalition.get_members():
            i = i + 1
            d2d_rx_id = dict_id2tx[d2d_tx_id].get_rx_id()
            j = -1
            for d2d_tx_id_2 in range(1 + cue_num, 1 + cue_num + d2d_num):
                if d2d_tx_id_2 in coalition.get_members():
                    j = j + 1
                    channel = dict_id2channel[d2d_rx_id]
                    link_loss = channel .get_link_loss(d2d_tx_id_2)
                    link_loss = pow(10, -link_loss / 10)
                    fa[i][j] = link_loss
                    if d2d_tx_id_2 != d2d_tx_id:
                        fa[i][j] = - link_loss * r_average
    fa = np.matrix(fa)
    # fa_t = np.transpose(fa)
    fa_inv = fa.I

    cue_power_optimal = [pow(10, (17 - 30) / 10)]
    for i in range(0, d2d_number):
        fa_inv_hang = fa_inv[i]
        x_pc = fa_inv_hang * a_t * r_average
        one_h = np.matrix(np.ones(d2d_number))
        one_l = np.transpose(one_h)
        c = fa_inv_hang * one_l * r_average * thermal_noise_pow

        cue_power_optimal.append(((pow(10, (17 - 30) / 10) - c) / x_pc).tolist()[0][0])
    cue_power_optimal = sorted(cue_power_optimal)
    cue_power = cue_power_optimal[0]
    if cue_power <= 0:
        dict_d2d_power[str(c_id)] = 10
        coalition.set_cue_capacity(cue_id, 0)
        coalition.set_capacity(cue_id, 0)
    if cue_power > 0:
        dict_d2d_power[str(c_id)] = 10 * math.log10(cue_power) + 30
        fa_inv = fa.I
        # print(np.transpose(fa))
        # print(fa_inv)
        power_d2d = fa_inv * r_average * (cue_power * a_t + thermal_noise_pow)
        # print(power_d2d)
        power = power_d2d.tolist()
        # print(power)
        i = 0
        for d2d_tx_id in range(1 + cue_num, 1 + cue_num + d2d_num):
            if d2d_tx_id in coalition.get_members():
                if power[i][0] < 0:
                    # dict_d2d_power[d2d_tx_id] = -100
                    dict_d2d_power[str(c_id)] = 0
                    coalition.set_cue_capacity(cue_id, 0)
                    coalition.set_capacity(cue_id, 0)
                else:
                    dict_d2d_power[d2d_tx_id] = 10 * math.log10(power[i][0] / pow(10, -3))
                    if dict_d2d_power[d2d_tx_id] > 17:
                        dict_d2d_power[d2d_tx_id] = 17
                    i += 1

def com_coalition_capacity(dict_id2tx, dict_id2rx, dict_id2channel, cue_num, d2d_num, coalition, cue_id, c_id, dict_cue_id2d2d_power):
    coalition_members = coalition.get_members()
    coalition_members[cue_id] = dict_id2tx[cue_id]
    rate = 0
    for target_tx_id in coalition_members:
        white_noise = -134  # -174dBm / Hz
        noise_fig = 5  # dB
        noise_fig = pow(10, noise_fig / 10)  # 线性值
        thermal_noise_pow = pow(10, (white_noise - 30) / 10) * 180000 * noise_fig  # 线性值

        target_tx = dict_id2tx[target_tx_id]
        if target_tx_id in range(1, 1 + cue_num):
            target_rx_id = 0
            target_power = dict_cue_id2d2d_power[cue_id][str(c_id)]  # dBm
        if target_tx_id in range(1 + cue_num, 1 + cue_num + d2d_num):
            target_rx_id = target_tx.get_rx_id()
            target_power = dict_cue_id2d2d_power[cue_id][target_tx_id]
        target_power = pow(10, (target_power - 30) / 10)  # W
        target_channel = dict_id2channel[target_rx_id]
        target_link_loss = target_channel.get_link_loss(target_tx_id)  # dB
        target_gain = pow(10, -target_link_loss / 10)
        receive_target_power = target_power * target_gain

        receive_inter_power = 0
        for inter_tx_id in coalition_members:
            if inter_tx_id != target_tx_id:
                inter_tx = dict_id2tx[inter_tx_id]  # 干扰发射机
                inter_power = inter_tx.get_power()  # dBm
                if inter_tx_id in range(1, 1 + cue_num):
                    inter_power = dict_cue_id2d2d_power[cue_id][str(c_id)]  # dBm
                if inter_tx_id in range(1 + cue_num, 1 + cue_num + d2d_num):
                    inter_power = dict_cue_id2d2d_power[cue_id][inter_tx_id]
                inter_power = pow(10, (inter_power - 30) / 10)  # W
                inter_channel = dict_id2channel[target_rx_id]
                inter_link_loss = inter_channel.get_link_loss(inter_tx_id)  # dB
                inter_gain = pow(10, -inter_link_loss / 10)
                receive_inter_power += inter_power * inter_gain
        sinr = receive_target_power / (receive_inter_power + thermal_noise_pow)
        # sinr = 10 * math.log10(sinr)
        if target_tx_id in range(1, 1+cue_num):
            if math.log2(1 + sinr) > 0:
                coalition.set_cue_capacity(target_tx_id, math.log2(1 + sinr)) #  0dB中断
            else:
                coalition.set_cue_capacity(target_tx_id, 0)
        else:
            if 10 * math.log10(sinr) > 5:
                rate += math.log2(1 + sinr)
            else:
                rate += 0
    coalition.set_capacity(cue_id, rate)
    coalition.remove_member(cue_id)
    # print('end')


class CWN:
    def __init__(self):
        self.__number = 0
        self.__members = {}
        self.__cue_id2d2d_capacity = {}
        self.__cue_rate = {}
        self.__cue_use = -1

    def set_member(self, tx):
        self.__members[tx.get_id()] = tx
        self.__number += 1

    def remove_member(self, tx):
        self.__members.pop(tx)

    def get_members(self):
        return self.__members

    def get_number(self):
        return self.__number

    def set_capacity(self, cue_id, capacity):
        self.__cue_id2d2d_capacity[cue_id] = capacity

    def set_cue_capacity(self, cue_id, capacity):
        self.__cue_rate[cue_id] = capacity

    def get_capacity(self, cue_id):
        return self.__cue_id2d2d_capacity[cue_id]

    def get_cue_capacity(self):
        return self.__cue_rate

    def set_cue_use(self, rb):
        self.__cue_use = rb

    def get_cue_use(self):
        return self.__cue_use

# 任务分配类
class TaskAssignment:

    # 类初始化，需要输入参数有任务矩阵以及分配方式，其中分配方式有两种，全排列方法all_permutation或匈牙利方法Hungary。
    def __init__(self, task_matrix, mode):
        self.task_matrix = task_matrix
        self.mode = mode
        if mode == 'all_permutation':
            self.min_cost, self.best_solution, self.best_solution_c = self.all_permutation(task_matrix)
        if mode == 'Hungary':
            self.min_cost, self.best_solution, self.best_solution_row, self.best_solution_h = self.Hungary(task_matrix)

    # 全排列方法
    def all_permutation(self, task_matrix):
        number_of_choice = len(task_matrix)
        solutions = []
        values = []
        for each_solution in itertools.permutations(range(number_of_choice)):
            each_solution = list(each_solution)
            each_solution_c = []
            solution = []
            value = 0
            for i in range(len(task_matrix)):
                value += task_matrix[i][each_solution[i]]
                solution.append(task_matrix[i][each_solution[i]])
            values.append(value)
            solutions.append(solution)
            each_solution_c.append(each_solution)
        min_cost = np.min(values)
        best_solution = solutions[values.index(min_cost)]
        best_solution_c = each_solution_c[values.index(min_cost)]
        return min_cost, best_solution, best_solution_c

    # 匈牙利方法
    def Hungary(self, task_matrix):
        b = task_matrix.copy()
        # 行和列减0
        for i in range(len(b)):
            row_min = np.min(b[i])
            for j in range(len(b[i])):
                b[i][j] -= row_min
        for i in range(len(b[0])):
            col_min = np.min(b[:, i])
            for j in range(len(b)):
                b[j][i] -= col_min
        line_count = 0
        # 线数目小于矩阵长度时，进行循环
        while (line_count < len(b)):
            line_count = 0
            row_zero_count = []
            col_zero_count = []
            for i in range(len(b)):
                row_zero_count.append(np.sum(b[i] == 0))
            for i in range(len(b[0])):
                col_zero_count.append((np.sum(b[:, i] == 0)))
            # 划线的顺序（分行或列）
            line_order = []
            row_or_col = []
            for i in range(len(b[0]), 0, -1):
                while (i in row_zero_count):
                    line_order.append(row_zero_count.index(i))
                    row_or_col.append(0)
                    row_zero_count[row_zero_count.index(i)] = 0
                while (i in col_zero_count):
                    line_order.append(col_zero_count.index(i))
                    row_or_col.append(1)
                    col_zero_count[col_zero_count.index(i)] = 0
            # 画线覆盖0，并得到行减最小值，列加最小值后的矩阵
            delete_count_of_row = []
            delete_count_of_rol = []
            row_and_col = [i for i in range(len(b))]
            for i in range(len(line_order)):
                if row_or_col[i] == 0:
                    delete_count_of_row.append(line_order[i])
                else:
                    delete_count_of_rol.append(line_order[i])
                c = np.delete(b, delete_count_of_row, axis=0)
                c = np.delete(c, delete_count_of_rol, axis=1)
                line_count = len(delete_count_of_row) + len(delete_count_of_rol)
                # 线数目等于矩阵长度时，跳出
                if line_count == len(b):
                    break
                # 判断是否画线覆盖所有0，若覆盖，进行加减操作
                if 0 not in c:
                    row_sub = list(set(row_and_col) - set(delete_count_of_row))
                    min_value = np.min(c)
                    for i in row_sub:
                        b[i] = b[i] - min_value
                    for i in delete_count_of_rol:
                        b[:, i] = b[:, i] + min_value
                    break
        row_ind, col_ind = linear_sum_assignment(b)
        min_cost = task_matrix[row_ind, col_ind].sum()
        best_solution = list(task_matrix[row_ind, col_ind])
        return min_cost, best_solution, row_ind, col_ind
