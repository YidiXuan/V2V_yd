#!/usr/bin/python
# -*- coding:utf8 -*-

from vehicular_topology import *
from Resource_management_by_YeLi import *

if __name__ == '__main__':
    slot_num = 200  # 循环次数

    cue_num = 10
    d2d_num = 20
    rb_num = cue_num + 1
    up_or_down_link = 'up'
    d_tx2rx = 20  # m
    highway = Highway(0, 0, 500, 4*3.5)  # 生成高速公路对象，赋值起始位置

    single_cell = SingleCell(cue_num, d2d_num, rb_num, up_or_down_link, d_tx2rx, highway)
    single_cell.initial()

    for slot in range(slot_num):
        print("********************循环次数: ", slot, " ********************")
        # single_cell.graph_spectrum_allocation_icc_work()
        # single_cell.resource_fair_graph()
        single_cell.graph_spectrum_allocation_work()
        # single_cell.random_spectrum_allocation_work()
        # single_cell.resource_coalitional_games_work()
        #single_cell.resource_YeLi_work()


        single_cell.update_location_slot(slot)  # 每1ms更新一次车辆位置


    single_cell.save_data()
    # single_cell.save_data_game()
    # single_cell.save_data_full_mmwave()
    #single_cell.save_data_full_YL()
    # single_cell.save_data_fair()




