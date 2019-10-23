from vehicular_topology import SingleCell
from vehicular_device import *
from dqn_reforcement import Ye_Li_DQN


if __name__ == '__main__':
    random_slot_num = 1000
    train_slot_num = 1000
    test_slot_num = 1000
    train_num = 1
    test_num = train_num

    all_slot_num = random_slot_num + train_slot_num * train_num + test_slot_num * test_num

    cue_num = 10
    d2d_num = 20
    rb_num = cue_num + 1
    up_or_down_link = 'up'
    d_tx2rx = 20  # m
    highway = Highway(0, 0, 500, 4 * 3.5)  # 生成高速公路对象，赋值起始位置

    single_cell = SingleCell(cue_num, d2d_num, rb_num, up_or_down_link, d_tx2rx, highway)

    single_cell.initial()

    agent = Ye_Li_DQN(2*rb_num + 8, rb_num)
    # agent = Ye_Li_DQN(2 * rb_num + 3, rb_num*power_level_num)
    # agent.load('./weights/dqn_3000' + '.pkl')

    for slot in range(all_slot_num):
        print("********************循环次数: ", slot, " ********************")
        if slot < random_slot_num:
            single_cell.clear_allocated_rb()
            single_cell.dqn_train_work(slot, agent)
            single_cell.update(slot)
        else:
            if 0 <= (slot - random_slot_num) % (train_slot_num + test_slot_num) < train_slot_num:  # train
                single_cell.clear_allocated_rb()
                single_cell.dqn_train_work(slot, agent)
                # single_cell.update(slot)
                # if (slot+1) % (train_slot_num+test_slot_num) == 0:
                #     agent.save('./weights/dqn_' + str(slot+1))
            else:  # test
                single_cell.clear_allocated_rb()
                single_cell.dqn_test_work(slot, agent)  # DQN
                single_cell.update((slot - random_slot_num) % (train_slot_num + test_slot_num) - random_slot_num)

            if (slot - random_slot_num + 1) % (train_slot_num + test_slot_num) == 0:
                # single_cell.save_data(str(d2d_num) + '_' +
                #                       str(int(train_slot_num * (slot - random_slot_num + 1)
                #                               / (train_slot_num + test_slot_num))) + '_dqn')
                single_cell.capacity(test_slot_num,
                                     'log_' + str(d2d_num) + '_' +
                                     str(int(train_slot_num * (slot - random_slot_num + 1)
                                             / (train_slot_num + test_slot_num))) + '_dqn')

                single_cell.result('slot2result_' + str(d2d_num) + '_dqn')