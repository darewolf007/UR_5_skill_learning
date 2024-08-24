import numpy as np
import os
import pickle

def find_transition_indices(arr, from_value, to_value):
    transition_mask = (arr[:-1] == from_value) & (arr[1:] == to_value)
    transition_indices = np.where(transition_mask)[0]
    print("transition_indices", transition_indices)
    print("transition_indices", transition_indices + 4)
    transition_indices = transition_indices + 4
    if transition_indices[-1] > arr.shape[0] -1:
        transition_indices[-1] = arr.shape[0]
    print("transition_indices", transition_indices)
    return transition_indices

def write_pickle(file_name, traj_env_data, traj_joint_action_data, traj_endeffector_action_data, gripper_data, Task):
    
    np_traj_env_data = np.copy(traj_env_data[:-1])
    np_traj_gripper_state_data = np.copy(gripper_data[:-1])
    
    next_np_traj_joint_action_data = np.copy(traj_joint_action_data[1:])
    next_np_traj_endeffector_action_data = np.copy(traj_endeffector_action_data[1:])
    now_np_traj_joint_action_data = np.copy(traj_joint_action_data[:-1])
    now_np_traj_endeffector_action_data = np.copy(traj_endeffector_action_data[:-1])
    
    np_traj_joint_action_data = np.copy(traj_joint_action_data[:-1])
    np_traj_endeffector_action_data = np.copy(traj_endeffector_action_data[:-1])
    np_traj_joint_action_data[:,:-1] = next_np_traj_joint_action_data[:,:-1] - now_np_traj_joint_action_data[:,:-1]
    np_traj_endeffector_action_data[:,:-1] = next_np_traj_endeffector_action_data[:,:-1] - now_np_traj_endeffector_action_data[:,:-1]
    print("np_traj_env_data", np_traj_env_data[10])
    print("np_traj_env_data", np_traj_env_data[11])
    print("np_traj_action_data", np_traj_joint_action_data[10])
    
    traj_pose_dict = {}
    sub_task_dict = {}
    begin_task_index = 0
    begin_task = 0
    task_split_index = find_transition_indices(np_traj_gripper_state_data,1,0)
    traj_pose_dict ['joint_actions'] = np_traj_joint_action_data
    traj_pose_dict ['endeffector_actions'] = np_traj_endeffector_action_data
    traj_pose_dict ['env_state'] = np_traj_env_data
    for idx in range(len(task_split_index)):
        Trans_task = str(begin_task) + '-' + str(Task[idx])
        sub_task_dict[Trans_task] = {
            'begin_task_idx': begin_task_index,
            'done_task_idx': task_split_index[idx],
            'done_task': Task[idx],
            'begin_task': begin_task
        }
        begin_task_index = task_split_index[idx]
        begin_task = Task[idx]
    traj_pose_dict ['subtask_info'] = sub_task_dict
    print("traj_pose_dict", traj_pose_dict['subtask_info'])
    with open(file_name, 'wb') as f:
        pickle.dump(traj_pose_dict, f)
        

def get_joint_traj(data):
    now_gripper = np.array([data[0]])
    ur_endeffector_position = data[1:8]
    ur_joint_angle = data[8:14]
    env_state = data[14:]
    env_traj_point = np.concatenate((ur_joint_angle, now_gripper, env_state))
    joint_action_traj_point = np.concatenate((ur_joint_angle, now_gripper))
    endeffector_action_traj_point = np.concatenate((ur_endeffector_position, now_gripper))
    return env_traj_point, joint_action_traj_point, endeffector_action_traj_point

def split_part_traj(traj_data, split_begin, split_end, split_step):
    return traj_data[split_begin:split_end:split_step]

def check_duplicates(index_list):
    return len(index_list) != len(set(index_list))

def sample_traj(traj_data, traj_gripper_data):
    open_to_close = list(find_transition_indices(traj_gripper_data,0,1))
    close_to_open = list(find_transition_indices(traj_gripper_data,1,0))
    gripper_state_change = sorted(open_to_close+close_to_open)
    split_traj_path1_data_index = []
    split_traj_path2_data_index = []
    begin_split_index = 0
    save_split_index = 3
    gripper_change_time = 0
    traj_index = range(traj_data.shape[0])
    sample_flag = True
    if len(gripper_state_change) != 8:
        print("gripper state wrong!!!!!!!!!!!!!!!!!!!!!!!!")
        sample_flag = False
    gripper_state_change.append(traj_data.shape[0])
    for index_index in range(len(gripper_state_change) - 1):
        # TODO need to add min max done but not test?
        index = gripper_state_change[index_index]
        # max_index = min(gripper_state_change[index_index+1] - save_split_index, traj_data.shape[0])
        max_index = traj_data.shape[0]
        left_index = max(index - save_split_index, begin_split_index)
        right_index = min(index + save_split_index+1, max_index)
        save_part = traj_index[left_index:right_index]
        if gripper_change_time%2 == 0:
            split_step = 4
        else:
            split_step = 2
        split_part_1 = split_part_traj(traj_index, begin_split_index, left_index, split_step)
        split_part_2 = split_part_traj(traj_index, begin_split_index + split_step/2, left_index, split_step)
        split_traj_path1_data_index += list(split_part_1 + save_part)
        split_traj_path2_data_index += list(split_part_2 + save_part)
        begin_split_index = right_index
        gripper_change_time+=1
    if check_duplicates(split_traj_path1_data_index) or check_duplicates(split_traj_path2_data_index):
        print("wrong!!!!!!!!!!!!!!!!!!!!!!!!")
        sample_flag = False
    print(gripper_state_change)
    print(split_traj_path1_data_index)
    print(split_traj_path2_data_index)
    print("********************")
    print(len(split_traj_path1_data_index))
    print(len(split_traj_path2_data_index))
    print("********************")
    return split_traj_path1_data_index, split_traj_path2_data_index, sample_flag


def read_npy_files_in_order(directory):
    files = [f for f in os.listdir(directory) if f.endswith('.npy')]
    files.sort(key=lambda x: int(os.path.splitext(x)[0].split("_")[1]))
    traj_env_data = []
    traj_joint_action_data = []
    traj_endeffector_action_data = []
    traj_gripper_state_data = []
    for file in files:
        filepath = os.path.join(directory, file)
        data = np.load(filepath)
        gripper_state = data[0]
        env_traj_point, joint_action_traj_point, endeffector_action_traj_point = get_joint_traj(data)
        traj_env_data.append(env_traj_point)
        traj_joint_action_data.append(joint_action_traj_point)
        traj_endeffector_action_data.append(endeffector_action_traj_point)
        traj_gripper_state_data.append(gripper_state)
    tmp_traj_env_data = np.array(traj_env_data[3:])
    tmp_traj_joint_action_data = np.array(traj_joint_action_data[3:])
    tmp_traj_endeffector_action_data = np.array(traj_endeffector_action_data[3:])
    tmp_traj_gripper_state_data = np.array(traj_gripper_state_data[3:])
    # print(np_traj_env_data.shape)
    # print(np_traj_joint_action_data.shape)
    # print(np_traj_endeffector_action_data.shape)
    # print(np_traj_env_data[0])
    # print(np_traj_joint_action_data[0])
    # print(np_traj_endeffector_action_data[0])
    np_traj_env_data = np.copy(tmp_traj_env_data)
    np_traj_gripper_state_data = np.copy(tmp_traj_gripper_state_data)
    np_traj_joint_action_data = np.copy(tmp_traj_joint_action_data)
    np_traj_endeffector_action_data = np.copy(tmp_traj_endeffector_action_data)
    
    print("np_traj_env_data", np_traj_env_data[10])
    print("np_traj_env_data", np_traj_env_data[11])
    print("np_traj_action_data", np_traj_joint_action_data[10])
    print(np_traj_env_data.shape)
    print(np_traj_joint_action_data.shape)
    print(np_traj_endeffector_action_data.shape)
    # print(np_traj_env_data[0])
    # print(np_traj_joint_action_data[0])
    # print(np_traj_endeffector_action_data[0])
    task_split_index = find_transition_indices(np_traj_gripper_state_data,1,0)
    print(task_split_index)
    split_traj_path1_data_index, split_traj_path2_data_index, sample_flag = sample_traj(np_traj_env_data, np_traj_gripper_state_data)
    if not sample_flag:
        return False
    split_traj_env_data_1 = np.copy(np_traj_env_data[split_traj_path1_data_index])
    split_traj_env_data_2 = np.copy(np_traj_env_data[split_traj_path2_data_index])
    split_traj_gripper_data_1 = np.copy(np_traj_gripper_state_data[split_traj_path1_data_index])
    split_traj_gripper_data_2 = np.copy(np_traj_gripper_state_data[split_traj_path2_data_index])
    split_traj_joint_action_data_1 = np.copy(np_traj_joint_action_data[split_traj_path1_data_index])
    split_traj_joint_action_data_2 = np.copy(np_traj_joint_action_data[split_traj_path2_data_index])
    split_traj_endeffector_action_data_1 = np.copy(np_traj_endeffector_action_data[split_traj_path1_data_index])
    split_traj_endeffector_action_data_2 = np.copy(np_traj_endeffector_action_data[split_traj_path2_data_index])
    
    
    traj_num = directory.split("/")[6]
    print("traj_num", traj_num)
    base_path = "/home/yhx/shw_dataset/ABCD_dataset/"
    ABCD_path = base_path + 'ABCD/'
    ABC_path = base_path + 'ABC/'
    BCD_path = base_path + 'BCD/'
    ABCD_all_path = ABCD_path + 'all/'
    ABCD_sample_path = ABCD_path + 'sample/'
    ABC_all_path = ABC_path + 'all/'
    ABC_sample_path = ABC_path + 'sample/'
    BCD_all_path = BCD_path + 'all/'
    BCD_sample_path = BCD_path + 'sample/'
    print("ABCD")
    ABCD_all_filename = ABCD_all_path + 'full_abcd_' + str(traj_num) + '.pkl'
    ABCD_sample_filename_1 = ABCD_sample_path + 'sample_abcd_1_' + str(traj_num) + '.pkl'
    ABCD_sample_filename_2 = ABCD_sample_path + 'sample_abcd_2_' + str(traj_num) + '.pkl'

    write_pickle(ABCD_all_filename, np_traj_env_data, np_traj_joint_action_data, np_traj_endeffector_action_data, np_traj_gripper_state_data, Task = [1,2,3,4])
    write_pickle(ABCD_sample_filename_1, split_traj_env_data_1, split_traj_joint_action_data_1, split_traj_endeffector_action_data_1, split_traj_gripper_data_1, Task = [1,2,3,4])
    write_pickle(ABCD_sample_filename_2, split_traj_env_data_2, split_traj_joint_action_data_2, split_traj_endeffector_action_data_2, split_traj_gripper_data_2, Task = [1,2,3,4])
    print("ABC")
    ABC_all_filename = ABC_all_path + 'full_abc_' + str(traj_num) + '.pkl'
    ABC_sample_filename_1 = ABC_sample_path + 'sample_abc_1_' + str(traj_num) + '.pkl'
    ABC_sample_filename_2 = ABC_sample_path + 'sample_abc_2_' + str(traj_num) + '.pkl'
    abc_all_task_index = find_transition_indices(np_traj_gripper_state_data,1,0)[2]+3
    abc_sample_task_index_1 = find_transition_indices(split_traj_gripper_data_1,1,0)[2]+3
    abc_sample_task_index_2 = find_transition_indices(split_traj_gripper_data_2,1,0)[2]+3

    write_pickle(ABC_all_filename, np_traj_env_data[:abc_all_task_index,:], np_traj_joint_action_data[:abc_all_task_index,:], np_traj_endeffector_action_data[:abc_all_task_index,:], np_traj_gripper_state_data[:abc_all_task_index], Task = [1,2,3])
    write_pickle(ABC_sample_filename_1, split_traj_env_data_1[:abc_sample_task_index_1,:], split_traj_joint_action_data_1[:abc_sample_task_index_1,:], split_traj_endeffector_action_data_1[:abc_sample_task_index_1,:], split_traj_gripper_data_1[:abc_sample_task_index_1], Task = [1,2,3])
    write_pickle(ABC_sample_filename_2, split_traj_env_data_2[:abc_sample_task_index_2,:], split_traj_joint_action_data_2[:abc_sample_task_index_2,:], split_traj_endeffector_action_data_2[:abc_sample_task_index_2,:], split_traj_gripper_data_2[:abc_sample_task_index_2], Task = [1,2,3])
    print("BCD")
    BCD_all_filename = BCD_all_path + 'full_bcd_' + str(traj_num) + '.pkl'
    BCD_sample_filename_1 = BCD_sample_path + 'sample_bcd_1_' + str(traj_num) + '.pkl'
    BCD_sample_filename_2 = BCD_sample_path + 'sample_bcd_2_' + str(traj_num) + '.pkl'
    bcd_all_task_index = find_transition_indices(np_traj_gripper_state_data,1,0)[0]+2
    bcd_sample_task_index_1 = find_transition_indices(split_traj_gripper_data_1,1,0)[0]+2
    bcd_sample_task_index_2 = find_transition_indices(split_traj_gripper_data_2,1,0)[0]+2
    write_pickle(BCD_all_filename, np_traj_env_data[bcd_all_task_index:,:], np_traj_joint_action_data[bcd_all_task_index:,:], np_traj_endeffector_action_data[bcd_all_task_index:,:], np_traj_gripper_state_data[bcd_all_task_index:], Task = [2,3,4])
    write_pickle(BCD_sample_filename_1, split_traj_env_data_1[bcd_sample_task_index_1:,:], split_traj_joint_action_data_1[bcd_sample_task_index_1:,:], split_traj_endeffector_action_data_1[bcd_sample_task_index_1:,:], split_traj_gripper_data_1[bcd_sample_task_index_1:], Task = [2,3,4])
    write_pickle(BCD_sample_filename_2, split_traj_env_data_2[bcd_sample_task_index_2:,:], split_traj_joint_action_data_2[bcd_sample_task_index_2:,:], split_traj_endeffector_action_data_2[bcd_sample_task_index_2:,:], split_traj_gripper_data_2[bcd_sample_task_index_2:], Task = [2,3,4])
    return True
if __name__  == "__main__":
    # read_npy_files_in_order("/home/yhx/shw/src/Dataset_Collection/5/traj")
    for i in range(5, 88):
        test_path = "/home/yhx/shw/src/Dataset_Collection/" + str(i) +"/traj"
        read_npy_files_in_order(test_path)