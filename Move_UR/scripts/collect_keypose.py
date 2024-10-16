import re
import subprocess

def read_end_pose():
    output = subprocess.Popen(["rostopic", "echo", "-n", "1", "/tf"], stdout=subprocess.PIPE).stdout.read().decode()
    output = output.strip().split("\n")
    x = output[11].strip().split()[1]
    y = output[12].strip().split()[1]
    z = output[13].strip().split()[1]
    rx = output[15].strip().split()[1]
    ry = output[16].strip().split()[1]
    rz = output[17].strip().split()[1]
    rw = output[18].strip().split()[1]
    return "[" + ", ".join([x, y, z, rx, ry, rz, rw]) + "],"

if __name__ == "__main__":
    traj = []
    traj_length = 0
    while True:
        x = input()
        if x == "e":
            break
        pose = read_end_pose()
        traj_length += 1
        print(traj_length, pose)
        traj.append(pose)
    traj_str = "[" + "\n".join(traj) + "]"
    with open("traj.txt", "w") as f:
        f.write(traj_str)