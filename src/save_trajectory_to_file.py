import datetime
def save_trajectory_to_file(trajectory):
    """
    Save trajectory to file
    Args:
        trajectory: list of lists, there should be 6 lists in the list, each list contains the joint angles of the joint
    """
    filename = f'trajectory_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.txt'
    # if trajectory is 
    with open(filename, 'w') as f:
        for joint in trajectory:
            for angle in joint:
                f.write(f'{angle},')
            f.write(';')

if __name__ == '__main__':
    trajectory = [
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], # trajectory of joint 1
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], # trajectory of joint 2
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], # trajectory of joint 3
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], # trajectory of joint 4
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], # trajectory of joint 5
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], # trajectory of joint 6
    ]
    save_trajectory_to_file(trajectory)