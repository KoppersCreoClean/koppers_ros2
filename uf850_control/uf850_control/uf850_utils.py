def ik_uf850(self, target_position, target_rotation):
    position = target_position

    q = np.zeros(6)

    position = position - target_rotation[:,2] * 0.09 - np.array([0,0,0.364])

    q[0] = np.arctan2(position[1],position[0])
    r = np.sqrt(position[0]**2 + position[1]**2)
    a1 = 0.39
    a2 = np.sqrt(0.15**2 + 0.426**2)
    q1_offset =  - np.pi/2
    q2_offset = -(np.pi/2 + np.arctan(0.426/0.15))
    q2 = -np.arccos((r**2 + position[2]**2 - a1**2 - a2**2)/(2*a1*a2))
    q1 = np.arctan2(position[2],r) - np.arctan2(a2*np.sin(q2),(a1 + a2*np.cos(q2)))
    q[1] = q1 + q1_offset
    q[2] = q2_offset - q2

    raw_rotation = rotz(q[0]) @ roty(q[2]-q[1]) @ np.diag([1,-1,-1])

    R_diff = raw_rotation.T @ target_rotation

    if(abs(np.arctan2(R_diff[1,2], R_diff[0,2])) < abs(np.arctan2(-R_diff[1,2], -R_diff[0,2]))):
        q[3] = np.arctan2(R_diff[1,2], R_diff[0,2])
        q[4] = np.arctan2(np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
        q[5] = np.arctan2(R_diff[2,1], -R_diff[2,0])
    else:
        q[3] = np.arctan2(-R_diff[1,2], -R_diff[0,2])
        q[4] = np.arctan2(-np.sqrt(R_diff[1,2]**2 + R_diff[0,2]**2), R_diff[2,2])
        q[5] = np.arctan2(-R_diff[2,1], R_diff[2,0])

    return q

# #TODO: probably a better way than just putting the DH parameters here
# dh = [[0, 0.364, 0, np.pi/2.0],
#     [np.pi/2.0, 0, 0.39, np.pi],
#     [np.pi/2.0, 0, 0.15, -np.pi/2.0],
#     [0, 0.426, 0, -np.pi/2.0],
#     [0, 0, 0, np.pi/2.0],
#     [0, 0.09, 0, 0]]

# self.arm = SerialArm(dh)