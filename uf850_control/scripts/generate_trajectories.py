import numpy as np

plane_surface = np.array([0,-0.017647, -0.348606])  #x-coef, y-coef, d-coef
block_x_pos = np.array([0.1644, 0.9144, 1.6644])
block_y_pos = np.array([[-0.725,-0.555],[0.555,0.725]])
block_width = 0.113
y_knot_pos = np.array([[-1.088,-0.898],[-0.918,-0.580],[-0.6,0.01],[-0.01,0.678],[0.658,0.827]])
x_end = 1.8288

squeegee_width = 0.15
overlap = 0.025

pan_cleaning_trajectories = np.array([[]])
pan_cleaning_modes = []

squeegee_x_pos = []
temp_x_pos = squeegee_width / 2
while True:
    squeegee_x_pos.append(temp_x_pos)
    temp_x_pos = temp_x_pos + squeegee_width - overlap
    if temp_x_pos + squeegee_width / 2 > x_end:
        temp_x_pos = x_end - squeegee_width / 2
        break

print(squeegee_x_pos)
print(len(squeegee_x_pos))

i = 0
while i < len(squeegee_x_pos):
    for j in range(block_x_pos.shape[0]):
        if abs(block_x_pos[j] - squeegee_x_pos[i]) < (squeegee_width + block_width) / 2:
            new_x_pos = [block_x_pos[j] - (squeegee_width + block_width) / 2, block_x_pos[j], block_x_pos[j] + (squeegee_width + block_width) / 2]
            squeegee_x_pos = squeegee_x_pos[:i] + new_x_pos + squeegee_x_pos[i+2:]
            i = i+2
            break
    i = i+1
    

print(squeegee_x_pos)
print(len(squeegee_x_pos))

pan_cleaning_trajectories = []

for i in range(len(squeegee_x_pos)):
    trajectory = np.array([[squeegee_x_pos[i], y_knot_pos[0,0], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[0,0] + plane_surface[2]],
                             [squeegee_x_pos[i], y_knot_pos[0,1], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[0,1] + plane_surface[2]]])
    pan_cleaning_trajectories.append(trajectory)

for i in range(len(squeegee_x_pos)):
    block_found = False
    for j in range(block_x_pos.shape[0]):
        if(abs(squeegee_x_pos[i] - block_x_pos) < squeegee_width / 2):
            block_found = True
        if(abs(squeegee_x_pos[i] - block_x_pos) > squeegee_width / 2):
            # pull away from edge
            trajectory = np.array([[squeegee_x_pos[i], y_knot_pos[0,0], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[0,0] + plane_surface[2]],
                             [squeegee_x_pos[i], y_knot_pos[0,1], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[0,1] + plane_surface[2]]])
            pan_cleaning_trajectories.append(trajectory)
            pan_cleaning_modes.append(0)
            # push towards first rail and under
            trajectory = np.array([[squeegee_x_pos[i], y_knot_pos[1,0], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[1,0] + plane_surface[2]],
                             [squeegee_x_pos[i], y_knot_pos[1,1], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[1,1] + plane_surface[2]]])
            pan_cleaning_trajectories.append(trajectory)
            pan_cleaning_modes.append(1)
            # pull away from first rail
            trajectory = np.array([[squeegee_x_pos[i], y_knot_pos[2,0], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[2,0] + plane_surface[2]],
                             [squeegee_x_pos[i], y_knot_pos[2,1], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[2,1] + plane_surface[2]]])
            pan_cleaning_trajectories.append(trajectory)
            pan_cleaning_modes.append(0)
            # push towards second rail and under
            trajectory = np.array([[squeegee_x_pos[i], y_knot_pos[3,0], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[3,0] + plane_surface[2]],
                             [squeegee_x_pos[i], y_knot_pos[3,1], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[3,1] + plane_surface[2]]])
            pan_cleaning_trajectories.append(trajectory)
            pan_cleaning_modes.append(1)
            # pull away from second rail
            trajectory = np.array([[squeegee_x_pos[i], y_knot_pos[4,0], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[4,0] + plane_surface[2]],
                             [squeegee_x_pos[i], y_knot_pos[4,1], plane_surface[0] * squeegee_x_pos[i] + plane_surface[1] * y_knot_pos[4,1] + plane_surface[2]]])
            pan_cleaning_trajectories.append(trajectory)
            pan_cleaning_modes.append(0)
        else:
            pass
    if not block_found:
        pass