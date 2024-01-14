y = 10 #-640 - 640
desired_distance_to_block = 10

error = desired_distance_to_block - y
desired_distance_wall = 50 - error * 0.1

print(desired_distance_wall)

if desired_distance_wall > 50:
    desired_distance_wall = 100 - desired_distance_wall
    print(desired_distance_wall)

distancey = 100

100 - distancey * 0.1