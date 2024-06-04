import time
import MPU6050 
import math

mpu = MPU6050.MPU6050(a_bus=0)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)


# Calibration
for i in range(10):
    print("Calibrating MPU6050. Please wait...")
    calibration_samples = 1000
    x_gyro_offset = 0
    y_gyro_offset = 0
    z_gyro_offset = 0
    x_accel_offset = 0
    y_accel_offset = 0
    z_accel_offset = 0

    for i in range(calibration_samples):
        rotation = mpu.get_rotation()
        acceleration = mpu.get_acceleration()
        x_gyro_offset += rotation[0]
        y_gyro_offset += rotation[1]
        z_gyro_offset += rotation[2]
        x_accel_offset += acceleration[0]
        y_accel_offset += acceleration[1]
        z_accel_offset += acceleration[2]
        time.sleep(0.005)  # short delay before next reading

    x_gyro_offset = (x_gyro_offset / calibration_samples) * -1
    y_gyro_offset = (y_gyro_offset / calibration_samples) * -1
    z_gyro_offset = (z_gyro_offset / calibration_samples) * -1
    x_accel_offset = (x_accel_offset / calibration_samples) * -1
    y_accel_offset = (y_accel_offset / calibration_samples) * -1
    z_accel_offset = (z_accel_offset / calibration_samples) * -1

    mpu.set_x_gyro_offset(int(x_gyro_offset))
    mpu.set_y_gyro_offset(int(y_gyro_offset))
    mpu.set_z_gyro_offset(int(z_gyro_offset))
    mpu.set_x_accel_offset(int(x_accel_offset))
    mpu.set_y_accel_offset(int(y_accel_offset))
    mpu.set_z_accel_offset(int(z_accel_offset))

    print("Calibration complete. Gyro Offsets: {0}, {1}, {2}".format(mpu.get_x_gyro_offset_TC(), mpu.get_y_gyro_offset(), mpu.get_z_gyro_offset_TC()))
    print("Calibration complete. Gyro Offsets: {0}, {1}, {2}".format(x_gyro_offset, y_gyro_offset, z_gyro_offset))
    print("Calibration complete. Accel Offsets: {0}, {1}, {2}".format(x_accel_offset, y_accel_offset, z_accel_offset))

while True:
    # Assuming you have already initialized and calibrated your MPU6050 as 'mpu'
    # Get FIFO buffer data
    fifo_count = mpu.get_FIFO_count()
    if fifo_count < 42:
        continue
    fifo_buffer = mpu.get_FIFO_bytes(fifo_count)
    
    # Now you can get quaternion data
    quat = mpu.DMP_get_quaternion(fifo_buffer)
    
    upright_quat = MPU6050.Q(quat.w, quat.x, quat.y, quat.z)
    
    # Get gravity vector
    grav = mpu.DMP_get_gravity(upright_quat)
    
    if grav.x == 0 or grav.y == 0 or grav.z == 0:
        continue
    
    upright_grav = MPU6050.V(grav.x, grav.z, grav.y)
    
    # Now you can get roll, pitch, yaw
    # rotation = mpu.DMP_get_euler_roll_pitch_yaw(upright_quat, upright_grav)
    
    rotation = mpu.get_rotation()
    print("Roll: {0:.2f}, Pitch: {1:.2f}, Yaw: {2:.2f}".format(rotation[0], rotation[1], rotation[2]))
    
    # Adjust axes for upright robot
    # upright_rotation = MPU6050.V(rotation.x, rotation.z, rotation.y)
    
    # print("Roll: {0:.2f}, Pitch: {1:.2f}, Yaw: {2:.2f}".format(rotation.x, rotation.y, rotation.z))