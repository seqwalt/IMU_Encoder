import csv
import sys
import numpy as np

if len(sys.argv) != 2:
    print('Usage: python3 compute_calib.py path/to/imu_data_directory')
    exit()

dir = sys.argv[1]

# Load recorded IMU data
# Each row: ax,ay,az,wx,wy,wz
reader = csv.reader(open(dir + "/x_up.txt"), delimiter=",")
x_up = np.array(list(reader)).astype("float")
reader = csv.reader(open(dir + "/x_down.txt"), delimiter=",")
x_dn = np.array(list(reader)).astype("float")
reader = csv.reader(open(dir + "/y_up.txt"), delimiter=",")
y_up = np.array(list(reader)).astype("float")
reader = csv.reader(open(dir + "/y_down.txt"), delimiter=",")
y_dn = np.array(list(reader)).astype("float")
reader = csv.reader(open(dir + "/z_up.txt"), delimiter=",")
z_up = np.array(list(reader)).astype("float")
reader = csv.reader(open(dir + "/z_down.txt"), delimiter=",")
z_dn = np.array(list(reader)).astype("float")

# ------ Acceleration Calibration ------ #

# Get accleration averages
x_up_avg = np.hstack((np.mean(x_up[:,:3],0), 1)).reshape(1,-1)
x_dn_avg = np.hstack((np.mean(x_dn[:,:3],0), 1)).reshape(1,-1)
y_up_avg = np.hstack((np.mean(y_up[:,:3],0), 1)).reshape(1,-1)
y_dn_avg = np.hstack((np.mean(y_dn[:,:3],0), 1)).reshape(1,-1)
z_up_avg = np.hstack((np.mean(z_up[:,:3],0), 1)).reshape(1,-1)
z_dn_avg = np.hstack((np.mean(z_dn[:,:3],0), 1)).reshape(1,-1)

# True acceleration values
x_up_true = np.array((9.81, 0, 0)).reshape(1,-1)
x_dn_true = np.array((-9.81, 0, 0)).reshape(1,-1)
y_up_true = np.array((0, 9.81, 0)).reshape(1,-1)
y_dn_true = np.array((0, -9.81, 0)).reshape(1,-1)
z_up_true = np.array((0, 0, 9.81)).reshape(1,-1)
z_dn_true = np.array((0, 0, -9.81)).reshape(1,-1)

# Least squares method
# want to estimate X, where Y = W*X
W = np.vstack((x_up_avg, x_dn_avg, y_up_avg, y_dn_avg, z_up_avg, z_dn_avg))
W_pinv = np.linalg.pinv(W) # W_pinv = (W.T * W)^-1 * W.T
Y = np.vstack((x_up_true, x_dn_true, y_up_true, y_dn_true, z_up_true, z_dn_true))
X_est = W_pinv @ Y

# Using notation from https://www.intelrealsense.com/wp-content/uploads/2019/07/Intel_RealSense_Depth_D435i_IMU_Calibration.pdf
# accel_est = SC * accel_raw - bias
SC = X_est[0:3,:].T
bias = -X_est[3,:].reshape(3,1)

print("Accelerometer scale-factor/off-axis matrix:")
print('SC =')
print(str(SC[0,0]) +","+ str(SC[0,1]) +","+ str(SC[0,2]) +","+ str(SC[1,0]) +","+ str(SC[1,1]) +","+ str(SC[1,2]) +","+ str(SC[2,0]) +","+ str(SC[2,1]) +","+ str(SC[2,2]))
print()
print("Accelerometer bias:")
print('ba =')
print(str(bias[0][0]) +","+ str(bias[1][0]) +","+ str(bias[2][0]))
print()

# ------ Gyroscope Calibration ------ #
gyro_bias = np.mean(x_dn[:,3:],0).reshape(3,1)
print("Gyroscope bias:")
print('bg =')
print(str(gyro_bias[0][0]) +","+ str(gyro_bias[1][0]) +","+ str(gyro_bias[2][0]))

print()
print('Apply the calibration:')
print('acc_est = SC * acc_raw - ba')
print('gyr_est = gyr_raw - bg')
print()
