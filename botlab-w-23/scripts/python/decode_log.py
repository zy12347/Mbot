import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import odometry_t, mbot_encoder_t,pose_xyt_t

enc2meters = ((2.0 * 3.14159 * .041) / (78 * 20))
WHEEL_BASE = 0.161
dt = 1/50
def speeds(right_delta, left_delta):
    measured_vel_r = enc2meters*right_delta/dt
    measured_vel_l = enc2meters*left_delta/dt
    
    measured_vel_fwd = 0.5*(measured_vel_r + measured_vel_l)
    measured_vel_turn = 1/WHEEL_BASE*(measured_vel_r - measured_vel_l)
    return measured_vel_fwd, measured_vel_turn
if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1],"r")

data = np.empty((0,4), dtype=float)
enc_data = np.empty((0,3), dtype=float)
slam = np.empty((0,4), dtype=float)
init = 0
for event in log:
    if event.channel == "ODOMETRY":
        msg = odometry_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        data = np.append(data, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)
    if event.channel == "SLAM_POSE":
        msg = pose_xyt_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        slam = np.append(slam, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)
    if event.channel == "MBOT_ENCODERS":
        msg = mbot_encoder_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        enc_data = np.append(enc_data, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.right_delta, \
            msg.left_delta
            ]]), axis=0)

vel_fwd, vel_turn = speeds(enc_data[:,1], enc_data[:,2])

#fig, (ax1, ax2) = plt.subplots(2)
#print(enc_data[:,0].shape)
#plt.plot(data[:,0], data[:,1]-slam[:,0], 'r')
#plt.set_xlabel("Time [s]")
#plt.set_ylabel("Error")
#ax1.set_title("Drive Square Forward Speed")

#ax2.plot(enc_data[:,0], vel_turn, 'r')
#ax2.set_xlabel("Time [s]")
#ax2.set_ylabel("Angular Velocity [rad/s]")
#ax2.set_title("Drive Square Angular Speed")

#plt.show()
np.savetxt("./odometry.csv",data,"%1.5f",",")
np.savetxt("./slam.csv",slam,"%1.5f",",")