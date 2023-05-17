import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import odometry_t,pose_xyt_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: decode_log.py <logfile>")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1],"r")

data_true = np.empty((0,4), dtype=float)
data_slam = np.empty((0,4), dtype=float)
init = 0
for event in log:
    #print(event.channel)
    # if event.channel == "ODOMETRY":
    #     msg = odometry_t.decode(event.data)
    #     if init==0:
    #         start_utime = msg.utime
    #         init = 1
    #     data = np.append(data, np.array([[ \
    #         (msg.utime-start_utime)/1.0E6, \
    #         msg.x, \
    #         msg.y, \
    #         msg.theta
    #         ]]), axis=0)
    if event.channel == "TRUE_POSE":
        msg = pose_xyt_t.decode(event.data)
        if init==0:
            start_utime = msg.utime
            init = 1
        data_true = np.append(data_true, np.array([[ \
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
        data_slam = np.append(data_slam, np.array([[ \
            (msg.utime-start_utime)/1.0E6, \
            msg.x, \
            msg.y, \
            msg.theta
            ]]), axis=0)

# plt.plot(data_true[:,1], data_true[:,2], 'r')
# plt.savefig("true.png")

# plt.plot(data_slam[:,1], data_slam[:,2], 'r')
# plt.savefig("slam.png")

# print(len(data_true),data_true)
# print(len(data_slam),data_slam)

def distance(x1,y1,x2,y2):
    return ((x1-x2)**2+(y1-y2)**2)

start =0
end = 0
true_list = []
slam_list = []
while end<len(data_true):
    if(data_true[end,0]<int(data_true[start,0])+1):
        end = end+1
    else:
        a = np.mean(data_true[start:end,:],axis=0)
        true_list.append(a)
        start = end

print(len(true_list),true_list)
true_array = np.array(true_list)
start=0
end=0
while end<len(data_slam):
    if(data_slam[end,0]<int(data_slam[start,0])+1):
        end = end+1
    else:
        a = np.mean(data_slam[start:end,:],axis=0)
        slam_list.append(a)
        start = end

print("\n")
print(len(slam_list),slam_list)
slam_array = np.array(slam_list)[1:]
# plt.scatter(true_array[:,1],true_array[:,2])
# plt.scatter(slam_array[:,1],slam_array[:,2])
dist = []
for i in range(len(slam_array)):
    dist.append(distance(slam_array[i,0],slam_array[i,1],true_array[i,0],true_array[i,1]))

plt.title("Distance between Slam pose and True Pose")
plt.xlabel("pose")
plt.ylabel("distance")
plt.plot(range(len(dist)),dist)
plt.show()
print(np.mean(dist)**0.5)
# for i in range(13,32):
        
#     dis=[]
#     for i in range(len(data_slam)):
#         for j in range((len(data_true))):
#                 dis.append(distance(data_slam[i,1],data_slam[i,2],data_true[j,1],data_true[j,2]))
# dis = np.array(dis)
# np.savetxt("a.csv",data_slam)
# np.savetxt("b.csv",data_true)

# res = np.sort(dis)
# res = res[0:len(data_slam)]

# plt.title("RMS")
# plt.xlabel("pose number")
# plt.ylabel("Error")
# plt.plot(range(len(data_slam)),res, 'r')
# print(np.sum(res))
# plt.savefig("slam.png")