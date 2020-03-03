import ikpy
import numpy as np
import math
from ikpy import plot_utils
import matplotlib.pyplot as plt
def RPYtoMatrix(roll,pitch,yaw):

      Rz = np.zeros((3,3),dtype=float)
      Ry = np.zeros((3,3),dtype=float)
      Rx = np.zeros((3,3),dtype=float)

      Rz[0,0] = math.cos(roll)
      Rz[0,1] = -math.sin(roll)
      Rz[1,0] = math.sin(roll)
      Rz[1,1] = math.cos(roll)
      Rz[2,2] = 1

      Ry[0,0] = math.cos(pitch)
      Ry[0,2] = math.sin(pitch)
      Ry[2,0] = -math.sin(pitch)
      Ry[2,2] = math.cos(pitch)
      Ry[1,1] = 1
 
      Rx[1,1] = math.cos(yaw)
      Rx[1,2] = -math.sin(yaw)
      Rx[2,1] = math.sin(yaw)
      Rx[2,2] = math.cos(yaw)
      Rx[0,0] = 1

      R = np.matmul(np.matmul(Rz,Ry),Rx)
      return R
def PathRotationMatrix(start,end,N):
    R_arr = []
    s = np.linspace(0,1,N)
    for i in range(N):
       R_arr.append(start+s[i]*(end-start))
    return R_arr
def PathPosition(start,end,N):
    Pos_arr = []
    s = np.linspace(0,1,N)
    for i in range(N):
       Pos_arr.append(start+s[i]*(end-start))
    return Pos_arr

my_chain = ikpy.chain.Chain.from_urdf_file("resources/open_manipulator_p.urdf")
#my_chain = ikpy.chain.Chain.from_urdf_file("../resources/poppy_ergo.URDF")
print(my_chain)
N = 200
R_arr = PathRotationMatrix(RPYtoMatrix(0,0,0),RPYtoMatrix(0,math.pi/4,0),N)
Pos_arr = PathPosition(np.array([0.55,0.0,0.45]),np.array([0.3,0.0,0.4]),N)
ax = plot_utils.init_3d_figure()
plt.ion()
for i in range(N):
   plt.cla()
   target_vector = Pos_arr[i]
   target_frame = np.eye(4)
   target_frame[:3, 3] = target_vector
   target_frame[:3,:3] = R_arr[i]
   print("The angles of each joints are : ", my_chain.inverse_kinematics(target_frame))
   real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
   print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))

   # If there is a matplotlib error, uncomment the next line, and comment the line below it.
   # %matplotlib inline

   my_chain.plot(my_chain.inverse_kinematics(target_frame), ax, target=target_vector)
   plt.xlim(-1, 1)
   plt.ylim(-1, 1)
   plt.show()
   plt.pause(0.01)
