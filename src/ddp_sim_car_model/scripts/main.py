from simple_car_dynamics import SimpleCarDynamics
from ddp import Ddp
from lqr_cost import LQRCost
import matplotlib.pyplot as plt
import numpy as np
import cPickle as pickle
import math

def main():
	# np.set_printoptions(precision=3, suppress=True)	
	dynamics = SimpleCarDynamics()
	N = 100
	tf = 35.0
	dt = tf/N
	ts = np.arange(N+1)*dt
	xt = np.zeros((101,5));
	xd1 = pickle.load(open("../../scarab_sim/control_signal/data_debug/traject.pkl", "rb" ))
	xt[:,0:2] = xd1
	print(xt.shape)

	us0 = np.zeros([N, dynamics.m])

	# Trajectory info
	# xt[0,2] = 0
	# xt[0,3] = 1
	# xt[0,4] = 0 
	# for i in range(N):
	# 	xt[i+1,2] = math.atan2(xt[i+1,1]-xt[i,1], xt[i+1,0] - xt[i+1,0])
	# 	xt[i+1,3] = 1
	# 	xt[i+1,4] =0

	# xtt = np.zeros((51,5));
	# xtt = xt[0::2]
	# print(xtt.shape)

	Q = dt*np.eye(dynamics.n)* 1000
	Q[2,2] = 1
	Q[3,3] = 1
	Q[4,4] = 1
	R = 0.01*dt*np.eye(dynamics.m)*100000
	Qf = 1000*np.eye(dynamics.n)
	Qf[-1,-1] = 1

	###########################
	## zero trajectory
	# trject = np.zeros((5,51))
	# for i in range(25):
	#     trject[0][i] = i/4.0
	#     trject[1][i] = 0

	# for i in range(6):
	#     trject[0][i+25] = 6
	#     trject[1][i+25] = -(i+1.0)/4

	# for i in range(20):
	#     trject[0][i+31] = 6.25 + i/4.0
	#     trject[1][i+31] = -1.5
	# trject = np.transpose(trject)

	# xt = trject
	# print(type(xt))
	############################
	## first trajectory
	# trject = np.zeros((5,51))
	# for i in range(17):
	#     trject[0][i] = i/4.0
	#     trject[1][i] = 0

	# for i in range(6):
	#     trject[0][i+17] = 4
	#     trject[1][i+17] = -(i+1.0)/4

	# for i in range(28):
	#     trject[0][i+23] = 4.25 + i/4.0
	#     trject[1][i+23] = -1.5
	# trject = np.transpose(trject)
	# xt = trject

	# xt = np.zeros([N+1,5])
	# xk = np.linspace(0, 5, N+1, endpoint=False)
	# for i in range(N+1):
	# 	xt[i,0] = xk[i]
	# 	xt[i,1] = 0.5 *xk[i]**2 + 0.05
	# 	xt[i,2] = 0.
	# 	xt[i,3] = 0.
	# 	xt[i,4] = 0.

	xd = xt
	cost = LQRCost(N, Q, R, Qf, xd)
	max_step = 10.0  # Allowed step for control
	x0 = np.array([0., 0., 0, 0, 0])



	ddp = Ddp(dynamics, cost, us0, x0, dt, max_step)
	# print(ddp.xs.shape)
	V = ddp.V
	for i in range(1000):
		ddp.iterate()
		V = ddp.V
		print(ddp.xs[:,0:2])
		
	us_total = ddp.us
	xs_total = ddp.xs
	pickle.dump(us_total, open("data_us_f.p", "wb"))
	pickle.dump(xs_total, open("data_xs_f.p", "wb"))

	#########################################################

	plt.figure(1)
	plt.plot(ddp.xs[:, 0], ddp.xs[:, 1])
	plt.plot(xt[:, 0], xt[:, 1])
	# plt.plot(xt[:, 0], xt[:, 1], 'k')
	plt.plot(xd[-1][0], xd[-1][1], 'r*')

	plt.xlabel('x (m)')
	plt.ylabel('y (m)')
	plt.figure(2)
	plt.subplot(2,1,1)
	plt.plot(ts[:-1], ddp.us[:, 0])
	plt.ylabel('Accelertion (m/ss)')
	plt.subplot(2,1,2)
	plt.plot(ts[:-1], ddp.us[:, 1])
	plt.ylabel('Steering rate (rad/s)')
	plt.figure(3)
	plt.subplot(2,1,1)
	plt.plot(ts, ddp.xs[:, 3])
	plt.ylabel('Velocity (m/s)')
	plt.subplot(2,1,2)
	plt.plot(ts, ddp.xs[:, 4])
	plt.ylabel('Steering angle (rad)')
	plt.show()

if __name__ == '__main__':
	main()