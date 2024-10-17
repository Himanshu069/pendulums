import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

theta2_dot_dot =0
def f(y,t):

    theta1, theta2, theta1_dot, theta2_dot = y
    g = 9.81  # acceleration due to gravity
    m1 = 1.0  # mass of pendulum bob 1
    m2 = 1.0  # mass of pendulum bob 2
    l1 = 1.0  # length of pendulum arm 1
    l2 = 1.0  # length of pendulum arm 2
    theta1_dot_dot= (m2 * math.pow(theta1,2)*np.sin(theta2-theta1)*np.cos(theta2-theta1) +\
        m2 * g/l1 * np.sin(theta2) + \
        m2*l2/l1*theta2_dot*np.sin(theta2-theta1)-\
        g/l1 * np.sin(theta1)*(m1+m2))/(m1+m2-m2*math.pow(np.cos(theta2-theta1),2))

    theta2_dot_dot = (-m2*math.pow(theta2,2)*np.sin(theta2-theta1)*np.cos(theta2-theta1) +\
        g/l2 * np.sin(theta1) * np.cos(theta2-theta1)*(m1+m2) \
        -l1/l2 * math.pow(theta1,2)*np.sin(theta2-theta1)*(m1+m2) \
        -g/l2 * np.sin(theta2)*(m1+m2))/(m1+m2-m2*math.pow(np.cos(theta2-theta1),2))
        

    return [theta1_dot, theta2_dot, theta1_dot_dot, theta2_dot_dot]

def rk4(y, t , dt):
    k1 = np.array(f(y,t))
    k2 = np.array(f(y+ 0.5 * dt*k1,t+ 0.5*dt))
    k3 = np.array(f(y+0.5 * dt *k2,t+0.5*dt))
    k4= np.array(f(y+ dt*k3, t+dt))

    return y + dt * (k1 + 2*k2 + 2*k3 + k4)/6

#initial conditions
theta1_0=math.pi/2+0.05
theta2_0=math.pi/2
theta1_dot_0=0.0
theta2_dot_0=0.0

t_start = 0.0
t_end = 20.0
dt = 0.01
num_steps = int((t_end-t_start)/dt)
time = np.arange(t_start, t_end, dt)

theta1_values=[]
theta2_values=[]
theta1_dot_values=[]
theta2_dot_values=[]
y = np.array([theta1_0, theta2_0, theta1_dot_0, theta2_dot_0])
for t in time:
    theta1_values.append(y[0])
    theta2_values.append(y[1])
    theta1_dot_values.append(y[2])
    theta2_dot_values.append(y[3])
    y=rk4(y,t,dt)

print(theta1_dot_values)

l1 = 1.0  # length of pendulum arm 1
l2 = 1.0 

fig, ax = plt.subplots()

line, = ax.plot([], [], lw=2)

def init():
    ax.set_xlim(-4,4)
    ax.set_ylim(-4,4)
    return line,
path_x = []
path_y = []
def update(frame):
    x1 = l1 * np.sin(theta1_values[frame])
    y1 = -l1 * np.cos(theta1_values[frame])
    x2 = l1 * np.sin(theta1_values[frame]) + l2 * np.sin(theta2_values[frame])
    y2 = -l1 * np.cos(theta1_values[frame]) - l2 *np.cos(theta2_values[frame])
    line.set_data([0,x1,x2],[0,y1,y2])
    path_x.append(x2)
    path_y.append(y2)
    #line.set_data(path_x, path_y)
    return line,    

ani = FuncAnimation(fig, update, frames=num_steps, init_func=init, interval=dt*1000, blit=True)

plt.show()
