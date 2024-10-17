import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PIDController:
    def __init__(self,Kp,Ki,Kd,setpoint,dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt

        self.prev_error = 0
        self.integral = 0
    
    def update(self,feedback_value):
        error = self.setpoint - feedback_value
        self.integral += error * self.dt
        derivative = (error - self.prev_error)/ self.dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        return output
    
def f(y,u,t):
    x, x_dot, phi, phi_dot = y
    M = 0.5
    m = 0.2
    b = 0.1
    l = 0.3
    I = 0.006
    g = 9.81
    q = I*(M+m) + M*m*math.pow(l,2)
    x_dot_dot = -((I + m*math.pow(l,2))*b*x_dot)/q + \
                (math.pow(m,2)*math.pow(l,2)*g*phi)/q + \
                (I+m*math.pow(l,2))*u/q
    phi_dot_dot = - (m*l*b*x_dot)/q + \
                    m*g*l*(M+m)*phi/q + \
                    m*l*u/q
    
    return [x_dot, x_dot_dot, phi_dot, phi_dot_dot]

def rk4(y,u,t,dt):
    k1 = np.array(f(y,u,t))
    k2 = np.array(f(y + 0.5 * dt * k1,u, t + 0.5*dt))
    k3 = np.array(f(y + 0.5 *dt * k2,u, t + 0.5*dt))
    k4 = np.array(f(y +  dt*k3,u, t + dt))

    return y + dt * (k1 + 2*k2 + 2*k3 + k4)/6

x_0 = 0.0
phi_0 = np.pi/3
x_dot_0 = 0.0
phi_dot_0 = 0.0

t_start = 0.0
t_end = 20.0
dt = 0.01
num_steps = int((t_end-t_start)/dt)
time = np.arange(t_start,t_end,dt)

x_values = []
phi_values = []
x_dot_values = []
phi_dot_values = []
y = np.array([x_0, x_dot_0, phi_0, phi_dot_0])
Kp = 100
Ki = 3
Kd = 5
F = 10
pid = PIDController(Kp, Ki, Kd, phi_dot_0, dt)

for t in time:
    u =  pid.update(y[2])
    x_values.append(y[0])
    x_dot_values.append(y[1])
    phi_values.append(y[2])
    phi_dot_values.append(y[3])
    y = rk4(y,u,t,dt)

# Plot and animation setup
fig, ax = plt.subplots()
ax.set_xlim(-4, 4)
ax.set_ylim(0, 3)
ax.set_xlabel('Position')
ax.set_ylabel('Height')
ax.set_title('Inverted Pendulum on a Cart')

line, = ax.plot([], [], 'k-', lw=2)
cart, = ax.plot([], [], 'ro', markersize=10)
pendulum, = ax.plot([], [], 'bo', markersize=15)

l = 0.9  # Length of the pendulum

def init():
    line.set_data([], [])
    cart.set_data([], [])
    pendulum.set_data([], [])
    return line, cart, pendulum

def update(frames):
    # Get the current position of the cart and pendulum
    x_cart = x_values[frames]
    phi = phi_values[frames]

    # Update the cart's position
    cart_width = 0.4
    cart_height = 0.2
    cart_x = [x_cart - cart_width/2, x_cart + cart_width/2, x_cart + cart_width/2, x_cart - cart_width/2]
    cart_y = [-cart_height/2, -cart_height/2, cart_height/2, cart_height/2]

    # Pendulum's end position
    pendulum_x = x_cart + l * np.sin(phi)
    pendulum_y = l * np.cos(phi)
    
    # Update line (pendulum)
    line.set_data([x_cart, pendulum_x], [0, pendulum_y])
    # Update cart and pendulum positions
    cart.set_data([x_cart], [0])
    pendulum.set_data([pendulum_x], [pendulum_y])
    return line, cart, pendulum

ani = FuncAnimation(fig, update, frames=num_steps, init_func=init, interval=dt*1000, blit=True)
plt.show()
