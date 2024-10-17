import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

g = 9.81
L = 1.0

t_start = 0.0
t_end = 20.0
dt = 0.01
num_steps = int((t_end-t_start)/dt)

theta0 = np.pi/4
omega0 = 0.0

def derivatives(theta, omega):
    dtheta_dt = omega
    domega_dt = (-g/L)* np.sin(theta)
    return dtheta_dt, domega_dt

def rk4(theta, omega):
    k1_theta = omega
    k1_omega = derivatives(theta, omega)[1]

    k2_theta = omega + 0.5 * dt * k1_omega
    k2_omega = derivatives(theta + 0.5 *dt* k1_theta, omega + 0.5 * dt * k1_omega)[1]

    k3_theta = omega + 0.5 * dt * k2_omega
    k3_omega = derivatives(theta + 0.5 * dt * k2_theta, omega + 0.5 * dt * k2_omega)[1]

    k4_theta = omega + dt* k3_omega
    k4_omega = derivatives(theta + dt * k3_theta, omega + dt * k3_omega)[1]

    theta_new = theta + (dt/6) * (k1_theta + 2*k2_theta + 2*k3_theta + k4_theta)
    omega_new = omega + (dt/6) * (k1_omega + 2*k2_omega + 2*k3_omega + k4_omega)

    return theta_new, omega_new

theta_values = [theta0]
omega_values = [omega0]

for _ in range(num_steps):
    theta_new, omega_new = rk4(theta_values[-1], omega_values[-1])

    theta_values.append(theta_new)
    omega_values.append(omega_new)
print(theta_values)

fig, ax = plt.subplots()
ax.set_xlim(-1.5 *L, 1.5*L)
ax.set_ylim(-2*L, 0)
ax.grid(True)
line, = ax.plot([],[],'o-', lw =2)

def update(frame):
    x = L *np.sin(theta_values[frame])
    y = -L*np.cos(theta_values[frame])
    line.set_data([0,x], [0,y])
    return line,

ani = animation.FuncAnimation(fig, update, frames=num_steps, interval=dt*1000, blit=True)

plt.show()