import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # ← AGREGAR ESTA LÍNEA

data = np.loadtxt('ur5_log_1.txt', skiprows=1)

plt.rcParams.update({'font.size': 14}) # Establece el tamaño de fuente global a 12
# leer datos de un archivo .txt con format ###  ###  ###, ###  ###  ###, ###





t = data[:,0]
# Comandos y actuales
q_cmd = data[:,1:7]
q_cur = data[:,7:13]

# Pose base (constante)
p_des = data[:,13:16]
q_des = data[:,16:20]
# Trayectoria deseada
p_traj = data[:,20:23]
q_traj = data[:,23:27]
# Actual
p_act = data[:,27:30]
q_act = data[:,30:34]
# Errores
pos_error = data[:,34:37]
ori_error = data[:,37:40]
v_cart_actual = data[:,40:46]
v_cart_desired = data[:,46:52]
tau = data[:,52:58]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(p_act[:,0], p_act[:,1], p_act[:,2], label='Trayectoria Actual', color='g')
ax.plot(p_traj[:,0], p_traj[:,1], p_traj[:,2], label='Trayectoria Deseada', color='b', linestyle='--')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Trayectoria de UR5')
ax.legend()
ax.grid(True)
plt.show()



plt.figure(figsize=(12, 10)) # Adjust figure size for multiple subplots

for i in range(6): # Loop through each of the 6 joints
    plt.subplot(3, 2, i + 1) # Create a 3x2 grid of subplots
    plt.plot(t, q_cmd[:, i], label=f'q_cmd[{i+1}]') # Plot commanded joint position
    plt.plot(t, q_cur[:, i], '--', label=f'q_cur[{i+1}]') # Plot actual joint position
    plt.ylabel('Posición (rad)') # Joint positions are typically in radians
    plt.legend()
    plt.grid()
    plt.title(f'Articulación {i+1}: Deseada vs Actual')

plt.tight_layout() # Adjust layout to prevent overlapping titles/labels
plt.show()

plt.figure(figsize=(12,6))
plt.subplot(3,1,1)
plt.plot(t, p_traj[:,0], label='x $^{des}$')
plt.plot(t, p_act[:,0], '--', label='x')
plt.ylabel('Posición (m)')
plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
plt.title('Trayectoria deseada vs actual')
plt.grid()

plt.subplot(3,1,2)
plt.plot(t, p_traj[:,1], label='y $^{des}$')
plt.plot(t, p_act[:,1], '--', label='y')
plt.ylabel('Posición (m)')
plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
plt.grid()

plt.subplot(3,1,3)
plt.plot(t, p_traj[:,2], label='z $^{des}$')
plt.plot(t, p_act[:,2], '--', label='z')
plt.ylabel('Posición (m)')
plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
plt.grid()


# plt.subplot(2,1,2)
# plt.plot(t, pos_error[:,0], label='Error x')
# plt.plot(t, pos_error[:,1], label='Error y')
# plt.plot(t, pos_error[:,2], label='Error z')
# plt.ylabel('Error (m)')
# plt.xlabel('Tiempo (s)')
# plt.legend()
# plt.grid()
# plt.title('Error cartesiano')

# plt.tight_layout()
plt.show()

plt.figure()
plt.plot(t, ori_error)
plt.xlabel('Tiempo (s)')
plt.ylabel('Error de orientación (rad)')
plt.title('Error de orientación (log3)')
plt.legend(['x','y','z'])
plt.grid()
plt.show()


plt.figure(figsize=(10,7))
plt.subplot(2,1,1)
plt.plot(t, v_cart_actual[:,0], label='Vx actual')
plt.plot(t, v_cart_desired[:,0], '--', label='Vx deseada')
plt.plot(t, v_cart_actual[:,1], label='Vy actual')
plt.plot(t, v_cart_desired[:,1], '--', label='Vy deseada')
plt.plot(t, v_cart_actual[:,2], label='Vz actual')
plt.plot(t, v_cart_desired[:,2], '--', label='Vz deseada')
plt.ylabel('Velocidad (m/s)')
plt.legend()
plt.grid()
plt.title('Velocidades cartesianas: Actual vs Deseada')

plt.subplot(2,1,2)
plt.plot(t, v_cart_actual[:,3], label='Wx actual')
plt.plot(t, v_cart_desired[:,3], '--', label='Wx deseada')
plt.plot(t, v_cart_actual[:,4], label='Wy actual')
plt.plot(t, v_cart_desired[:,4], '--', label='Wy deseada')
plt.plot(t, v_cart_actual[:,5], label='Wz actual')
plt.plot(t, v_cart_desired[:,5], '--', label='Wz deseada')
plt.ylabel('Velocidad angular (rad/s)')
plt.xlabel('Tiempo (s)')
plt.legend()
plt.grid()
plt.title('Velocidades angulares cartesianas')
plt.tight_layout()
plt.show()

plt.figure()
plt.plot(t, tau)
plt.xlabel('Tiempo (s)')
plt.ylabel('Torque (N.m)')
plt.title('ley de control tau')
plt.legend(['u1','u2','u3','u4','u5','u6'])
plt.grid()
plt.show()

print("Error min absoluto", min(abs(p_act[:,0] - p_traj[:,0]))," ", min(abs(p_act[:,1] - p_traj[:,1])), " ", min(abs(p_act[:,2] - p_traj[:,2])))
print("Error max absoluto", max(abs(p_act[:,0] - p_traj[:,0]))," ", max(abs(p_act[:,1] - p_traj[:,1])), " ", max(abs(p_act[:,2] - p_traj[:,2])))
#error promedio
print("Error promedio", np.mean(abs(p_act[:,0] - p_traj[:,0]))," ", np.mean(abs(p_act[:,1] - p_traj[:,1])), " ", np.mean(abs(p_act[:,2] - p_traj[:,2])))  