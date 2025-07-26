import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt


def leer_haptic(ruta):
    posiciones = []
    orientaciones = []
    with open(ruta, 'r') as f:
        for linea in f:
            partes = linea.split()
            # Asegúrate de que la línea tiene la estructura esperada
            if len(partes) >= 12:
                #print(float(partes[11]))
                pos = [float(partes[2]), float(partes[3]), float(partes[4])]
                ori = [float(partes[len(partes)-4]), float(partes[len(partes)-3]), float(partes[len(partes)-2]), float(partes[len(partes)-1])]
                posiciones.append(pos)
                orientaciones.append(ori)
    return posiciones, orientaciones
def leer_joints(filename):
    joints = []
    with open(filename, 'r') as f:
        for linea in f:
            if linea.startswith("Joint Positions:"):
                partes = linea.strip().split()
                joint = [float(x) for x in partes[len(partes)-6:len(partes)]]
                joints.append(joint)
    return np.array(joints)
def leer_joints_ur(filename):
    joints = []
    with open(filename, 'r') as f:
        for linea in f:
            if linea.startswith("Joint Positions ur:"):
                partes = linea.strip().split()
                joint = [float(x) for x in partes[len(partes)-6:len(partes)]]
                joints.append(joint)
    return np.array(joints)


ruta_base = '/home/david/workspaces/ur_gz/src/ur_simulation_gz/ur_simulation_gz/launch/'

haptic_pos, haptic_ori = leer_haptic(ruta_base + 'output_data.txt')
joint_pos = leer_joints_ur(ruta_base + 'output_data3.txt')
print(joint_pos)
#joint_pos_ur5 = leer_joints_ur5(ruta_base + 'output_data3.txt')



# Cinemática directa con Pinocchio
urdf_path = '/home/david/workspaces/ur_gz/install/ur_simulation_gz/share/ur_simulation_gz/launch/ur5.urdf'
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# Cambia el nombre del frame del end-effector según tu URDF
ee_frame = 'tool0'  # Por ejemplo: 'tool0', 'ee_link', etc.
ee_id = model.getFrameId(ee_frame)
if ee_id == -1: 
    raise ValueError(f"El frame '{ee_frame}' no se encuentra en el modelo URDF.")



xyz = []
for q in joint_pos:
    q_pin = np.zeros(model.nq)
    q_pin[:len(q)] = q  # Copia los 6 valores de las articulaciones del UR5
    pin.forwardKinematics(model, data, q_pin)
    pin.updateFramePlacements(model, data)
    pos = data.oMf[ee_id].translation.copy()  # ¡Importante copiar!
    xyz.append(pos)
print(len(xyz))
xyz = np.array(xyz)[0:len(xyz),:]  # Limitar a los primeros 500 puntos
# Graficar trayectoria en X, Y, Z (puedes elegir X y Z)
haptic_pos = np.array(haptic_pos)[0:len(xyz),:]  # Limitar a los primeros 500 puntos
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xyz[:,0], xyz[:,1], xyz[:,2], label='Trayectoria EE UR5', color='g')
ax.plot(-haptic_pos[:,0]-0.1152, 0.493-haptic_pos[:,1]+0.0881142, haptic_pos[:,2]+0.0655108+0.293, label='Trayectoria Haptic', color='r', linestyle='--')
ax.set_xlabel('X[m]')
ax.set_ylabel('Y[m]')
ax.set_zlabel('Z[m]')
ax.set_title('Trayectorias 3D: UR5 vs Haptic')
ax.legend()
ax.grid(True)
plt.show()         

""" # vector tiempo de 50 ms en 50 ms
fig2 = plt.figure()
t = np.arange(0, len(haptic_pos[:,0])*0.010, 0.010)
plt.plot(t, -(xyz[:,0]-(-haptic_pos[:,0]-0.1152))*1000, label='Error en eje X', color='r')
plt.xlabel('Tiempo [s]')
plt.ylabel('Diferencia X[mm]')
plt.title('Diferencia en X [mm] vs Tiempo')
plt.legend()
plt.grid(True) 
plt.show()

fig3 = plt.figure()
t = np.arange(0, len(haptic_pos)*0.05, 0.05)
plt.plot(t, -(xyz[:,1]-(0.493-haptic_pos[:,1]+0.0881142))*1000, label='Error en eje Y', color='r')
plt.xlabel('Tiempo [s]')
plt.ylabel('Diferencia Y[mm]')
plt.title('Diferencia en Y [mm] vs Tiempo')
plt.legend()
plt.grid(True) 
plt.show()

fig4 = plt.figure()
t = np.arange(0, len(haptic_pos)*0.05, 0.05)
plt.plot(t, -(xyz[:,2]-(haptic_pos[:,2]+0.0655108+0.293))*1000, label='Error en eje Z', color='r')
plt.xlabel('Tiempo [s]')
plt.ylabel('Diferencia Z[mm]')
plt.title('Diferencia en Z [mm] vs Tiempo')
plt.legend()
plt.grid(True) 
plt.show()  """
haptic_x = -haptic_pos[:,0] - 0.1152
haptic_y = 0.493 - haptic_pos[:,1] + 0.0881142
haptic_z = haptic_pos[:,2] + 0.0655108 + 0.293

# Ejes UR5
ur5_x = xyz[:,0]
ur5_y = xyz[:,1]
ur5_z = xyz[:,2]

# Vector de tiempo
t = np.arange(0, len(haptic_x)*0.01, 0.01)

# --- GRAFICA X ---
fig_x, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 5))
ax1.plot(t, ur5_x*1000, label='UR5 X', color='g')
ax1.plot(t, haptic_x*1000, label='Haptic X', color='r', linestyle='--')
ax1.set_xlabel('Tiempo [s]')
ax1.set_ylabel('Posición X [mm]')
ax1.set_title('Posición X')
ax1.legend()
ax1.grid(True)

error_x = (ur5_x - haptic_x)*1000  # en mm
ax2.plot(t, error_x, label='Error X', color='b')
ax2.set_xlabel('Tiempo [s]')
ax2.set_ylabel('Error X [mm]')
ax2.set_title('Error en X')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()

# --- GRAFICA Y ---
fig_y, (ay1, ay2) = plt.subplots(2, 1, figsize=(12, 5))
ay1.plot(t, ur5_y*1000, label='UR5 Y', color='g')
ay1.plot(t, haptic_y*1000, label='Haptic Y', color='r', linestyle='--')
ay1.set_xlabel('Tiempo [s]')
ay1.set_ylabel('Posición Y [mm]')
ay1.set_title('Posición Y')
ay1.legend()
ay1.grid(True)

error_y = (ur5_y - haptic_y)*1000  # en mm
ay2.plot(t, error_y, label='Error Y', color='b')
ay2.set_xlabel('Tiempo [s]')
ay2.set_ylabel('Error Y [mm]')
ay2.set_title('Error en Y')
ay2.legend()
ay2.grid(True)

plt.tight_layout()
plt.show()

# --- GRAFICA Z ---
fig_z, (az1, az2) = plt.subplots(2, 1, figsize=(12, 5))
az1.plot(t, ur5_z*1000, label='UR5 Z', color='g')
az1.plot(t, haptic_z*1000, label='Haptic Z', color='r', linestyle='--')
az1.set_xlabel('Tiempo [s]')
az1.set_ylabel('Posición Z [mm]')
az1.set_title('Posición Z')
az1.legend()
az1.grid(True)

error_z = (ur5_z - haptic_z)*1000  # en mm
az2.plot(t, error_z, label='Error Z', color='b')
az2.set_xlabel('Tiempo [s]')
az2.set_ylabel('Error Z [mm]')
az2.set_title('Error en Z')
az2.legend()
az2.grid(True)

plt.tight_layout()
plt.show()