import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # ← AGREGAR ESTA LÍNEA

plt.rcParams.update({'font.size': 14})
data = np.loadtxt('/home/david/tesis_ws/src/ur5_simulation/impednacia_circunferencia/output_data_impedancia.txt')
x = data[:, 3]
y = data[:, 4]
z = data[:, 5]
xdes = data[:, 0]
ydes = data[:, 1]
zdes = data[:, 2]
t = np.arange(len(x)) / 500
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='Trayectoria de UR5', color='g')
ax.plot(xdes, ydes, zdes,'--', label='Trayectoria Deseada', color='b')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Trayectoria de UR5')
ax.legend()
ax.grid(True)
plt.show()







# graficas de seguimiento a la trayectoria en los ejes x, y, z
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t,x, label='x')
plt.plot(t,xdes, '--', label='x deseada')
plt.ylabel('Posición (m)')
plt.xlabel('Tiempo (s)')
plt.legend()
plt.grid()      
plt.subplot(3, 1, 2)
plt.plot(t,y, label='y')
plt.plot(t,ydes, '--', label='y deseada')
plt.ylabel('Posición (m)')
plt.xlabel('Tiempo (s)')
plt.legend()
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(t,z, label='z')
plt.plot(t,zdes, '--', label='z deseada')
plt.ylabel('Posición (m)')
plt.xlabel('Tiempo (s)')
plt.legend()
plt.grid()
plt.show()

data2 = np.loadtxt('/home/david/tesis_ws/src/ur5_simulation/impednacia_circunferencia/output_data_esfuerzo.txt')
u1 = data2[:, 0]
u2 = data2[:, 1]
u3 = data2[:, 2]
u4 = data2[:, 3]
u5 = data2[:, 4]
u6 = data2[:, 5]
plt.figure(figsize=(10, 6))
plt.subplot(3, 2, 1)
plt.plot(t, u1, label='Esfuerzo U1')
plt.xlabel('Tiempo (s)')    
plt.ylabel('Esfuerzo (Nm)')
plt.axis('tight')  # Ajusta los ejes para que tengan el mismo rango
plt.xlim([0, max(t)])  # Establece el límite del eje x
plt.title('Gráfica de los esfuerzos U1')
plt.grid(True)  
plt.subplot(3, 2, 2)
plt.plot(t, u2, label='Esfuerzo U2')
plt.xlabel('Tiempo (s)')    
plt.ylabel('Esfuerzo (Nm)')
plt.axis('tight')  # Ajusta los ejes para que tengan el mismo rango
plt.xlim([0, max(t)])  # Establece el límite del eje x
plt.title('Gráfica de los esfuerzos U2')
plt.grid(True) 
plt.subplot(3, 2, 3)
plt.plot(t, u3, label='Esfuerzo U3')
plt.xlabel('Tiempo (s)')    
plt.ylabel('Esfuerzo (Nm)')
plt.axis('tight')  # Ajusta los ejes para que tengan el mismo rango
plt.xlim([0, max(t)])  # Establece el límite del eje x
plt.title('Gráfica de los esfuerzos U3')
plt.grid(True) 
plt.subplot(3, 2, 4)
plt.plot(t, u4, label='Esfuerzo U4')
plt.xlabel('Tiempo (s)')    
plt.ylabel('Esfuerzo (Nm)')
plt.axis('tight')  # Ajusta los ejes para que tengan el mismo rango
plt.xlim([0, max(t)])  # Establece el límite del eje x
plt.title('Gráfica de los esfuerzos U4')
plt.grid(True) 
plt.subplot(3, 2, 5)
plt.plot(t, u5, label='Esfuerzo U5')
plt.xlabel('Tiempo (s)')    
plt.ylabel('Esfuerzo (Nm)')
plt.axis('tight')  # Ajusta los ejes para que tengan el mismo rango
plt.xlim([0, max(t)])  # Establece el límite del eje x
plt.title('Gráfica de los esfuerzos U5')
plt.grid(True) 
plt.subplot(3, 2, 6)
plt.plot(t, u6, label='Esfuerzo U6')
plt.xlabel('Tiempo (s)')    
plt.ylabel('Esfuerzo (Nm)')
plt.axis('tight')  # Ajusta los ejes para que tengan el mismo rango
plt.xlim([0, max(t)])  # Establece el límite del eje x
plt.title('Gráfica de los esfuerzos U6')
plt.grid(True)    
plt.tight_layout()
plt.show() 


print("Error min absoluto", min(abs(x - xdes))," ", min(abs(y - ydes)), " ", min(abs(z - zdes)))
print("Error max absoluto", max(abs(x - xdes))," ", max(abs(y - ydes)), " ", max(abs(z - zdes)))
#error promedio
print("Error promedio", np.mean(abs(x - xdes))," ", np.mean(abs(y - ydes)), " ", np.mean(abs(z - zdes)))  