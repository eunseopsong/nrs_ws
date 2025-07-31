# 데이터를 읽어서 그래프 그리기 코드 예제 ..................
import matplotlib.pyplot as plt
import numpy as np

# Load the data
data=np.loadtxt('Data1.txt')

# Alocate the data
Data0=data[:,0] # time
Data1=data[:,1] # Fx
Data2=data[:,2] # Fy
Data3=data[:,3] # Fz
Data4=data[:,4] # Mx
Data5=data[:,5] # My
Data6=data[:,6] # Mz

# Plot the data
plt.figure(num=1,dpi=100,facecolor='white')

plt.plot(Data0,Data1,'k-',label="Fx")
plt.plot(Data0,Data2,'b-',label="Fy")
plt.plot(Data0,Data3,'r-',label="Fz")

plt.title('Force')
plt.xlabel('time(s)')
plt.ylabel('Force(N)')
plt.grid()

plt.figure(num=2,dpi=100,facecolor='white')

plt.plot(Data0,Data4,'k-',label="Mx")
plt.plot(Data0,Data5,'b-',label="My")
plt.plot(Data0,Data6,'r-',label="Mz")

plt.title('Moment')
plt.xlabel('time(s)')
plt.ylabel('Moment(N)')
plt.grid()





plt.show()
