from termios import VEOL
import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt

m =  np.array([0, 1, 1, 4, 7, 7, 8, 8, 7, 4, 1, 0, 0, 0, 1, 1, 4, 7, 7, 8, 8, 7, 4, 1, 0, 0,
               0 ,0 ,6 ,0 ,6 ,0 ,0 ,8 ,8 ,2 ,8 ,8 ,0 ,0, 0, 6, 0, 6, 0, 0, 8, 8, 2, 8, 8, 0,
               0, 0, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
               1, 1, 1, 1, 1, 1 ,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
m = m.reshape(4,26)
velo_to_cam = np.array([7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
                        1.480249e-02,  7.280733e-04, -9.998902e-01, -7.631618e-02,
                        9.998621e-01,  7.523790e-03,  1.480755e-02, -2.717806e-01,
                        0,   0,   0 ,  1])
velo_to_cam = velo_to_cam.reshape(4,4)

fig = plt.figure()
ax1 = fig.add_subplot(221, projection='3d')
ax1.plot3D(m[0,:], m[1,:], m[2,:], 'gray')

# yaw 
the = 90 * np.pi/180
Rx1 = np.array([np.cos(the), -np.sin(the), 0, 0,
                  np.sin(the),  np.cos(the), 0, 0, 
                  0,                      0, 1 ,0,
                  0,                      0, 0, 1]);
Rx1 = Rx1.reshape(4,4)          
# pitch
the = 0 *np.pi/180
Rx2 = np.array([ np.cos(the),  0, np.sin(the), 0,
                 0,            1,           0, 0,
                 -np.sin(the), 0, np.cos(the), 0, 
                 0,            0,           0, 1]);      
Rx2 = Rx2.reshape(4,4) 
# roll
the = 90 * np.pi/180
Rx3 = np.array([1,                      0, 0, 0,
                  0, np.cos(the), -np.sin(the), 0,
                  0, np.sin(the),  np.cos(the), 0, 
                  0,                      0, 0, 1]);
Rx3 = Rx3.reshape(4,4)         
# linear                 
Rxn = np.array([1, 0, 0, 0.27,
                  0, 1, 0, 0.06,
                  0, 0, 1, -0.08, 
                  0, 0, 0, 1]);
Rxn = Rxn.reshape(4,4)  

temp=Rxn@Rx3@Rx2@Rx1@m
print(Rxn@Rx3@Rx2@Rx1)

ax2 = fig.add_subplot(223, projection='3d')
ax2.plot3D(temp[0,:], temp[1,:], temp[2,:], 'red')

velo = velo_to_cam @ m
ax3 = fig.add_subplot(224, projection='3d')
ax3.plot3D(velo[0,:], velo[1,:], velo[2,:], 'blue')

dja = m
ax4 = fig.add_subplot(222, projection='3d')
ax4.plot3D(dja[0,:], dja[1,:], dja[2,:], 'green')

ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z');
ax1.set_xlim(-20, 20)
ax1.set_ylim(-20, 20)
ax1.set_zlim(-20, 20)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z');
ax2.set_xlim(-20, 20)
ax2.set_ylim(-20, 20)
ax2.set_zlim(-20, 20)
ax3.set_xlabel('x')
ax3.set_ylabel('y')
ax3.set_zlabel('z');
ax3.set_xlim(-20, 20)
ax3.set_ylim(-20, 20)
ax3.set_zlim(-20, 20)
ax4.set_xlabel('x')
ax4.set_ylabel('y')
ax4.set_zlabel('z');
ax4.set_xlim(-20, 20)
ax4.set_ylim(-20, 20)
ax4.set_zlim(-20, 20)
plt.tight_layout()
plt.show()


'''
% 3D "M" rotarion
clf;
x3 = [0 1 1 4 7 7 8 8 7 4 1 0 0 0 1 1 4 7 7 8 8 7 4 1 0 0;
      0 0 6 0 6 0 0 8 8 2 8 8 0 0 0 6 0 6 0 0 8 8 2 8 8 0;
      0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1;
      1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];
figure(3);
hold on;
line(x3(1,:), x3(2,:), x3(3,:),'color','k');
r = [7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03;
    1.480249e-02,  7.280733e-04, -9.998902e-01, -7.631618e-02;
    9.998621e-01,  7.523790e-03,  1.480755e-02, -2.717806e-01;
    0,   0,   0 ,  1];

the = 0 * pi / 180;
% roll
Rx = [1 0        0         0; 
      0 cos(the) -sin(the) 0; 
      0 sin(the) cos(the)  0; 
      0 0        0         1];
% pitch
the = -90 * pi / 180;
Rx1 = [cos(the) 0 sin(the) 0;
        0       1   0      0;
        -sin(the) 0 cos(the) 0;
        0   0   0       1];
% yaw
the = 90 * pi / 180;
Rx2 =[cos(the) -sin(the) 0 0; 
      sin(the) cos(the)  0 0; 
       0        0        1 0;
       0 0 0 1];

identt=[1,0,0,0;
        0,1,0,0;
        0,0,1,0;
        0,0,0,1];
y2 = r * x3;
y4 = Rx2 * Rx * Rx1 * identt * x3;
line(y2(1,:), y2(2,:), y2(3,:),'color','red');
line(y4(1,:), y4(2,:), y4(3,:),'color','b');
xlabel('x axis'); ylabel('y axis'); zlabel('z axis');
axis([-20 20 -20 20 -20 20]); grid;
view(45,45)
pause(0.0001);
'''