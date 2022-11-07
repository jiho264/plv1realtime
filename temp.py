import matplotlib.pyplot as plt
import numpy as np
  
x = np.linspace(0, 10*np.pi, 100)
y = np.sin(x)
  
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'b-')
  
for phase in np.linspace(0, 10*np.pi, 100):
    fig.canvas.draw()
    fig.canvas.flush_events()