from numpy import *
import matplotlib.pyplot as plt

plt.rc('text', usetex=True)
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 26}
plt.rc('font', **font)


roadFilePath = loadtxt('roadMat.dat')
roadCarFilePath = loadtxt('roadDataCar.dat')
roadTrkFilePath = loadtxt('roadDataTrk.dat')
velCarFilePath = loadtxt('velDataCar.dat')
velTrkFilePath = loadtxt('velDataTrk.dat')


# plot planned travel paths (pathPlan.png)
fig, ax = plt.subplots()

# main road
xPos_road = roadFilePath[:, 0]
yPos_road = roadFilePath[:, 1]
# car travel path
xPos_car = roadCarFilePath[:, 0]
yPos_car = roadCarFilePath[:, 1]
# truck travel path
xPos_trk = roadTrkFilePath[:, 0]
yPos_trk = roadTrkFilePath[:, 1]

# plot main road
ax.plot(xPos_road, yPos_road, 'b-', label='road data')
# plot car path
ax.plot(xPos_car, yPos_car, 'g-', label='car data')
# plot truck path
ax.plot(xPos_trk, yPos_trk, 'r-', label='truck data')

# format axis
ax.set_xlabel(r'$x [m]$')
ax.set_ylabel(r'$y [m]$')
ax.set_xlim((-30, 1300))
ax.set_ylim((-80, 240))
ax.grid(visible=True,alpha=0.5)
ax.legend(loc='upper right',prop={'size': 18})

fig.set_size_inches(10, 5)
plt.savefig('pathPlan.png', bbox_inches='tight', dpi=200)
plt.close()



# plot planned velocity trajectories (velPlan.png)
fig, ax = plt.subplots()

# car arclength and velocity
s_car = velCarFilePath[:, 1]
v_car = velCarFilePath[:, 2]
# truck arclength and velocity
s_trk = velTrkFilePath[:, 1]
v_trk = velTrkFilePath[:, 2]
# arclength and max velocity
v_max = velTrkFilePath[:, 3]

# plot truck velocity
ax.plot(s_trk, v_trk, 'r-', label=r'${v}_{trk}(s)$')
# plot car velocity
ax.plot(s_car, v_car, 'g-', label=r'${v}_{car}(s)$')
# plot max velocity
ax.plot(s_trk, v_max, 'b-', label=r'${v}_{max}(s)$')

# format axis
ax.set_xlabel(r'$s [m]$')
ax.set_ylabel(r'$v [m/s]$')
ax.set_xlim((0, 1535))
ax.set_xticks([0, 200, 400, 600, 800, 1000, 1200, 1400])
ax.set_ylim((0, 15))
ax.set_yticks([0, 2, 4, 6, 8, 10, 12, 14])
ax.grid(visible=True,alpha=0.5)
ax.legend(loc='upper left',prop={'size': 18})

fig.set_size_inches(10, 5)
plt.savefig('velPlan.png', bbox_inches='tight', dpi=200)
plt.close()