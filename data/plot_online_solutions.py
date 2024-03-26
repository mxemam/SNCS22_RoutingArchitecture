from numpy import *
import matplotlib.pyplot as plt

plt.rc('text', usetex=True)
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 26}
plt.rc('font', **font)


trackCarData = loadtxt('pathTrackCar.dat')
trackTrkData = loadtxt('pathTrackTrk.dat')



# plot patherr_car
fig, ax = plt.subplots()

# car data
t_car = trackCarData[:, 0]
d_car = trackCarData[:, 5]
chi_car = trackCarData[:, 6]

# plot errors
ax.plot(t_car, sqrt((d_car)**2), 'b-', label=r'$||{d}_{car}||$')
ax.plot(t_car, sqrt((chi_car)**2), 'r-', label=r'$||{\chi}_{car}||$')

# format axis
ax.set_xlabel(r'$t [s]$')
ax.set_xlim((0, 227))
ax.set_ylim((0, 0.05))
ax.grid(visible=True,alpha=0.5)
ax.legend(loc='upper right',prop={'size': 16})

fig.set_size_inches(10, 5)
plt.savefig('patherr_car.png', bbox_inches='tight', dpi=200)
plt.close()



# plot patherr_trk
fig, ax = plt.subplots()

# truck data
t_trk = trackTrkData[:, 0]
d_trk = trackTrkData[:, 5]
chi_trk = trackTrkData[:, 6]

# plot errors
ax.plot(t_trk, sqrt((d_trk)**2), 'b-', label=r'$||{d}_{trk}||$')
ax.plot(t_trk, sqrt((chi_trk)**2), 'r-', label=r'$||{\chi}_{trk}||$')

# format axis
ax.set_xlabel(r'$t [s]$')
ax.set_xlim((0, 227))
ax.set_ylim((0, 0.05))
ax.grid(visible=True,alpha=0.5)
ax.legend(loc='upper right',prop={'size': 16})

fig.set_size_inches(10, 5)
plt.savefig('patherr_trk.png', bbox_inches='tight', dpi=200)
plt.close()



# plot veltrj_car
fig, ax = plt.subplots()

# car data
s_car = trackCarData[:, 4]
v_car = trackCarData[:, 8]
vopt_car = trackCarData[:, 19]
vmax_car = trackCarData[:, 20]

# plot velocities
ax.plot(s_car, vmax_car, 'b-', label=r'${v}_{max}(s)$')
ax.plot(s_car, vopt_car, 'r-', label=r'${v}_{opt}(s)$')
ax.plot(s_car, v_car, 'g-', label=r'${v}_{car}(s)$')

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
plt.savefig('veltrj_car.png', bbox_inches='tight', dpi=200)
plt.close()



# plot veltrj_trk
fig, ax = plt.subplots()

# truck data
s_trk = trackTrkData[:, 4]
v_trk = trackTrkData[:, 8]
vopt_trk = trackTrkData[:, 19]
vmax_trk = trackTrkData[:, 20]

# plot velocities
ax.plot(s_trk, vmax_trk, 'b-', label=r'${v}_{max}(s)$')
ax.plot(s_trk, vopt_trk, 'r-', label=r'${v}_{opt}(s)$')
ax.plot(s_trk, v_trk, 'g-', label=r'${v}_{trk}(s)$')

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
plt.savefig('veltrj_trk.png', bbox_inches='tight', dpi=200)
plt.close()



# plot velTrjCmp
fig, ax = plt.subplots()

# plot velocities
ax.plot(t_car, v_car, 'b-', label=r'${v}_{car}$')
ax.plot(t_trk, v_trk, 'r-', label=r'${v}_{trk}$')

# format axis
ax.set_xlabel(r'$t [s]$')
ax.set_ylabel(r'$v [m/s]$')
ax.set_xlim((0, 225))
ax.set_ylim((0, 15))
ax.set_yticks([0, 2, 4, 6, 8, 10, 12, 14])
ax.grid(visible=True,alpha=0.5)
ax.legend(loc='upper left',prop={'size': 18})

fig.set_size_inches(10, 5)
plt.savefig('velTrjCmp.png', bbox_inches='tight', dpi=200)
plt.close()



# plot cputime
fig, ax = plt.subplots()

# data
cpu_car = trackCarData[:, 22]
cpu_trk = trackTrkData[:, 22]

# plot cputime
ax.plot(t_car, cpu_car, 'b-', label=r'${cpu}_{car}$')
ax.plot(t_trk, cpu_trk, 'r-', label=r'${cpu}_{trk}$')

# format axis
ax.set_xlabel(r'$t [s]$')
ax.set_ylabel(r'$t [s]$')
ax.set_xlim((0, 230))
ax.set_ylim((0, 0.2))
ax.grid(visible=True,alpha=0.5)
ax.legend(loc='upper left',prop={'size': 18})

fig.set_size_inches(10, 5)
plt.savefig('cputime.png', bbox_inches='tight', dpi=200)
plt.close()