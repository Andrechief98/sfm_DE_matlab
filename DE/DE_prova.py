#!/usr/bin/env python3

import csv
from pedestrian import Pedestrian
import json
import matplotlib.pyplot as plt
import numpy as np

pedestrian_list=[]
time_vector=[]

for id in range(13):
    ped=Pedestrian(id+1)
    pedestrian_list.append(ped)

with open('exp_1_run_1.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count=0

    for row in csv_reader:
        if line_count==0:
            line_count+=1
            continue
        else:
            line_count+=1
            time_vector.append(row[1])
            counter=0
            for ped in pedestrian_list:
                    if row[2+counter] == "NaN" and row[15+counter] == "NaN":
                        ped.trajectory.append((row[2+counter],row[15+counter]))
                    else:
                        ped.trajectory.append((float(row[2+counter])/1000,float(row[15+counter])/1000))
                    counter+=1
# print("PROVA:")
print(pedestrian_list[1].trajectory[1])
print(type(pedestrian_list[1].trajectory[1][0]))
print(type(pedestrian_list[1].trajectory[1][1]))

pedestrian_list[2].update_goal(4.46)
print(pedestrian_list[2].goal)

# #COSTRUZIONE GRAFICO
# fig, ax = plt.subplots()

# ax.plot(1, 2, color="green", marker='o', markersize=7)

# ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
#        ylim=(0, 8), yticks=np.arange(1, 8))

# #per dare un nome agli assi
# plt.xlabel('Smarts')
# plt.ylabel('Probability')

# #per dare un titolo al grafico
# plt.title('Histogram of IQ')

# #per aggiungere del testo nella coordinata specifica
# plt.text(60, .025, r'$\mu=100,\ \sigma=15$')

# #per specificare limiti max e minimi delle variabili di ogni asse
# plt.axis([40, 160, 0, 0.03])

# #per visualizzare la griglia
# plt.grid(True)

# #per mostrare il grafico
# plt.show()

# plt.clf()
# plt.close('Histogram of IQ')



#SIMULATION:
fig, ax=plt.subplots()

skip_frame=100 #1 sec
for instant in range(len(time_vector)):
    ax.set_xlabel('X coordinate')
    ax.set_ylabel('Y coordinate')
    ax.set_title("Simulation")
    plt.axis([-10, 10, -15, 15])

    time=time_vector[instant*skip_frame]
    print(time)

    for pedestrian in pedestrian_list:
        if type(pedestrian.trajectory[instant*skip_frame][0]) == float:
            ax.plot(pedestrian.trajectory[instant*skip_frame][0], pedestrian.trajectory[instant*skip_frame][1], color='green', marker='.')
            plt.pause(0.01)
            continue 
        else:
            continue
    ax.clear()
plt.show()