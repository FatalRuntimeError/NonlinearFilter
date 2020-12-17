import sys
import matplotlib.pyplot as plt
import csv

x=[]
y=[]

colors = ["blue", "red", "orange", "green", "black", "grey"]

count = int(sys.argv[2])

with open(sys.argv[1], 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=';')
    for row in plots:
        x.append(float(row[0]))
        for i in range(count):
            y.append(float(row[i + 1]))

fig, ax = plt.subplots(1, 1, figsize=(3, 2), dpi=400)
plt.grid()

for i in range(count):
    yi = []
    for k in range(int(len(y) / count)):
        yi.append(y[i + k * count])

    plt.plot(x, yi, colors[i], linewidth = 0.4)

plt.show()