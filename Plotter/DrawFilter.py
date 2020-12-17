import sys
import matplotlib.pyplot as plt
import csv

t=[]
y=[]
true=[]
flt=[]


with open(sys.argv[2], 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=';')
    for row in plots:
        t.append(float(row[0]))
        true.append(float(row[1]))
        flt.append(float(row[2]))
        y.append(float(row[3]))


fig, ax = plt.subplots(1, 1, figsize=(4, 4), dpi=400)
plt.grid()
plt.title(sys.argv[1])
plt.plot(t, y, 'b',label='signal', linewidth = 0.4)
plt.plot(t, flt, 'r', label='filtered signal', linewidth = 1)
plt.plot(t, true, 'orange', label='real value', linewidth = 1)
ax.legend(fontsize='xx-small')
plt.show()