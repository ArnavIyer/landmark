data = open('localize.txt', 'r').read()

odometry = [[float(j) for j in i.split(' ')[2:]] for i in data.split('\n') if 'VERTEX_SE2' == i.split(' ')[0]]
print(str(odometry).replace('[','{').replace(']', '}'))
print()

landmarks = [[int(j) for j in i.split(' ')[1:3]] + [float(j) for j in i.split(' ')[3:]] for i in data.split('\n') if 'LANDMARK' == i.split(' ')[0]]
a = max([i[1] for i in landmarks])

print(str(landmarks).replace('[','{').replace(']', '}'))

import matplotlib.pyplot as plt

plt.scatter([i[5-2] for i in odometry], [i[6-2] for i in odometry])
plt.show()

print(a)