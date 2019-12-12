import os
import numpy as np 
import matplotlib.pyplot as plt

FILENAME = 'webots_pso_1576163893.csv'

results = np.genfromtxt(os.path.join(os.getenv("HOME"), FILENAME), delimiter=',')

best_result_index = np.argmax(results[:, 6])
best_result = np.max(results[:, 6])


print(best_result_index, best_result)

plt.plot(results[:, 6])
plt.show()