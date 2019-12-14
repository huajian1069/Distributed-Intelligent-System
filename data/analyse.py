import os
import numpy as np 
import matplotlib.pyplot as plt

# FILENAME = os.path.join(os.getenv("HOME"), 'webots_pso.csv')
FILENAME = 'data/obstacles_swarm6_fixedrad_v2_lw2_nw2_n1.csv'
AGGREGATION_BIN = 20

results = np.genfromtxt(FILENAME, delimiter=',')
aggregated_results = []

best_result_index = np.argmax(results[:, 6])
best_result = np.max(results[:, 6])

for i in range(0, len(results), AGGREGATION_BIN):
    best_bin_result = np.max(results[i:i+AGGREGATION_BIN, 6])
    aggregated_results.append(best_bin_result)

print(best_result_index, best_result)

# plt.plot(results[:, 6])
plt.plot(aggregated_results)
plt.savefig('analyse.pdf', format='pdf')

plt.show()
