import numpy as np
import matplotlib.pyplot as plt

type_mapping = {
    'water': 'b',
    'pastures': 'g',
    'seed': 'y',
    'potato': 'r',
    'soil': 'm'
}


d = {'water': [(0.6682843242935954, 0.05576352611781728), (0.07417595522311199, 1.3580092745832155), (2.1411071891560107, 0.9028015113691168)], 'pastures': [], 'seed': [(1.8504489136638165, 0.05742380464529189)], 'potato': [], 'soil': [(0.1066737011091573, 0.4738402849504233), (1.2086184839330145, 2.1269133118752412), (1.4482893092308675, 0.35127770722116036)], 'water_land': [(1.743544169541543, 2.4466779227439464)], 'seed_land': [(1.3235200340239288, 0.9702029070199754)], 'pastures_land': []}

for key in d:
    if key in ['water_land', 'seed_land', 'pastures_land']: continue
    for i in range(len(d[key])):
        color = type_mapping[key]
        if d[key][i][0] < 0 or d[key][i][1] < 0: continue
        plt.plot(d[key][i][0], d[key][i][1], marker = 'o', color = color)
        plt.annotate(key, d[key][i])
plt.show()