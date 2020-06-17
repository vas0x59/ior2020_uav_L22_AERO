import numpy as np
import matplotlib.pyplot as plt

type_mapping = {
    'water': 'b',
    'pastures': 'g',
    'seed': 'y',
    'potato': 'r',
    'soil': 'm'
}


d = {'water': [(0.5516167237691494, 1.1055739355966845), (0.07963070839334332, 2.6447184994821624), (1.4387859099528955, 2.039153054611976), (4.120973889989733, 0.10661801411406796), (3.1646031685359235, 2.365366893417597)], 'pastures': [(1.2476130271951487, 0.6066661067360831), (4.120973889989733, 0.10661801411406796), (3.9340635508716604, 3.6202971941570516)], 'seed': [(0.6303823308709453, 2.29881502814844), (1.0356142956119418, 2.9933790401551192), (0.18453704770477278, 3.4955212555812443), (2.19320752194517, 0.42908008189453767)], 'soil': [(2.3356711641665817, 2.299809264164062)], 'potato': [(2.4948318101475593, 1.4388867204556743)]}


for key in d:
    if key in ['water_land', 'seed_land', 'pastures_land']: continue
    for i in range(len(d[key])):
        color = type_mapping[key]
        if d[key][i][0] < 0 or d[key][i][1] < 0: continue
        plt.plot(d[key][i][0], d[key][i][1], marker = 'o', color = color)
        plt.annotate(key, d[key][i])
plt.show()