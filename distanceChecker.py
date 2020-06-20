import numpy as np
import matplotlib.pyplot as plt

#d = {'water': [(0.8873134493642003, 1.2540822306141264), (0.12680818485758294, 2.784158016161086), (1.2556914291375354, 2.3942925156490116), (2.799334205169208, 2.793371710128687)], 'pastures': [], 'seed': [(0.4938548570852421, 2.404726109190621), (0.9034012490610992, 3.1646465144201708), (1.6555758637451992, 0.5821088577126128)], 'potato': [], 'soil': [(2.403254638234139, 0.5035148888594937)], 'water_land': [], 'seed_land': [[1.6343488134744035, 2.0689548225126213, 7]], 'pastures_land': []}

d = 

field = {
    'seed': [(1.662, 0.528), (0.906, 3.174), (0.528, 2.418)],
    'seed_land': [(0.15, 3.552)],
    'water': [(0.15, 2.796), (1.284, 2.418), (0.906, 1.284), (2.796, 2.796)],
    'water_land': [(3.552, 0.15)],
    'pastures': [(1.284, 1.284)],
    'pastures_land': [(3.552, 3.552)],
    'potato': [(2.418, 0.528)],
    'soil': [(2.418, 2.04)]
}

TH = 1
mapping = {
    'water': 'b',
    'pastures': 'g',
    'seed': 'y',
    'potato': 'r',
    'soil': 'm',
    'water_land': 'b',
    'seed_land': 'y',
    'pastures_land': 'g'
}

MODE = 0

deltas = []

def distance(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

for key in d:
    for i in range(len(d[key])):
        if len(d[key][i]) == 0: continue
        if MODE:
            temp = np.array(d[key][i])
            if len(temp.shape) == 1:
                d[key][i] = np.delete(d[key][i], 2)
                temp = np.array(d[key][i])
                m = np.array(temp)
            else:
                m = temp.mean(axis=0)
            # plt.plot(m[0], m[1], marker='o', color=mapping[key])
            min_ = 99999999999999
            for point in field[key]:
                d = distance(m, (point[0], point[1]))
                if d < min_  and dist < TH:
                    min_ = d
            if min_ != 99999999999999:
                deltas.append(min_)
        else:
            # plt.plot(d[key][i][0], d[key][i][1], marker='o', color=mapping[key])
            min_ = 99999999999999
            for point in field[key]:
                dist = distance(d[key][i], (point[0], point[1]))
                if dist < min_ and dist < TH:
                    min_ = dist
            if min_ != 99999999999999:
                deltas.append(min_)

print(sum(deltas)/len(deltas), max(deltas), min(deltas))   
# plt.show()
