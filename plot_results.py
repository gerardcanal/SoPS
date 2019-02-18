#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np # mean, std, var

def split_csv_line(line):
    return line.split(', ')

def parse_csv(filepath):
    data = {}  # key: exp type, values = list of rewards for that type
    with open(csv_path, 'r') as f:
        for l in f:
            split = split_csv_line(l)
            if split[0] in data:
                data[split[0]] = (split[2], [])
            else:
                data[split[0]][1].append(float(split[1]))
    return data

def plot_one(x, y, label, ax, width=2, color=None):
    x_idxs = range(len(x))
    line2, = ax.plot(x_idxs, y, linewidth=width, label=label, color=color)
    plt.xticks(x_idxs, x, rotation=90)


def plot(data, show=True):
    fig, ax = plt.subplots()

    for k, v in data:
        x = v[0]
        y = np.mean(v[1])
        plot_one(x, y, label=k, ax=ax)

    # Final
    plt.title(TITLE)
    plt.ylabel('Words')
    ax.legend()
    plt.savefig(PLOT_SAVE_PATH, format='svg', bbox_inches='tight')
    if show:
        plt.show()