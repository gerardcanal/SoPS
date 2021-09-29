#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np # mean, std, var
import re
import os

DOMAIN_NAME='shoe'
NSUGGESTIONS = 8  # 6 jacket, feeding, 8 shoe
SAME_START = False


def split_csv_line(line):
    return re.compile(",\s?").split(line)


def parse_csv(filepath):
    columns = ['REWARD', 'NADDITIONS', 'NCHANGES']
    data = {}  # key: exp type, values = list of rewards for that type
    with open(filepath, 'r') as f:
        for l in f:
            split = split_csv_line(l)
            if len(split) == 1:
                continue
            if split[0] not in data:
                data[split[0]] = {}
            for i in xrange(len(columns)):
                if columns[i] not in data[split[0]]:
                    data[split[0]][columns[i]] = [[] for _ in xrange(NSUGGESTIONS+1)]
                data[split[0]][columns[i]][int(split[1])].append(float(split[i+2]))
            #for i in xrange(2, len(split)):
            #    data[split[0]][int(split[1])][i-2].append(float(split[i]))
    return data


def prepare_data(data):
    for k, v in data.iteritems():
        for k1, v1 in v.iteritems():
            for i in xrange(len(v1)):
                data[k][k1][i] = np.mean(v1[i]) if v1[i] else -1
    return data

def trim(x, y):
    while y and y[-1] == -1:
        y.pop()
    x = x[0:len(y)]
    return x, y

def plot_one(x, y, label, ax, width=1, color=None, std=True):
    #x_idxs = range(len(x))
    line2, = ax.plot(x, y, '.-', linewidth=width, label=label, color=color, )
    #plt.xticks(x_idxs, x, rotation=90)
    if std:
        #sigma = np.std(y)
        ax.fill_between(x, np.add(y, std), np.subtract(y, std), facecolor=line2.get_c(), alpha=0.5)


def plot(data, plot_path, show=True, std=False, title='Results PSS-change %s domain' % DOMAIN_NAME):
    fig, ax = plt.subplots()

    plot_key = 'REWARD'
    for k, v in sorted(data.iteritems()):
        x = range(len(v[plot_key]))
        y = v[plot_key]  # mean and std
        x, y = trim(x, y)
        plot_one(x, y, label=k, ax=ax, std=v[2] if std else False)

    # Final
    plt.title(title)
    plt.ylabel('Reward')
    if SAME_START:
        plt.xlabel('Suggestions')
    else:
        plt.xlabel('Fixed predicates')
    ax.legend()
    print "Saving plot to", plot_path
    plt.savefig(plot_path, format='svg', bbox_inches='tight')
    if show:
        plt.show()


if __name__ == '__main__':
    domains = {'Jacket dressing': 'final_results/changes_jacket_results.txt',
               'Shoe fitting': 'final_results/changes_shoe_results.txt',
               'Assistive feeding': 'final_results/changes_feeding_results.txt'}

    for DOMAIN_NAME, path in domains.iteritems():
        data = parse_csv(path)
        data = prepare_data(data)

        plot(data, title='%s domain allowing changes' % DOMAIN_NAME, plot_path=os.getcwd()+"/changes_%s_results.svg" % DOMAIN_NAME.replace(' ', '-').lower())
    print 'Done'
