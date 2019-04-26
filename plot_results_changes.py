#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np # mean, std, var
import re
import os

DOMAIN_NAME='feeding'
NSUGGESTIONS = 6
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
                data[k][k1][i] = np.mean(v1[i])
    return data


def plot_one(x, y, label, ax, width=1, color=None, std=True):
    #x_idxs = range(len(x))
    line2, = ax.plot(x, y, '.-', linewidth=width, label=label, color=color, )
    #plt.xticks(x_idxs, x, rotation=90)
    if std:
        #sigma = np.std(y)
        ax.fill_between(x, np.add(y, std), np.subtract(y, std), facecolor=line2.get_c(), alpha=0.5)


def plot(data, plot_path, show=True, std=False, ):
    fig, ax = plt.subplots()

    plot_key = 'REWARD'
    for k, v in sorted(data.iteritems()):
        x = range(len(v[plot_key]))
        y = v[plot_key]  # mean and std
        plot_one(x, y, label=k, ax=ax, std=v[2] if std else False)

    # Final
    plt.title('Results PSS-change %s domain' % DOMAIN_NAME)
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
    ## Shoe
    path = '/home/gcanal/Dropbox/PrefsIROS19/shoe_results_rdn15_10.txt'
    path = '/home/gcanal/Dropbox/PrefsIROS19/final_shoe_results_rdn50x20.txt'
    path = '/home/gcanal/Dropbox/PrefsIROS19/shoe_results.txt'
    path = '/home/gcanal/Dropbox/PrefsIROS19/final_results/shoe_results.txt'
    #path = '/tmp/shoe_results.txt'
    #data2 = parse_csv('/home/gcanal/Dropbox/PrefsIROS19/shoe_results.txt')
    #data = join(data, data2)
    #data.update(data2)

    ## Jacket
    path = '/home/gcanal/Dropbox/PrefsIROS19/final_results/jacket_dressing_results.txt'
    path = '/home/gcanal/Dropbox/PrefsIROS19/final_results/feeding_results.txt'
    path = '/home/gcanal/Dropbox/PrefsIROS19/changes_feeding_results.txt'
    data = parse_csv(path)
    data = prepare_data(data)

    plot(data, plot_path=os.getcwd()+"/changes_%s_results.svg" % DOMAIN_NAME)
    print 'Done'
