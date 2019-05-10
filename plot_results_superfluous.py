#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np # mean, std, var
import re
import os

NSUGGESTIONS = 10
SAME_START = False

def split_csv_line(line):
    return re.compile(",\s?").split(line)


def parse_csv(filepath):
    data = {}  # key: exp type, values = list of rewards for that type
    with open(filepath, 'r') as f:
        for l in f:
            split = split_csv_line(l)
            if len(split) == 1:
                continue
            if split[0] not in data:
                data[split[0]] = (split[2], [])
            data[split[0]][1].append(float(split[1]))
    return data


def mode(x):
    values, counts = np.unique(x, return_counts=True)
    m = counts.argmax()
    return values[m]#, counts[m]


def join(d1, d2):
    for k, v in d2.iteritems():
        assert(v[0] == d1[k][0])
        d1[k][1].extend(v[1])
    return d1

# Prepares the data for the plot
def prepare_data(data):
    r = re.compile(r'RAND-(\d+)\+SUGG-(\d+)')
    rr = re.compile(r'RANDOM-(\d+)')
    rs = re.compile(r'SUGGESTIONS-(\d+)')
    newdata = {}  # Key will be the xtick point of the data

    if SAME_START:
        for i in xrange(NSUGGESTIONS):
            newdata['RANDOM '+str(i+1) + ' + PSS'] = (range(NSUGGESTIONS-i), [-1] * (NSUGGESTIONS-i), [-1] * (NSUGGESTIONS-i))
    else:
        for i in xrange(NSUGGESTIONS):
            newdata['RANDOM ' + str(i + 1) + ' + PSS'] = (range(i+1, NSUGGESTIONS + 1), [-1] * (NSUGGESTIONS - i), [-1] * (NSUGGESTIONS - i))
    newdata['PSS'] = (range(0, NSUGGESTIONS+1), [-1]*(NSUGGESTIONS+1), [-1]*(NSUGGESTIONS+1))

    for k, v in data.iteritems():
        if 'RANDOM' in k:
            m = re.search(rr, k)
            newdata['RANDOM ' + m.group(1) + ' + PSS'][1][0] = np.mean(v[1])
            newdata['RANDOM ' + m.group(1) + ' + PSS'][2][0] = np.std(v[1])
        elif 'RAND' in k:
            m = re.search(r, k)
            newdata['RANDOM ' + m.group(1) + ' + PSS'][1][int(m.group(2))] = np.mean(v[1])
            newdata['RANDOM ' + m.group(1) + ' + PSS'][2][int(m.group(2))] = np.std(v[1])
        elif 'SUGGESTION' in k:
            m = re.search(rs, k)
            newdata['PSS'][1][int(m.group(1))] = np.mean(v[1])
            newdata['PSS'][2][int(m.group(1))] = np.std(v[1])
        elif 'BASELINE' in k:
            newdata['PSS'][1][0] = np.mean(v[1])
            newdata['PSS'][2][0] = np.std(v[1])
        else:
            newdata[k] = np.mean(v[1])
            exit(-1)
    return newdata


def plot_one(x, y, label, ax, width=1, color=None, std=True):
    line2, = ax.plot(x, y, '.-', linewidth=width, label=label, color=color, )
    if std:
        ax.fill_between(x, np.add(y, std), np.subtract(y, std), facecolor=line2.get_c(), alpha=0.5)


def trim(x, y):
    while y[-1] == -1:
        y.pop()
    x = x[0:len(y)]
    return x, y

def plot(data, plot_path, show=True, std=False, ):
    fig, ax = plt.subplots()

    for k, d in data.iteritems():
        x = d[0]
        y = d[1]  # mean and std
        x, y = trim(x, y)
        plot_one(x, y, label=k, ax=ax, std=d[2] if std else False)

    # Final
    plt.title('PSS with superfluous predicates')
    plt.ylabel('Reward')
    if SAME_START:
        plt.xlabel('Suggestions')
    else:
        plt.xlabel('Known predicates')
    ax.legend()
    print "Saving plot to", plot_path
    plt.savefig(plot_path, format='svg', bbox_inches='tight')
    if show:
        plt.show()


if __name__ == '__main__':

    data_feeding = prepare_data(parse_csv('/home/gcanal/Dropbox/PrefsIROS19/final_results/superfluous_feeding_results.txt'))
    data_jacket = prepare_data(parse_csv('/home/gcanal/Dropbox/PrefsIROS19/final_results/superfluous_jacket_dressing_results.txt'))
    data_shoe = prepare_data(parse_csv('/home/gcanal/Dropbox/PrefsIROS19/superfluous_shoe_results.txt'))

    data = {'Shoe fitting': data_shoe['PSS'], 'Jacket dressing': data_jacket['PSS'], 'Feeding': data_feeding['PSS']}
    plot(data, plot_path=os.getcwd()+"/superflous_results.svg")
    print 'Done'
