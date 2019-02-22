#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np # mean, std, var
import re

NSUGGESTIONS = 8

def split_csv_line(line):
    return re.compile(",\s?").split(line)


def parse_csv(filepath):
    data = {}  # key: exp type, values = list of rewards for that type
    with open(filepath, 'r') as f:
        for l in f:
            split = split_csv_line(l)
            if split[0] not in data:
                data[split[0]] = (split[2], [])
            data[split[0]][1].append(float(split[1]))
    return data


# Prepares the data for the plot
def prepare_data(data):
    r = re.compile(r'RAND-(\d+)\+SUGG-(\d+)')
    rr = re.compile(r'RANDOM-(\d+)')
    rs = re.compile(r'SUGGESTIONS-(\d+)')
    newdata = {}  # Key will be the xtick point of the data

    for i in xrange(NSUGGESTIONS):
        newdata['RANDOM '+str(i+1) + ' + PSS'] = (range(NSUGGESTIONS-i), [-1] * (NSUGGESTIONS-i))
    newdata['PSS'] = (range(1, NSUGGESTIONS+1), [-1]*NSUGGESTIONS)

    for k, v in data.iteritems():
        if 'RANDOM' in k:
            m = re.search(rr, k)
            newdata['RANDOM ' + m.group(1) + ' + PSS'][1][0] = np.mean(v[1])
        elif 'RAND' in k:
            m = re.search(r, k)
            newdata['RANDOM ' + m.group(1) + ' + PSS'][1][int(m.group(2))] = np.mean(v[1])
        elif 'SUGGESTION' in k:
            m = re.search(rs, k)
            newdata['PSS'][1][int(m.group(1))-1] = np.mean(v[1])
        elif 'BASELINE' in k:
            newdata['PSS'][1][0] = np.mean(v[1])
        else:
            newdata[k] = np.mean(v[1])
    return newdata


def plot_one(x, y, label, ax, width=1, color=None):
    #x_idxs = range(len(x))
    line2, = ax.plot(x, y, '.-', linewidth=width, label=label, color=color, )
    #plt.xticks(x_idxs, x, rotation=90)


def plot(data, show=True):
    fig, ax = plt.subplots()

    for k, v in sorted(data.iteritems()):
        x = v[0]
        y = v[1]
        plot_one(x, y, label=k, ax=ax)

    # Final
    plt.title('Results shoe domain')
    plt.ylabel('Reward')
    plt.xlabel('Suggestions')
    ax.legend()
    #plt.savefig(PLOT_SAVE_PATH, format='svg', bbox_inches='tight')
    if show:
        plt.show()


if __name__ == '__main__':
    path = '/home/gcanal/Dropbox/PrefsIROS19/shoe_results_rdn15_10.txt'
    data = parse_csv(path)
    data = prepare_data(data)
    plot(data)
    print 'Done'
