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
    #x_idxs = range(len(x))
    line2, = ax.plot(x, y, '.-', linewidth=width, label=label, color=color, )
    #plt.xticks(x_idxs, x, rotation=90)
    if std:
        #sigma = np.std(y)
        ax.fill_between(x, np.add(y, std), np.subtract(y, std), facecolor=line2.get_c(), alpha=0.5)


def plot(data, plot_path, show=True, std=False, ):
    fig, ax = plt.subplots()

    for k, v in sorted(data.iteritems()):
        x = v[0]
        y = v[1]  # mean and std
        plot_one(x, y, label=k, ax=ax, std=v[2] if std else False)

    # Final
    plt.title('Results %s domain' % DOMAIN_NAME)
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
    data = parse_csv(path)

    data = prepare_data(data)
    plot(data, plot_path=os.getcwd()+"/%s_results.svg" % DOMAIN_NAME)
    print 'Done'
