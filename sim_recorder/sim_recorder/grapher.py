#BSD 3-Clause License
#
#Copyright (c) 2021, Florent Audonnet
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
import sys
import warnings
from collections import OrderedDict, defaultdict
from os import walk

import matplotlib.patches as mpatches
import numpy as np
from matplotlib import cm
from matplotlib import pyplot as plt
from numpy import linspace
from numpy.lib.scimath import sqrt
from scipy import signal

SMOOTH_INDEX = 21
POLY_INDEX = 3
if len(sys.argv) < 2:
    folder = ""
else:
    folder = sys.argv[1]
f = []
# exclude = ["data", "data_webots_org", "data_webots_throw", "data_webots", "data_gazebo", "data_gazebo_throw"]
# exclude = [el for el in exclude if el not in folder]
# exclude = ["data", "data_webots"]
for (dirpath, dirnames, filenames) in walk(os.path.join(os.path.dirname(__file__), "../data", folder), topdown=True):
    # dirnames[:] = [d for d in dirnames if d not in exclude]
    f.extend([os.path.join(*dirpath.split("/"), s) for s in filenames])
#tmp = [el for el in f if el[-5:] == "ipynb"]
tmp = [el for el in f if el[-3:] == "csv"]
print(f"Found {len(tmp)}")
types = defaultdict(list)
for el in tmp:
    if "ram" in el:
        types["ram"].append(el)
    else:
        types["cpu"].append(el)

procs = defaultdict(lambda: defaultdict(list))

for key, el in types.items():
    for name in el:
        existing = []
        with open(name) as f:
            for lines in f.readlines():
                lines = lines.replace("\n", "")
                line = lines.split(",")
                p = line[0]
                if "ruby" in p:
                    p = "ignition"
                val = line[1:]
                val = [float(x) for x in val if x]
                counter = 0
                new_p = p
                while new_p in existing:
                    counter += 1
                    new_p = new_p + "_" + str(counter)
                if counter != 0:
                    p = p+"_"+str(counter)
                procs[key][p].append(val)
                existing.append(p)
# colors = {}
# colors.update(mcolors.TABLEAU_COLORS)
# colors.update(mcolors.BASE_COLORS)
# colors.update(mcolors.CSS4_COLORS)
# colors = list(colors.values())
# random.shuffle(colors)

runtime = 0
success = 0
fruntime = 0
failure = 0
maxtime = 0
fmaxtime = 0
total = 0
start_time = 0;
with open(os.path.join(os.path.dirname(__file__), "../data", folder, "run.txt")) as f:
    for el in f.readlines():
        splitted = el.split()
        if not "Timeout" == splitted[0]:
            runtime += int(splitted[4])/1000
            total += 1
            start_time += int(splitted[-4])/1000
        if "Completed" == splitted[0]:
            success += 1
            if int(splitted[4])/1000 > maxtime:
                maxtime = int(splitted[4])/1000
        if "Failed" == splitted[0]:
            failure += 1
            if int(splitted[4])/1000 > fmaxtime:
                fmaxtime = int(splitted[4])/1000

    if total != 0:
        mean = runtime/total
        start_time /= total
    else:
        mean = 0
    mean_square = 0
    f.seek(0)
    for el in f.readlines():
        if not "Timeout" == el.split()[0]:
            val = int(el.split()[4])/1000

            mean_square += pow(val-mean, 2)
    if total != 0:
        stddev = sqrt(mean_square / total)
    else:
        stddev = 0

print(f"Name & Success & Failure & Timeout & Average Runtime & Standart Deviation\\\\")
print(f"{folder} & {success} & {failure} & {150-(success + failure)} & {mean:.2f} & {stddev:.2f} \\\\")


def create_figure(figname, printing=False):
    fig, axs = plt.subplots(2, figsize=(12, 8))

    for axs, (type, proc) in zip(axs, procs.items()):
        cm_subsection = linspace(0.0, 1.0, len(
            proc.values())+2)  # +2 to handle the span
        colors = [cm.jet(x) for x in cm_subsection]
        sorted_dict = OrderedDict()

        keys = sorted(proc.keys())
        for key in keys:
            sorted_dict[key] = proc[key]
        # colors.reverse()
        total = None
        length = 0
        for ls in sorted_dict.values():
            tmp = max(map(len, ls))
            if tmp > length:
                length = tmp
        for color, (name, ls) in zip(colors, sorted_dict.items()):
            arr = np.array([xi+[np.nan]*(length-len(xi)) for xi in ls])
            if "_win" in figname and type == "cpu": 
                arr *= 8 ## acount for windows using full cpu usage vs linux and core
            if total is None:
                total = arr
            else:
                arr = np.resize(arr, total.shape[0:2])
                total = np.dstack((arr, total))
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                meanarr = np.nanmean(arr, axis=0)
                standard_dev = np.nanstd(arr, axis=0)
            # because recording every 100 ms
            x = np.arange(0, meanarr.shape[0], 1)/10
            y = [meanarr]
            if printing:
                y = signal.savgol_filter(meanarr,
                                     SMOOTH_INDEX,  # window size used for filtering
                                     POLY_INDEX),  # order of fitted polynomial
            axs.plot(x, y[0], label=name, color=color)

            lower = meanarr-standard_dev
            high = meanarr+standard_dev
            if printing:
                lower = signal.savgol_filter(lower,
                                            SMOOTH_INDEX,  # window size used for filtering
                                            POLY_INDEX),  # order of fitted polynomial
                high = signal.savgol_filter(high,
                                            SMOOTH_INDEX,  # window size used for filtering
                                            POLY_INDEX),  # order of fitted polynomial

            else:
                lower = [lower]
                high = [high]

            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                lower[0][lower[0] < 0] = 0
                high[0][high[0] < 0] = 0

            axs.fill_between(x, lower[0], high[0],
                             alpha=0.5, interpolate=False, color=color)
            axs.set_xlabel("Time (s)")
            if type == "ram":
                axs.set_ylabel("RAM usage (MB)")
                axs.set_title("RAM usage against time")
            else:
                axs.set_title("CPU usage against time")
                axs.set_ylabel("CPU Usage (% of one core)")
        legend1 = axs.legend(bbox_to_anchor=(1, 1.1), loc="upper left")
        if success+failure != 0:
            axs.axvline(x=mean+start_time, ls='--', color=colors[-2], label="Mean success")
            axs.axvline(x=start_time, ls='--', color=colors[0], label="Task Start Time")
            axs.axvspan(mean-stddev+start_time, mean+stddev+start_time, alpha=0.2, color=colors[-2])

        if failure != 0:
            axs.axvspan(maxtime, x[-1], alpha=0.2, color=colors[-1])
        pmark = mpatches.Patch(facecolor=colors[-1],
                               edgecolor='white',
                               linestyle='--',
                               alpha=0.2,
                               label='Failure Only')
        # axs.annotate(f"{mean:.1f}",
        #             xy=(mean-max(x)/40, -15), xycoords=("data", "axes points") )

        lines = axs.get_lines()
        if failure != 0:
            legend2 = axs.legend([lines[-1], lines[-2], pmark], ["Task Start Time",'Average Runtime',
                                                      "Failure Only"], loc="upper right", bbox_to_anchor=(1, 1.1))
        elif success+failure != 0:
            legend2 = axs.legend(
                [lines[-1], lines[-2]], ["Task Start Time",'Average Runtime'], loc="upper right", bbox_to_anchor=(1, 1.1))

        axs.add_artist(legend1)
        axs.set_xticks(list(axs.get_xticks())[1:-1] + [start_time, mean+start_time])
        labels = axs.get_xticklabels()
        for idx, el in enumerate(axs.get_xticks()):
            labels[idx] = f"{el:.2f}"
        labels[-1] = f"\n{mean+start_time:.2f}"
        labels[-2] = f"\n{start_time:.2f}"
        axs.set_xticklabels(labels)

        if printing:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                meanarr = np.nanmean(total, axis=0)
                maxi = np.nanmax(total, axis=0)
                mini = np.nanmin(total, axis=0)
            a = np.nansum(meanarr, axis=1)
            b = np.nansum(maxi, axis=1)
            c = np.nansum(mini, axis=1)
            print(f"========={type}=========")
            print(f"Name & Max & Mean & Min \\\\")
            print(
                f"{folder} & {np.max(b):.0f} & {np.mean(a):.0f} & {np.min(c):.0f} \\\\")
            print(
                f"Number of processes : {len(sorted_dict.keys())}, Per process : {np.mean(a)/len(sorted_dict.keys()):.2f}")

    plt.subplots_adjust(bottom=0.08, top=0.95, hspace=0.26)

    #plt.subplots_adjust(hspace=0.25 + 0.2*(len(lines)-16))
    plt.savefig(os.path.join(os.path.dirname(__file__),
                             f"../data/{folder}/{figname}"), bbox_inches="tight")


create_figure(f"{folder}_smooth.svg", True)
create_figure(f"{folder}_no_smooth.svg",)
