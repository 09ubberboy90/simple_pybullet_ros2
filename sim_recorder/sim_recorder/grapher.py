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
from matplotlib.ticker import ScalarFormatter
import re


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
    elif "clock" in el:
        types["clock"].append(el)
    else:
        types["cpu"].append(el)

procs = defaultdict(lambda: defaultdict(list))
skipped = []
for key in ["cpu", "ram"]:
    for name in types[key]:
        existing = []
        with open(name) as f:
            for lines in f.readlines():
                lines = lines.replace("\n", "")
                line = lines.split(",")
                p = line[0]
                if "ruby" in p:
                    p = "ignition"
                val = line[2:] # skip first reading that is often eronous
                val = np.array([float(x) for x in val if x])
                counter = 0
                new_p = p
                while new_p in existing:
                    counter += 1
                    new_p = new_p + "_" + str(counter)
                if counter != 0:
                    p = p+"_"+str(counter)
                if np.all(val==0) and key == "cpu": ## Can't skip ram since cpu went first
                    skipped.append(p)
                procs[key][p].append(val)
                existing.append(p)
# colors = {}
# colors.update(mcolors.TABLEAU_COLORS)
# colors.update(mcolors.BASE_COLORS)
# colors.update(mcolors.CSS4_COLORS)
# colors = list(colors.values())
# random.shuffle(colors)

success = 0
maxtime = 0
start_time = 0;
timeout = 0
number_re = re.compile(r'\b\d+\b')
runtime = []

with open(os.path.join(os.path.dirname(__file__), "../data", folder, "run.txt")) as f:
    for el in f.readlines():
        splitted  = el.split()[0]
        numbers = [int(s) for s in re.findall(number_re, el)]
        if "Completed" == splitted:
            runtime.append(numbers[3]/1000)
            start_time += numbers[2]/1000
            success += 1
            if numbers[3]/1000 > maxtime:
                maxtime = numbers[3]/1000
        if "Timeout" == splitted:
            timeout += 1


if success != 0:
    mean = sum(runtime) / success
    start_time /= success
    mean_square = 0
    for el in runtime:
        mean_square += pow(el-mean, 2)
    stddev = sqrt(mean_square / success)

else:
    mean = 0
    stddev = 0

maxtime+= start_time

print(f"Name & Success & Timeout & Average Runtime & Standart Deviation\\\\")
print(f"{folder} & {success} & {timeout} & {mean:.2f} & {stddev:.2f} \\\\")


def create_figure(figname, printing=False):
    fig, axs = plt.subplots(2, figsize=(12, 8))

    for axs, (type, proc) in zip(axs, procs.items()):
        sorted_dict = OrderedDict()
        keys = sorted(proc.keys())
        for key in keys:
            if not key in skipped:
                sorted_dict[key] = proc[key]

        cm_subsection = linspace(0.0, 1.0, len(
            sorted_dict.values())+3)  # +2 to handle the span
        colors = [cm.jet(x) for x in cm_subsection]

        total = None
        length = 0
        for ls in sorted_dict.values():
            tmp = max(map(len, ls))
            if tmp > length:
                length = tmp
        for color, (name, ls) in zip(colors[1:], sorted_dict.items()):
            arr = np.array([np.concatenate((np.full(length-len(xi), np.nan), xi)) for xi in ls])
            if "_win" in figname and type == "cpu": 
                arr /= 24 ## acount for windows using full cpu usage vs linux and core
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
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                y[0][y[0] < 0] = 0 # clamp to zero
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
                axs.set_yscale('log')
                axs.yaxis.set_major_formatter(ScalarFormatter())

            else:
                axs.set_title("CPU usage against time")
                axs.set_ylabel("CPU Usage (% of one core)")
        # meanarr = np.mean(total, axis=0)
        # sum_arr = np.sum(meanarr, axis=1)
        # axs.plot(x, sum_arr, label="Total", color=colors[-2])

        legend1 = axs.legend(bbox_to_anchor=(1, 1.1), loc="upper left")
        
    
        std_dev_diff = 0
        timeout_diff = 0

        if success != 0:
            axs.axvline(x=mean+start_time, ls='--', color=colors[-2], label="Mean success")
            axs.axvline(x=start_time, ls='--', color=colors[0], label="Task Start Time")
            axs.axvspan(mean-stddev+start_time, min(mean+stddev+start_time, x[-1]), alpha=0.2, color=colors[-3])
            std_dev_diff = (mean+stddev+start_time) - (mean-stddev+start_time)
            std_dev_mark = mpatches.Patch(facecolor=colors[-3],
                                edgecolor='white',
                                linestyle='--',
                                alpha=0.2,
                                label='Standard Deviation')
        
        timeout_diff = x[-1] - maxtime
        if timeout != 0 and timeout_diff > 0:
            axs.axvspan(maxtime, x[-1], alpha=0.2, color=colors[-1])
            timeout_mark = mpatches.Patch(facecolor=colors[-1],
                        edgecolor='white',
                        linestyle='--',
                        alpha=0.2,
                        label='Timeout Only')

        lines = axs.get_lines()
        lines_data = [lines[-1], lines[-2], ]
        lines_legend = ["Task Start Time",'Mean Runtime',]

        if std_dev_diff > 0:
            lines_data.append(std_dev_mark)            
            lines_legend.append("Standard Deviation")            


        if timeout != 0 and timeout_diff > 0:
            lines_data.append(timeout_mark)            
            lines_legend.append("Timeout Only")       

        legend2 = axs.legend(lines_data,lines_legend, loc="upper right", bbox_to_anchor=(1, 1.1))
        axs.add_artist(legend1)
        axs.add_artist(legend2)
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
            np.savetxt(os.path.join(os.path.dirname(__file__),
                             f"../data/{folder}/{folder}_total_{type}.txt"), a)
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

def create_clock_plot(figname):
    arr = []
    time = []
    max_length = 0
    fig, ax = plt.subplots()
    for name in types["clock"]:
        with open(name) as f:
            tmp = f.readline().strip().split(",")
            tmp2 = f.readline().strip().split(",")
            realtime = np.array([int(i) for i in tmp])
            simtime = np.array([int(i) for i in tmp2])
        realtime -= realtime[0]

        sim_delta = []
        real_delta = []
        for el in range(simtime.shape[0]-1):
            sim_delta.append(simtime[el]-simtime[el+1])
        for el in range(realtime.shape[0]-1):
            real_delta.append(realtime[el]-realtime[el+1])    
            
        sim_delta = np.array(sim_delta)
        real_delta = np.array(real_delta)

        div = sim_delta/real_delta
        if div.shape[0] > max_length:
            max_length = div.shape[0]
        arr.append(div)
        time.append(realtime[:-1]/pow(10,9))

    arr = np.array([np.concatenate((np.full(max_length-xi.shape[0], np.nan), xi)) for xi in arr])
    time = np.array([np.concatenate((np.full(max_length-xi.shape[0], np.nan), xi)) for xi in time])
    meanarr = np.nanmean(arr, axis=0)
    y = signal.savgol_filter(meanarr,
                            SMOOTH_INDEX,  # window size used for filtering    
                            POLY_INDEX),  # order of fitted polynomial
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        y[0][y[0] < 0] = 0 # clamp to zero

    ax.plot(np.nanmax(time, axis=0),y[0] ,label=f"{folder}",)
    
    ax.legend()
    ax.set_ylabel("Real time factor (%)")
    ax.set_xlabel("Time(s)")

    # ax.set_xticks([], [])
    plt.savefig(os.path.join(os.path.dirname(__file__),
                             f"../data/{folder}/{figname}"), bbox_inches="tight")

create_clock_plot(f"{folder}_clock.svg")
create_figure(f"{folder}_smooth.svg", True)
create_figure(f"{folder}_no_smooth.svg",)