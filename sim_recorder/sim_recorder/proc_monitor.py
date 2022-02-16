# BSD 3-Clause License
#
# Copyright (c) 2021, Florent Audonnet
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import signal
import sys
from collections import defaultdict
import threading

import psutil
import rclpy
from rclpy.node import Node
import numpy as np
class ProcMonitor(Node):
    """
    Create the main window and connect the menu bar slots.
    """

    def __init__(self, allowed, idx, sim_name, path):
        super().__init__('proccess_monitor')

        self.proc_nb = len(allowed)*2

        self.cpu = np.full((self.proc_nb, 100), np.nan, dtype=np.float)
        self.ram = np.full((self.proc_nb, 100), np.nan, dtype=np.float)#
        self.allowed = allowed
        self.capacity = {"cpu":100,"ram":100}
        self.size = 0
        self.counter = 0

        self.procs = {}
        self.pids = {}

        self.timer = self.create_timer(0.1, self.animate)
        self.idx = idx
        self.sim_name = sim_name
        self.path = path

    def get_proc(self):
        for proc in psutil.process_iter():
            pid = proc.pid
            name = proc.name()
            if pid not in self.procs.keys() and name in self.allowed:
                self.procs[pid] = proc
                self.pids[pid] = (name, self.counter)
                proc.cpu_percent()  # discard value
                proc.cpu_percent()  # discard value
                self.counter += 1

    def add(self, array, type, pid, data):
        if self.size == self.capacity[type]:
            self.capacity[type] *= 4
            newdata = np.full((self.proc_nb, self.capacity[type]),np.nan, dtype=np.float)
            newdata[:,:self.size] = array
            array = newdata
        array[self.pids[pid][1]][self.size] = data
        return array

    def animate(self):
        self.get_proc()
        for pid, p in self.procs.items():
            try:
                with p.oneshot():
                    cpu_usage = p.cpu_percent()
                    if cpu_usage > 2400:
                        print(f"Error : High Cpu Usage for {p.name()}")
                        cpu_usage = 2400
                    ram_usage = p.memory_info().rss / (1024*1024)
            except:
                cpu_usage = 0.0
                ram_usage = 0.0
            finally:
                self.cpu = self.add(self.cpu,"cpu", pid, cpu_usage)
                self.ram = self.add(self.ram,"ram", pid, ram_usage)#
        self.size += 1



    def dump_values(self):
        print("dump_values")
        self.name = {}
        for pid, (name, _) in self.pids.items():
            new_p = name
            counter = 0
            while new_p in self.name.keys():
                counter += 1
                new_p = new_p + "_" + str(counter)
            if counter != 0:
                name = name+"_"+str(counter)
            self.name[pid] = name
        with open(self.path + f"/{self.sim_name}/cpu/cpu_{self.idx}.csv", "w") as f:
            for pid, (name, idx) in self.pids.items():
                f.write(name + "," + ",".join(map(str, self.cpu[idx, :self.size].tolist())) + "\n")

        with open(self.path + f"/{self.sim_name}/ram/ram_{self.idx}.csv", "w") as f:
            for pid, (name, idx) in self.pids.items():
                f.write(name + "," + ",".join(map(str, self.ram[idx, :self.size].tolist())) + "\n")

        sys.exit(0)


def run(path, simulator="isaac", idx=0):
    rclpy.init(args=None)
    allowed = ['driver', 'mongod', 'move_group', 'moveit_collision', 'moveit_controller', 'python3', 
                'robot_state_publisher','run_recording', 'rviz2', 'spawner', 'static_transform_publisher', "ros2"]  # Needed since they start before recording starts
    if "isaac" in simulator:
        allowed.extend(["kit"])
    if "webots" in simulator:
        allowed.extend(["webots-bin", "webots"])
    if "ignition" in simulator:
        allowed.extend(["ruby"])
    if "vrep" in simulator:
        allowed.extend(["coppeliaSim", "vrep_control"])
    if "pybullet" in simulator:
        allowed.extend(["panda"])
    monitor = ProcMonitor(allowed, idx, simulator, path)
    signal.signal(signal.SIGINT, lambda sig, frame: monitor.dump_values())
    signal.signal(signal.SIGTERM, lambda sig, frame: monitor.dump_values())

    rclpy.spin(monitor)
    rclpy.shutdown()
