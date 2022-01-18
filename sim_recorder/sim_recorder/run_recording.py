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

import io
import os
import signal
import subprocess
import sys
import time
from multiprocessing import Event, Pipe, Process, Queue

import psutil

try:
    import proc_monitor
except ModuleNotFoundError:
    from . import proc_monitor

import _thread
import threading
import re
class Webots():
    def __init__(self):
        self.name = "webots"
        self.timeout = 300 # 5 minute
        self.commands = [
            "ros2 launch webots_driver panda.launch.py",
            "ros2 launch webots_driver run_move_group.launch.py",
            "ros2 launch webots_driver collision.launch.py",
            "ros2 launch webots_driver moveit_controller.launch.py",
        ]
        self.delays = [5, 7] # it doesn't matter the timing for the rest it doesn't launch anyway



def kill_proc_tree(pids, procs, interrupt_event, including_parent=False):
    interrupt_event.set()
    for pid in pids:
        try:
            parent = psutil.Process(pid)
            for child in parent.children(recursive=True):
                child.kill()
            if including_parent:
                parent.kill()
        except:
            pass
    time.sleep(5)  # Wait for everything ot close to prevent broken_pipe
    for proc in procs[:-1]:
        proc.kill()
    time.sleep(5)  # Wait for everything ot close to prevent broken_pipe


# Reference : https://stackoverflow.com/a/40281422
def interrupt_handler(interrupt_event):
    interrupt_event.wait()
    _thread.interrupt_main()


def run_com(w, q, com):
    os.dup2(w.fileno(), 1)
    proc = subprocess.Popen(com, shell=True, env=os.environ)
    q.put(proc.pid)


def run_recorder(q, interrupt_event, simulator, idx, path):
    task = threading.Thread(target=interrupt_handler, args=(interrupt_event,))
    task.start()
    try:
        proc_monitor.run(simulator=simulator, idx=idx, path=path)
    except Exception as e:
        print(e)
        q.put("Exit")


def generate_procs(simulator, commands, r, w, q, interrupt_event, idx, path):
    procs = []
    for com in commands:
        procs.append(Process(target=run_com, args=(w, q, com), name=com))
    procs.append(Process(target=run_recorder, args=(
        q, interrupt_event, simulator, idx, path), daemon=True, name="Recorder"))
    return procs


def start_proces(delay, procs, q):
    pids = []
    for _ in range(len(procs) - len(delay)):
        delay.append(0)
    for idx, p in enumerate(procs):
        print(p.name)
        p.start()
        time.sleep(delay[idx])

    for proc in range(len(procs)-1):
        pids.append(q.get())

    return pids

def handler(signum, frame):
    raise Exception("Timeout")

def log(file, msg):
    print(msg)
    file.write(msg + "\n")
    
def run(sim, idx, path):
    r, w = Pipe()
    q = Queue()
    reader = os.fdopen(r.fileno(), 'r')
    interrupt_event = Event()
    procs = generate_procs(sim.name, sim.commands, r,
                           w, q, interrupt_event, idx, path)
    time.sleep(1)
    pids = start_proces(sim.delays, procs, q)
    signal.signal(signal.SIGALRM, handler)
    signal.alarm(sim.timeout)
    with open(path+f"/{sim.name}/log/{idx}.txt", "w") as f,\
            open(path+f"/{sim.name}/run.txt", "a") as out:
        try:
            while True:
                text = reader.readline()
                f.write(text)
                if "Task executed successfully" in text:
                    timing = [int(s) for s in re.findall(r'\b\d+\b', text)][-1]
                    log(out, f"Completed for {idx} in {timing} ms")
                    signal.alarm(0)
                    kill_proc_tree(pids, procs, interrupt_event)
                    return 1, 0
                if "Task failed" in text: 
                    numbers = [int(s) for s in re.findall(r'\b\d+\b', text)]
                    log(out, f"Failed for {idx} in {numbers[-1]} ms with {numbers[-2]} cube stacked")
                    signal.alarm(0)
                    kill_proc_tree(pids, procs, interrupt_event)
                    return 0, 1
        except:
            log(out,f"Timeout for {idx}")
            f.write("Timeout")
            kill_proc_tree(pids, procs, interrupt_event)
            return 0, 0


def main(args=None):
    succ = 0
    fail = 0

    sim = Webots()

    if len(sys.argv) == 2:
        iteration = int(sys.argv[1])
    else:
        iteration = 1
    dir_path = os.path.dirname(os.path.realpath(__file__))
    path = os.path.join(dir_path, "..", "data")
    try:
        os.mkdir(path+f"/{sim.name}")
        os.mkdir(path+f"/{sim.name}/log")
        os.mkdir(path+f"/{sim.name}/ram")
        os.mkdir(path+f"/{sim.name}/cpu")
    except Exception as e:
        print(e)
        print("Folder exist. Overwriting...")
    if os.path.exists(path+f"/{sim.name}/run.txt"):
        os.remove(path+f"/{sim.name}/run.txt")

    for idx in range(1, iteration+1):
        a, b = run(sim, idx, path)
        succ += a
        fail += b
    print(f"Success {succ}; Failure {fail}; Timeout {iteration-(succ + fail)}")


if __name__ == "__main__":
    main()
