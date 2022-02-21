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
import argparse

import psutil

try:
    import proc_monitor
    import clock_logger
except ModuleNotFoundError:
    from . import proc_monitor
    from . import clock_logger

import _thread
import threading
import re
class Pybullet():
    def __init__(self, gui=False, throw= False):
        self.name = "pybullet" +("_throw" if throw else "") + ("_gui" if gui else "")
        self.timeout = 900 if not throw else 600 # 15 minute
        self.commands = [
            f"ros2 launch pybullet_panda {'throw' if throw else 'stack'}_cubes.launch.py gui:={str(gui).lower()}"
        ]
        self.delays = [5] #added the timer delay from launch file

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
    time.sleep(2)  # Wait for everything ot close to prevent broken_pipe
    for proc in procs[:2]:
        os.kill(proc.pid,signal.SIGINT)
        # proc.terminate()
    for proc in procs[2:]:
        proc.kill()
    time.sleep(2)  # Wait for everything ot close to prevent broken_pipe


# Reference : https://stackoverflow.com/a/40281422
def interrupt_handler(interrupt_event):
    interrupt_event.wait()
    _thread.interrupt_main()


def run_com(w, q, com):
    os.dup2(w.fileno(), 1)
    proc = subprocess.Popen(com, shell=True, env=os.environ)
    q.put(proc.pid)


def run_recorder(q, interrupt_event, simulator, idx, path, clock = False):
    task = threading.Thread(target=interrupt_handler, args=(interrupt_event,))
    task.start()
    try:
        if clock:
            clock_logger.run(simulator=simulator, idx=idx, path=path)
        else:
            proc_monitor.run(simulator=simulator, idx=idx, path=path)
    except Exception as e:
        print(e)
        q.put("Exit")


def generate_procs(simulator, commands, r, w, q, interrupt_event, idx, path):
    procs = []
    procs.append(Process(target=run_recorder, args=(
        q, interrupt_event, simulator, idx, path), daemon=True, name="Recorder"))
    procs.append(Process(target=run_recorder, args=(
        q, interrupt_event, simulator, idx, path, True), daemon=True, name="Clock"))
    for com in commands:
        procs.append(Process(target=run_com, args=(w, q, com), name=com))
    return procs


def start_proces(delay, procs, q):
    pids = []
    for _ in range(len(procs) - len(delay)):
        delay.insert(0,0)
        delay.insert(0,0)
    for idx, p in enumerate(procs):
        p.start()
        time.sleep(delay[idx])

    for proc in range(len(procs)-2):
        pids.append(q.get())

    return pids

def handler(signum, frame):
    raise Exception("Timeout")

def log(file, msg):
    print(msg)
    file.write(msg)
    
def run(sim, idx, path):
    r, w = Pipe()
    q = Queue()
    reader = os.fdopen(r.fileno(), 'r')
    interrupt_event = Event()
    procs = generate_procs(sim.name, sim.commands, r,
                           w, q, interrupt_event, idx, path)
    time.sleep(1)
    start_time = time.time()
    pids = start_proces(sim.delays, procs, q)
    signal.signal(signal.SIGALRM, handler)
    signal.alarm(sim.timeout)

    with open(path+f"/{sim.name}/log/{idx}.txt", "w") as f,\
            open(path+f"/{sim.name}/run.txt", "a") as out:
        try:
            while True:
                text = reader.readline()
                f.write(text)
                if "Starting timer" in text:
                    start_exec_time = time.time() - start_time
                if "Task finished executing in" in text: 
                    end_time = [int(s) for s in re.findall(r'\b\d+\b', text)][-1]
                    log(out, f"Completed for {idx}: Total execution time {(time.time()-start_time)*1000:.0f} ms. Task started {start_exec_time*1000:.0f} ms after start and took {end_time} ms")
                if "cubes placed correctly" in text or "cubes moved out" in text:
                    log(out, text.split(":")[-1])
                    signal.alarm(0) 
                    kill_proc_tree(pids, procs, interrupt_event)
                    return 0
        except:
            log(out,f"Timeout for {idx} after {time.time()-start_time} \n")
            signal.alarm(0) 
            kill_proc_tree(pids, procs, interrupt_event)
            return 1


def main(args=None):
    fail = 0

    parser = argparse.ArgumentParser(description='Sim recorder parameters')
    parser.add_argument("-i", '--iterations', type=int, default=1,
                        help='Number of iterations of the simulation')
    parser.add_argument("-s", '--start-index', type=int, default=1,
                        help='Allow to start the simulation at a different index then 1')
    parser.add_argument('--headless', action='store_true',
                        help='Whetever to render to a GUI or not')
    parser.add_argument('-t', '--throw', action='store_true',
                        help='If enabled run the throw simulation')

    args = parser.parse_args()
    gui = True if not args.headless else False
    sim = Pybullet(gui, args.throw)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    path = os.path.join(dir_path, "..")
    try:
        os.mkdir(path+"/data")
    except Exception as e:
        print(f"Error {e}")
    path = os.path.join(dir_path, "..", "data")
    try:
        os.mkdir(path+f"/{sim.name}")
    except Exception as e:
        print(f"Error {e}")
    try:
        os.mkdir(path+f"/{sim.name}/log")
    except Exception as e:
        print(f"Error {e}")
    try:
        os.mkdir(path+f"/{sim.name}/ram")
    except Exception as e:
        print(f"Error {e}")
    try:
        os.mkdir(path+f"/{sim.name}/cpu")
    except Exception as e:
        print(f"Error {e}")
    try:
        os.mkdir(path+f"/{sim.name}/clock")
    except Exception as e:
        print(f"Error {e}")
    if os.path.exists(path+f"/{sim.name}/run.txt") and args.start_index == 1:
        os.remove(path+f"/{sim.name}/run.txt")

    for idx in range(args.start_index, args.iterations+1):
        fail += run(sim, idx, path)
    print(f"Completed {args.iterations-fail}; Timeout {fail}")


if __name__ == "__main__":
    main()
