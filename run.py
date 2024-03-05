import os
import subprocess
import signal
import time
from pathlib import Path

roscore, websocket = None, None

def init():
    global roscore, websocket
    roscore = subprocess.Popen(['roscore'])
    time.sleep(0.5)
    websocket = subprocess.Popen(['/root/ut_automata/bin/websocket'])
    time.sleep(0.5)

def cleanup():
    global roscore, websocket
    if roscore:
        roscore.send_signal(signal.SIGINT)
    if websocket:
        websocket.send_signal(signal.SIGINT)

def run_config(config_path: Path):
    print(f"cp {config_path} config/particle_filter.lua")
    os.system(f"cp {config_path} config/particle_filter.lua")
    time.sleep(0.5)

    print(f"./bin/particle_filter > logs/{config_path.stem}.txt")
    particle_filter = subprocess.Popen(['./bin/particle_filter'], stdout=open(f'logs/{config_path.stem}.txt', 'w'))
    time.sleep(0.5)

    print("rosbag play GDC3_easy3.bag --topics /scan /odom /set_pose")
    rosbag = subprocess.Popen(['rosbag', 'play', 'GDC3_easy3.bag', '--topics', '/scan', '/odom', '/set_pose'])
    rosbag.wait()
    time.sleep(0.5)

    print("Interrupt particle_filter")
    particle_filter.send_signal(signal.SIGINT)

init()
config_dir = Path("configs")
for config in config_dir.glob("*.lua"):
    run_config(config)
cleanup()
