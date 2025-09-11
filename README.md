# csc376

# Installation

Might need linux dependencies, if a fresh install:
```bash
sudo apt install gcc g++ unzip 
```

Installing python robotics toolbox:
```bash 
conda create -n csc376 python==3.10
conda activate csc376
pip install roboticstoolbox-python ruckig
pip install websockets==10.4 # Fix asyncio run error
pip install numpy==1.25.0 # Fix numpy > 2 error
```

Installing franky:
```bash
conda activate csc376
pip uninstall franky-control # (added by us) remove previously installed
VERSION=0-9-2 # (added by us) Teachinglab frankas use 0.9.2
wget https://github.com/TimSchneider42/franky/releases/latest/download/libfranka_${VERSION}_wheels.zip
unzip libfranka_${VERSION}_wheels.zip
# pip install numpy # (added by us) Should already be done from above
pip install --no-index --find-links=./dist franky-control
```

If outdated, the above was taken and modified from: https://github.com/TimSchneider42/franky?tab=readme-ov-file#installing-franky. 

# Run

Prerequisites:
1. Make sure you are running a linux real-time kernel: `uname -r` should show "rt" in the output
2. Robot will move forward 10cm, please put in a possible position
3. Put the Franka robot to FCI mode in Franka Desk

```bash
conda activate csc376
python3 test_visualization_then_execute.py  
```

# Install csc376_franky

```bash
sudo apt -y install python3-pybind11
pip install pybind11
conda activate csc376
cd csc376_franky
pip install .
```

# Realtime Kernel

sudo nano /etc/security/limits.conf

@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400

sudo groupadd -r realtime
sudo usermod -aG realtime $USER
