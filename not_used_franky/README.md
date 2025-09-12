

# Installation

Might need linux dependencies, if a fresh install:
```bash
sudo apt install gcc g++ unzip 
```

Installing python robotics toolbox:
```bash 
conda create -n csc376 python==3.12
conda activate csc376
pip install roboticstoolbox-python ruckig
pip install numpy==1.26.4 matplotlib==3.8.4 websockets==13.0.1
```

Install our own version of swift:
```bash
git clone https://github.com/ContinuumRoboticsLab/swift-csc376
cd swift-csc376
pip install -e .
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
